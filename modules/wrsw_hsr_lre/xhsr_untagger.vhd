-------------------------------------------------------------------------------
-- Title      : HSR Link Redundancy Entity - top level
-- Project    : White Rabbit
-------------------------------------------------------------------------------
-- File       : xhsr_untagger.vhd
-- Author     : José Luis Gutiérrez
-- Company    : University of Granada 
-- Department : Computer Architecture and Technology
-- Created    : 2016-08-02
-- Last update: 2016-08-02
-- Platform   : FPGA-generic
-- Standard   : VHDL '93
-------------------------------------------------------------------------------
-- Description: Struct-ized wrapper for WR HSR UNTAGGER ENTITY (HSR-LRE)
-------------------------------------------------------------------------------
--
-- Copyright (c) 2011 - 2012 CERN / BE-CO-HT
--
-- This source file is free software; you can redistribute it   
-- and/or modify it under the terms of the GNU Lesser General   
-- Public License as published by the Free Software Foundation; 
-- either version 2.1 of the License, or (at your option) any   
-- later version.                                               
--
-- This source is distributed in the hope that it will be       
-- useful, but WITHOUT ANY WARRANTY; without even the implied   
-- warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR      
-- PURPOSE.  See the GNU Lesser General Public License for more 
-- details.                                                     
--
-- You should have received a copy of the GNU Lesser General    
-- Public License along with this source; if not, download it   
-- from http://www.gnu.org/licenses/lgpl-2.1.html
--
-------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.CEIL;
use ieee.math_real.log2;

library work;
use work.swc_swcore_pkg.all;
use work.wr_fabric_pkg.all;
use work.wrsw_shared_types_pkg.all;
use work.mpm_pkg.all;
use work.genram_pkg.all;
use work.endpoint_private_pkg.all;




entity xhsr_untagger is
  generic(
    g_adr_width : integer := 2;
    g_dat_width : integer :=16;
    g_size    : integer := 1024; -- things for the fifo
    g_with_fc : boolean := false -- things for the fifo
    --g_num_ports : integer
    );
  port(

    rst_n_i : in  std_logic;
    clk_i   : in  std_logic;
    
-------------------------------------------------------------------------------
-- pWB  : input (comes from HSR_DROPPER)
-------------------------------------------------------------------------------
    snk_i : in  t_wrf_sink_in;
    snk_o : out  t_wrf_sink_out;

-------------------------------------------------------------------------------
-- pWB : output (goes to SWC)
-------------------------------------------------------------------------------  
    src_i : in  t_wrf_source_in;
    src_o : out  t_wrf_source_out

    );
end xhsr_untagger;

architecture behavoural of xhsr_untagger is

type t_state is (WAIT_FRAME, 
					DATA,
					REMOVE_TAG,
					BYPASS_FRAME);

  -- new crap
  signal hdr_offset : std_logic_vector(9 downto 0);
  signal at_ethertype     : std_logic;
  
  -- general signals
  signal state   : t_state;

  component chipscope_icon
    port (
      CONTROL0 : inout std_logic_vector(35 downto 0));
  end component;
  component chipscope_ila
    port (
      CONTROL : inout std_logic_vector(35 downto 0);
      CLK     : in    std_logic;
      TRIG0   : in    std_logic_vector(31 downto 0);
      TRIG1   : in    std_logic_vector(31 downto 0);
      TRIG2   : in    std_logic_vector(31 downto 0);
      TRIG3   : in    std_logic_vector(31 downto 0));
  end component;

  signal CONTROL0 : std_logic_vector(35 downto 0);
  signal TRIG0		: std_logic_vector(31 downto 0);
  signal TRIG1		: std_logic_vector(31 downto 0);
  signal TRIG2		: std_logic_vector(31 downto 0);
  signal TRIG3		: std_logic_vector(31 downto 0);
  
  -- CONTROL SIGNALS --
  signal sof_p1, eof_p1, abort_p1, error_p1 : std_logic;
  signal snk_cyc_d0                         : std_logic;
  signal snk_valid			    : std_logic := '0';
  
  type t_write_state is(WAIT_FRAME, DATA);
  
  -- DEBUG --
  signal debug_state : std_logic_vector (2 downto 0);
  signal fab_2_wb_o : t_wrf_source_out;
  signal ack_counter : integer := 0;
  signal eof_counter : integer := 0;
  signal sof_counter : integer := 0;
  
  begin  -- behavioral
  
  at_ethertype <= hdr_offset(7) and snk_i.stb;

  p_detect_frame : process(clk_i)
  begin
    if rising_edge(clk_i) then
      if rst_n_i = '0' then
        snk_cyc_d0 <= '0';
      else
        snk_cyc_d0 <= snk_i.cyc;
      end if;
    end if;
  end process;

  -- convert wb to fab
  sof_p1 <= not snk_cyc_d0 and snk_i.cyc;
  eof_p1 <= snk_cyc_d0 and not snk_i.cyc;
  
  --snk_valid <= snk_i.cyc and snk_i.stb and snk_i.we and not src_i.stall;
  snk_valid <= snk_i.cyc and snk_i.stb and snk_i.we;

p_untag : process(clk_i)
  begin
    if rising_edge(clk_i) then
      if rst_n_i = '0' then
        hdr_offset(hdr_offset'left downto 1) <= (others => '0');
        hdr_offset(0)    <= '1';

        state            <= WAIT_FRAME;   
        debug_state <= "000";     
      else
           case state is
            when WAIT_FRAME =>
              debug_state <= "001";
              
              if(sof_p1 = '1') then
                hdr_offset(hdr_offset'left downto 1) <= (others => '0');
                hdr_offset(0)                        <= '1';
                state                                <= DATA;
              end if;
              
              src_o.adr <= snk_i.adr;
			  src_o.dat <= snk_i.dat;
			  src_o.cyc <= snk_i.cyc;
			  src_o.stb <= snk_i.stb;
			  src_o.we <= snk_i.we;
			  src_o.sel <= snk_i.sel;

            when DATA =>
			  debug_state <= "010";

              if(at_ethertype = '1') then

                if(snk_i.dat = x"892f") then  -- got a HSR tagged frame, and it should be removed... (3 cycles)
                  -- we have already removed these data (tag) (1st removed)
                  src_o.adr <= snk_i.adr;
				  src_o.dat <= snk_i.dat;
				  src_o.cyc <= snk_i.cyc;
				  src_o.stb <= '0';
				  src_o.we <= snk_i.we;
				  src_o.sel <= snk_i.sel;
				  
				  state <= REMOVE_TAG;
                else -- just bypass frame 
				  src_o.adr <= snk_i.adr;
				  src_o.dat <= snk_i.dat;
				  src_o.cyc <= snk_i.cyc;
				  src_o.stb <= snk_i.stb;
				  src_o.we <= snk_i.we;
				  src_o.sel <= snk_i.sel;
				  
				  state <= BYPASS_FRAME;
                end if;
              else
                  src_o.adr <= snk_i.adr;
				  src_o.dat <= snk_i.dat;
				  src_o.cyc <= snk_i.cyc;
				  src_o.stb <= snk_i.stb;
				  src_o.we <= snk_i.we;
				  src_o.sel <= snk_i.sel;
              end if;

              if(snk_i.stb = '1') then
                hdr_offset <= hdr_offset(hdr_offset'left-1 downto 0) & '0';
              end if;

			when BYPASS_FRAME =>
				debug_state <= "011";
				src_o.adr <= snk_i.adr;
				src_o.dat <= snk_i.dat;
				src_o.cyc <= snk_i.cyc;
				src_o.stb <= snk_i.stb;
				src_o.we <= snk_i.we;
				src_o.sel <= snk_i.sel;
				 
				if (eof_p1 = '1') then
				   state <= WAIT_FRAME;
				end if;
				
				if (snk_i.stb = '1') then
				  hdr_offset <= hdr_offset(hdr_offset'left-1 downto 0) & '0';
				end if;
			
            when REMOVE_TAG =>
              debug_state <= "100";
				
				if(hdr_offset(8) = '1') then -- 2nd removed
				  src_o.adr <= snk_i.adr;
				  src_o.dat <= snk_i.dat;
				  src_o.cyc <= snk_i.cyc;
				  src_o.stb <= '0';
				  src_o.we <= snk_i.we;
				  src_o.sel <= snk_i.sel;
				end if;
				
				if(hdr_offset(9) = '1') then -- 3rd removed
				  src_o.adr <= snk_i.adr;
				  src_o.dat <= snk_i.dat;
				  src_o.cyc <= snk_i.cyc;
				  src_o.stb <= '0';
				  src_o.we <= snk_i.we;
				  src_o.sel <= snk_i.sel;
				  
				  state <= BYPASS_FRAME; -- bypass the rest of the frame

				end if;
				
				if (snk_i.stb = '1') then
				  hdr_offset <= hdr_offset(hdr_offset'left-1 downto 0) & '0';
				end if;              
          end case;
      end if;
    end if;
  end process;

  --snk_o <= src_i;

p_gen_ack : process(clk_i)
  begin
    if rising_edge(clk_i) then
      --snk_o.ack <= '1'; -- shitty hack (TB FIXED)
	snk_o.ack <= snk_valid;
    end if;
  end process;
  
    -- DEBUG --	
--   cs_icon : chipscope_icon
--   port map(
--      CONTROL0	=> CONTROL0
--   );
--   cs_ila : chipscope_ila
--   port map(
--      CLK	=> clk_i,
--      CONTROL	=> CONTROL0,
--      TRIG0	=> TRIG0,
--      TRIG1	=> TRIG1,
--      TRIG2	=> TRIG2,
--      TRIG3	=> TRIG3
--   );

trig0(2 downto 0) <= debug_state;
trig0(3) <= sof_p1;
trig0(4) <= eof_p1;
trig0(25 downto 10) <= snk_i.dat;
trig0(26) <= snk_i.cyc;
trig0(27) <= snk_i.stb;

trig2(9 downto 0) <= hdr_offset;	

	


end behavoural;
