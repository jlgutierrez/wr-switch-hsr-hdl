-------------------------------------------------------------------------------
-- Title      : HSR Link Redundancy Entity - top level
-- Project    : White Rabbit
-------------------------------------------------------------------------------
-- File       : xhsr_untagger_debug.vhd
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




entity xhsr_untagger_debug is
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
end xhsr_untagger_debug;

architecture behavoural of xhsr_untagger_debug is

  signal hdr_offset : std_logic_vector(10 downto 0);
  
  signal src_stored_o : t_wrf_source_out;
  
  signal drop_word : std_logic := '0';
  
  -- fifo constants
  constant c_drop_threshold    : integer := g_size - 2;
  constant c_release_threshold : integer := g_size * 7 / 8;
  
  -- fifo signals  
  signal q_in, q_out             : std_logic_vector(25 downto 0);
  signal q_usedw                 : std_logic_vector(f_log2_size(g_size)-1 downto 0);
  signal q_empty                 : std_logic;
  signal q_reset                 : std_logic;
  signal q_rd                    : std_logic;
  signal q_drop                  : std_logic;
  signal q_in_valid, q_out_valid : std_logic;
  signal q_aempty, q_afull       : std_logic;

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
  signal snk_cyc_d0, snk_cyc_d1, snk_cyc_d2 : std_logic;
  signal snk_valid			    : std_logic := '0';
  signal q_rd_p1			    : std_logic;
  signal src_stall_d0			: std_logic;
  
  -- DEBUG --
  signal debug_state : std_logic_vector (2 downto 0);
  signal fab_2_wb_o : t_wrf_source_out;
  signal ack_counter : integer := 0;
  signal eof_counter : integer := 0;
  signal sof_counter : integer := 0;
  
  begin  -- behavioral

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
  
  p_detect_stall : process(clk_i)
  begin
    if rising_edge(clk_i) then
      if rst_n_i = '0' then
        src_stall_d0 <= '0';
      else
        src_stall_d0 <= src_i.stall;
      end if;
    end if;
  end process;

  sof_p1 <= not snk_cyc_d0 and snk_i.cyc;
  eof_p1 <= snk_cyc_d0 and not snk_i.cyc;
  
  snk_valid <= snk_i.cyc and snk_i.stb and snk_i.we and not src_i.stall;
  
  src_o.adr <= snk_i.adr;
  src_o.dat <= snk_i.dat;
  src_o.cyc <= snk_i.cyc;
  src_o.stb <= snk_i.stb and not drop_word;
  src_o.we <= snk_i.we;
  src_o.sel <= snk_i.sel;

p_untag : process(clk_i)
  begin
    if rising_edge(clk_i) then
      if rst_n_i = '0' then
        hdr_offset(hdr_offset'left downto 1) <= (others => '0');
        hdr_offset(0)    <= '1';
        drop_word <= '0';
      else
         
		if(sof_p1 = '1') then -- fifo's sof
		 hdr_offset(hdr_offset'left downto 1) <= (others => '0');
		 hdr_offset(0)                        <= '1';
		 drop_word <= '0';
		end if;
		
		if (hdr_offset(6) = '1' or hdr_offset(7) = '1' or hdr_offset(8) = '1') then
		   drop_word <= '1';
		else
		   drop_word <= '0';
		end if;
		
		if snk_valid = '1' then 
		   hdr_offset <= hdr_offset(hdr_offset'left-1 downto 0) & '0';
		end if;
            
      end if;
    end if;
  end process;

  --snk_o <= src_i;
  snk_o.ack <= snk_valid;
  snk_o.stall <= src_i.stall;

  
    -- DEBUG --	
   --cs_icon : chipscope_icon
   --port map(
      --CONTROL0	=> CONTROL0
   --);
   --cs_ila : chipscope_ila
   --port map(
      --CLK	=> clk_i,
      --CONTROL	=> CONTROL0,
      --TRIG0	=> TRIG0,
      --TRIG1	=> TRIG1,
      --TRIG2	=> TRIG2,
      --TRIG3	=> TRIG3
   --);

trig0(0) <= src_stall_d0;
trig0(3) <= sof_p1;
trig0(4) <= eof_p1;
trig0(25 downto 10) <= snk_i.dat;
trig0(26) <= snk_i.cyc;
trig0(27) <= snk_i.stb;
trig0(28) <= src_i.stall;
trig0(29) <= q_empty;
trig0(30) <= q_rd;
trig0(31) <= snk_valid;

trig1(1 downto 0) <= q_out(1 downto 0); --adr
trig1(17 downto 2) <= q_out(17 downto 2); --dat
trig1(18) <= q_out(18); --cyc
trig1(19) <= q_out(19); --stb
trig1(20) <= q_out(20); --we
trig1(22 downto 21) <= q_out(22 downto 21); --sel
trig1(23) <= q_out(23); --sof;
trig1(24) <= q_out(24); --eof;
trig1(25) <= q_out(25); --snk_valid;

trig2(10 downto 0) <= hdr_offset;	

	


end behavoural;
