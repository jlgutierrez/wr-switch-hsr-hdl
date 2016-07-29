-------------------------------------------------------------------------------
-- Title      : HSR Forwarding Module
-- Project    : White Rabbit
-------------------------------------------------------------------------------
-- File       : xhsr_fwd.vhd
-- Author     : José Luis Gutiérrez
-- Company    : University of Granada 
-- Department : Computer Architecture and Technology
-- Created    : 2016-01-18
-- Last update: 2016-01-18
-- Platform   : FPGA-generic
-- Standard   : VHDL '93
-------------------------------------------------------------------------------
-- Description: Forwarding module.
-- It forwards input from endpoint to the HSR Arbiter after checking if the
-- frame hasn't been already forwarded.
-- PTP Frames are not forwarded using this module, it is done in software (ppsi)
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
use work.genram_pkg.all;
use work.swc_swcore_pkg.all;
use work.wr_fabric_pkg.all;
use work.endpoint_private_pkg.all;
use work.wrsw_shared_types_pkg.all;
use work.mpm_pkg.all;

entity xhsr_fwd is
  generic(
    g_adr_width : integer := 2;
    g_dat_width : integer :=16;
    g_size    : integer := 1024; -- things for the fifo
    g_with_fc : boolean := false -- things for the fifo
    );
  port(

    rst_n_i : in  std_logic;
    clk_i   : in  std_logic;
    
-------------------------------------------------------------------------------
-- pWB  : input (comes from ENDPOINT)
-------------------------------------------------------------------------------

    snk_i : in  t_wrf_sink_in;
    snk_o : out  t_wrf_sink_out;

-------------------------------------------------------------------------------
-- pWB : output (goes to DROPPING/UNTAGGER MODULE)
-------------------------------------------------------------------------------  

    src_i : in  t_wrf_source_in;
    src_o : out  t_wrf_source_out;
    
-------------------------------------------------------------------------------
-- pWB : output (goes to TX HSR ARBITER)
-------------------------------------------------------------------------------  

	fwd_dreq_i : in  std_logic;
   fwd_fab_o : out  t_ep_internal_fabric

    );
end xhsr_fwd;

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.CEIL;
use ieee.math_real.log2;

library work;
use work.genram_pkg.all;
use work.swc_swcore_pkg.all;
use work.wr_fabric_pkg.all;
use work.endpoint_private_pkg.all;
use work.wrsw_shared_types_pkg.all;
use work.mpm_pkg.all;

entity xhsr_fwd_debug is
  generic(
    g_adr_width : integer := 2;
    g_dat_width : integer :=16;
    g_size    : integer := 1024; -- things for the fifo
    g_with_fc : boolean := false -- things for the fifo
    );
  port(

    rst_n_i : in  std_logic;
    clk_i   : in  std_logic;
    
-------------------------------------------------------------------------------
-- pWB  : input (comes from ENDPOINT)
-------------------------------------------------------------------------------

    snk_i : in  t_wrf_sink_in;
    snk_o : out  t_wrf_sink_out;

-------------------------------------------------------------------------------
-- pWB : output (goes to DROPPING/UNTAGGER MODULE)
-------------------------------------------------------------------------------  

    src_i : in  t_wrf_source_in;
    src_o : out  t_wrf_source_out;
    
-------------------------------------------------------------------------------
-- pWB : output (goes to TX HSR ARBITER)
-------------------------------------------------------------------------------  

	fwd_dreq_i : in  std_logic;
   fwd_fab_o : out  t_ep_internal_fabric

    );
end xhsr_fwd_debug;

architecture behavioral of xhsr_fwd is

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
  
  -- CONTROL SIGNALS --
  signal sof, eof : std_logic;
  signal eof_d0	: std_logic;
  signal snk_cyc_d0     : std_logic;
    
  signal snk_valid : std_logic;
  
    -- fifo constants
  constant c_drop_threshold    : integer := g_size - 2;
  constant c_release_threshold : integer := g_size * 7 / 8;
  
  -- fifo signals  
  signal fifo_in, fifo_out             : std_logic_vector(22 downto 0);
  signal fifo_count                 : std_logic_vector(f_log2_size(1520)-1 downto 0);
  signal fifo_empty                 : std_logic;
  signal fifo_rd,fifo_rd_d0         : std_logic;
  signal fifo_wr                    : std_logic;
  signal fifo_aempty, fifo_afull       : std_logic;
  
  signal hdr_done                   : std_logic;
  signal hdr_count                  : std_logic_vector(3 downto 0);
  signal src_mac, dst_mac           : std_logic_vector(47 downto 0);
  signal is_multicast               : std_logic;
  signal is_ptp                     : std_logic;
  signal is_hsr                     : std_logic;
  signal is_other                   : std_logic;
  signal is_vlan_tagged             : std_logic;
  
  type t_fsm_state is (s_IDLE, s_PARSING, s_DROP, s_FORWARD);
  signal state : t_fsm_state;
  
  begin -- behavioral


    -- First of all, the data pipe must go through this module
  -- just like it didn't exist.
  src_o <= snk_i;
  snk_o <= src_i;

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

  sof <= not snk_cyc_d0 and snk_i.cyc;
  eof <= snk_cyc_d0 and not snk_i.cyc;

  snk_valid <= (snk_i.cyc and snk_i.stb and snk_i.we) and not src_i.stall;

  --decoded_status <= f_unmarshall_wrf_status(wb_snk_i.dat);

  --error_p1 <= snk_valid and b2s(snk_i.adr = c_WRF_STATUS) and decoded_status.error;

  --type t_write_state is(WAIT_FRAME, DATA);
  
 

  BUF_FIFO : generic_sync_fifo
    generic map (
      g_data_width => 23,
      g_size       => 1520,--g_size,
      g_with_almost_empty => true,
      g_with_almost_full  => true,
      g_almost_empty_threshold  => 20,--c_release_threshold,
      g_almost_full_threshold   => 1400,--c_drop_threshold,
      g_with_count              => true)--g_with_fc)
    port map (
      rst_n_i        => rst_n_i,
      clk_i          => clk_i,
      d_i            => fifo_in,
      we_i           => fifo_wr,
      q_o            => fifo_out,
      rd_i           => fifo_rd,
      empty_o        => fifo_empty,
      full_o         => open,
      almost_empty_o => fifo_aempty,
      almost_full_o  => fifo_afull,
      count_o        => fifo_count);
    

	-- fifo_wr <= sof or eof or snk_valid;
	
--	p_fifoin : process(sof,eof)
--	begin
--		if sof = '1' then
--			fifo_wr <= '1';
--		elsif eof_d0 = '1' then
--			fifo_wr <= '0';
--		end if;
--	end process;

	fifo_wr <= snk_cyc_d0 or snk_i.cyc;

	
	fifo_in <= sof & eof & snk_valid & snk_i.sel & snk_i.adr & snk_i.dat;
	
	fwd_fab_o.sof <= fifo_out(22) when (fifo_rd_d0 = '1' and (state /= s_DROP)) else '0';
	fwd_fab_o.eof <= fifo_out(21) when (fifo_rd_d0 = '1' and (state /= s_DROP)) else '0';
	fwd_fab_o.dvalid <= fifo_out(20) when (fifo_rd_d0 = '1' and (state /= s_DROP)) else '0';
	fwd_fab_o.bytesel <= not fifo_out(18) when (fifo_rd_d0 = '1' and (state /= s_DROP)) else '0';
	fwd_fab_o.addr <= fifo_out(17 downto 16) when (fifo_rd_d0 = '1' and (state /= s_DROP)) else (others => '0');
	fwd_fab_o.data <= fifo_out(15 downto 0) when (fifo_rd_d0 = '1' and (state /= s_DROP)) else (others => '0');
	
	p_header : process(clk_i)
	
	begin
	
		if(rst_n_i = '0') then
		
		elsif(rising_edge(clk_i)) then
		
			case state is
		
				when s_IDLE =>
					
					hdr_done <= '0';
					hdr_count <= (others => '0');
					src_mac	 <= (others => '0');
					dst_mac	 <= (others => '0');
					is_multicast <= '0';
					is_ptp		 <= '0';
					is_hsr		 <= '0';
					is_other     <= '0';
					is_vlan_tagged <= '0';
					if sof = '1' then
						state <= s_PARSING;
					end if;
					
				when s_PARSING =>
				
					if snk_valid = '1' and snk_i.adr = c_WRF_DATA then
						hdr_count <= std_logic_vector(unsigned(hdr_count) + 1);					
						case hdr_count is
							when x"0" =>
									dst_mac(47 downto 32) <= snk_i.dat;
									if(snk_i.dat(12) = '1') then
										is_multicast <= '1';
									end if;
							when x"1" =>
									dst_mac(31 downto 16) <= snk_i.dat;
							when x"2" =>
									dst_mac(15 downto 0) <= snk_i.dat;
							when x"3" =>
									src_mac(47 downto 32) <= snk_i.dat;
							when x"4" =>
									src_mac(31 downto 16) <= snk_i.dat;
							when x"5" =>
									src_mac(15 downto 0) <= snk_i.dat;
							when x"6" =>
									if snk_i.dat = x"892f" then
										is_hsr         <= '1';
								   elsif snk_i.dat = x"88f7" then
										is_ptp         <= '1';
									elsif snk_i.dat = x"8100" then
										is_vlan_tagged <= '1';
									else
										is_other       <= '1';
									end if;
									
									hdr_done <= '1';
									
									-- If multicast and source != my mac, forward.
									if(is_multicast = '1' and src_mac /= x"0800300DECFB") then
										state <= s_FORWARD;
									elsif(dst_mac /= x"0800300DECFB" and src_mac /= x"0800300DECFB") then
										state <= s_FORWARD;
									else
										state <= s_DROP;
									end if;
							
							when others =>
									
						end case;
					end if;
					
				when s_FORWARD =>
					
					-- Previously if EOF:
					if fifo_empty = '1' then
						state <= s_IDLE;
					end if;
				
				when s_DROP =>
				
					-- Previously if EOF:
					if fifo_empty = '1' then
						state <= s_IDLE;
					end if;
					
--				when s_WAIT_EOF =>
--					
--					if eof = '1' then
--						state <= s_IDLE;
--					end if;
				
				when others =>
		
			end case;
		
		end if;
	
	end process;
	
	p_fwd_drop : process(clk_i)
	begin
		if rst_n_i = '0' then
		
		elsif rising_edge(clk_i) then
		
			fifo_rd_d0 <= fifo_rd;
			eof_d0 <= eof;
		
			case state is
			
				when s_IDLE =>
				
					fifo_rd <= '0';
				
				when s_PARSING =>
					
					fifo_rd <= '0';
				
				when s_FORWARD =>
				
					fifo_rd <= fwd_dreq_i and not(fifo_empty);
				
				when s_DROP =>
				
					fifo_rd <= not(fifo_empty);
				
				when others =>
				
			end case;
		
		end if;
	end process;	
	

end behavioral;

architecture behavioral_debug of xhsr_fwd_debug is

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
  signal TRIG0   :  std_logic_vector(31 downto 0);
  signal TRIG1   :  std_logic_vector(31 downto 0);
  signal TRIG2   :  std_logic_vector(31 downto 0);
  signal TRIG3   :  std_logic_vector(31 downto 0);
  
  -- CONTROL SIGNALS --
  signal sof, eof : std_logic;
  signal eof_d0	: std_logic;
  signal snk_cyc_d0     : std_logic;
    
  signal snk_valid : std_logic;
  
    -- fifo constants
  constant c_drop_threshold    : integer := g_size - 2;
  constant c_release_threshold : integer := g_size * 7 / 8;
  
  -- fifo signals  
  signal fifo_in, fifo_out             : std_logic_vector(22 downto 0);
  signal fifo_count                 : std_logic_vector(f_log2_size(1520)-1 downto 0);
  signal fifo_empty, fifo_full      : std_logic;
  signal fifo_rd,fifo_rd_d0         : std_logic;
  signal fifo_wr                    : std_logic;
  signal fifo_aempty, fifo_afull       : std_logic;
  
  signal hdr_done                   : std_logic;
  signal hdr_count                  : std_logic_vector(3 downto 0);
  signal src_mac, dst_mac           : std_logic_vector(47 downto 0);
  signal is_multicast               : std_logic;
  signal is_ptp                     : std_logic;
  signal is_hsr                     : std_logic;
  signal is_other                   : std_logic;
  signal is_vlan_tagged             : std_logic;
  
  type t_fsm_state is (s_IDLE, s_PARSING, s_DROP, s_FORWARD);
  signal state : t_fsm_state;  
  
  begin -- behavioral

    -- First of all, the data pipe must go through this module
  -- just like it didn't exist.
  src_o <= snk_i;
  snk_o <= src_i;

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

  sof <= not snk_cyc_d0 and snk_i.cyc;
  eof <= snk_cyc_d0 and not snk_i.cyc;

  snk_valid <= (snk_i.cyc and snk_i.stb and snk_i.we) and not src_i.stall;

  --decoded_status <= f_unmarshall_wrf_status(wb_snk_i.dat);

  --error_p1 <= snk_valid and b2s(snk_i.adr = c_WRF_STATUS) and decoded_status.error;

  --type t_write_state is(WAIT_FRAME, DATA);
  
 

  BUF_FIFO : generic_sync_fifo
    generic map (
      g_data_width => 23,
      g_size       => 1520,--g_size,
      g_with_almost_empty => true,
      g_with_almost_full  => true,
      g_almost_empty_threshold  => 20,--c_release_threshold,
      g_almost_full_threshold   => 1400,--c_drop_threshold,
      g_with_count              => true)--g_with_fc)
    port map (
      rst_n_i        => rst_n_i,
      clk_i          => clk_i,
      d_i            => fifo_in,
      we_i           => fifo_wr,
      q_o            => fifo_out,
      rd_i           => fifo_rd,
      empty_o        => fifo_empty,
      full_o         => open,
      almost_empty_o => fifo_aempty,
      almost_full_o  => fifo_afull,
      count_o        => fifo_count);
    

	-- fifo_wr <= sof or eof or snk_valid;
	
--	p_fifoin : process(sof,eof)
--	begin
--		if sof = '1' then
--			fifo_wr <= '1';
--		elsif eof_d0 = '1' then
--			fifo_wr <= '0';
--		end if;
--	end process;

	fifo_wr <= snk_cyc_d0 or snk_i.cyc;
	
	fifo_in <= sof & eof & snk_valid & snk_i.sel & snk_i.adr & snk_i.dat;
	
	fwd_fab_o.sof <= fifo_out(22) when (fifo_rd_d0 = '1' and (state /= s_DROP)) else '0';
	fwd_fab_o.eof <= fifo_out(21) when (fifo_rd_d0 = '1' and (state /= s_DROP)) else '0';
	fwd_fab_o.dvalid <= fifo_out(20) when (fifo_rd_d0 = '1' and (state /= s_DROP)) else '0';
	fwd_fab_o.bytesel <= not fifo_out(18) when (fifo_rd_d0 = '1' and (state /= s_DROP)) else '0';
	fwd_fab_o.addr <= fifo_out(17 downto 16) when (fifo_rd_d0 = '1' and (state /= s_DROP)) else (others => '0');
	fwd_fab_o.data <= fifo_out(15 downto 0) when (fifo_rd_d0 = '1' and (state /= s_DROP)) else (others => '0');
	
	p_header : process(clk_i)
	
	begin
	
		if(rst_n_i = '0') then
		
		elsif(rising_edge(clk_i)) then
		
			case state is
		
				when s_IDLE =>
					
					hdr_done <= '0';
					hdr_count <= (others => '0');
					src_mac	 <= (others => '0');
					dst_mac	 <= (others => '0');
					is_multicast <= '0';
					is_ptp		 <= '0';
					is_hsr		 <= '0';
					is_other     <= '0';
					is_vlan_tagged <= '0';
					if sof = '1' then
						state <= s_PARSING;
					end if;
					
				when s_PARSING =>
				
					if snk_valid = '1' and snk_i.adr = c_WRF_DATA then
						hdr_count <= std_logic_vector(unsigned(hdr_count) + 1);					
						case hdr_count is
							when x"0" =>
									dst_mac(47 downto 32) <= snk_i.dat;
									if(snk_i.dat(12) = '1') then
										is_multicast <= '1';
									end if;
							when x"1" =>
									dst_mac(31 downto 16) <= snk_i.dat;
							when x"2" =>
									dst_mac(15 downto 0) <= snk_i.dat;
							when x"3" =>
									src_mac(47 downto 32) <= snk_i.dat;
							when x"4" =>
									src_mac(31 downto 16) <= snk_i.dat;
							when x"5" =>
									src_mac(15 downto 0) <= snk_i.dat;
							when x"6" =>
									if snk_i.dat = x"892f" then
										is_hsr         <= '1';
								   elsif snk_i.dat = x"88f7" then
										is_ptp         <= '1';
									elsif snk_i.dat = x"8100" then
										is_vlan_tagged <= '1';
									else
										is_other       <= '1';
									end if;
									
									hdr_done <= '1';
									
									-- If multicast and source != my mac, forward.
									if(is_multicast = '1' and src_mac /= x"0800300DECFB") then
										state <= s_FORWARD;
									elsif(dst_mac /= x"0800300DECFB" and src_mac /= x"0800300DECFB") then
										state <= s_FORWARD;
									else
										state <= s_DROP;
									end if;
							
							when others =>
									
						end case;
					end if;
					
				when s_FORWARD =>
					
					-- Previously if EOF:
					if fifo_empty = '1' then
						state <= s_IDLE;
					end if;
				
				when s_DROP =>
				
					-- Previously if EOF:
					if fifo_empty = '1' then
						state <= s_IDLE;
					end if;
					
--				when s_WAIT_EOF =>
--					
--					if eof = '1' then
--						state <= s_IDLE;
--					end if;
				
				when others =>
		
			end case;
		
		end if;
	
	end process;
	
	p_fwd_drop : process(clk_i)
	begin
		if rst_n_i = '0' then
		
		elsif rising_edge(clk_i) then
		
			fifo_rd_d0 <= fifo_rd;
			eof_d0 <= eof;
		
			case state is
			
				when s_IDLE =>
				
					fifo_rd <= '0';
				
				when s_PARSING =>
					
					fifo_rd <= '0';
				
				when s_FORWARD =>
				
					fifo_rd <= fwd_dreq_i and not(fifo_empty);
				
				when s_DROP =>
				
					fifo_rd <= not(fifo_empty);
				
				when others =>
				
			end case;
		
		end if;
	end process;	
  
  
--  	-- DEBUG --
--	cs_icon : chipscope_icon
--	port map(
--		CONTROL0	=> CONTROL0
--	);
--	cs_ila : chipscope_ila
--	port map(
--		CLK		=> clk_i,
--		CONTROL	=> CONTROL0,
--		TRIG0		=> TRIG0,
--		TRIG1		=> TRIG1,
--		TRIG2		=> TRIG2,
--		TRIG3		=> TRIG3
--	);
	
	trig0(0) <= sof;
	trig0(1) <= eof;
	trig0(2) <= snk_valid;
	trig0(3) <= fifo_rd;
	trig0(4) <= fifo_wr;
	trig0(5) <= fifo_empty;
	trig0(6) <= fifo_full;	
	trig0(7) <= snk_i.cyc;
	trig0(8) <= snk_i.stb;
	trig0(9) <= src_i.stall;
	trig0(25 downto 10) <= snk_i.dat;
	trig0(26) <= is_multicast;
	trig0(27) <= is_ptp;
	trig0(28) <= is_hsr;
	trig0(29) <= is_other;
	trig0(30) <= hdr_done;
	
	trig1 <= dst_mac(47 downto 16);
	trig2(15 downto 0) <= dst_mac(15 downto 0);
	trig2(31 downto 16) <= src_mac(47 downto 32);
	trig3 <= src_mac(31 downto 0);
	

end behavioral_debug;
