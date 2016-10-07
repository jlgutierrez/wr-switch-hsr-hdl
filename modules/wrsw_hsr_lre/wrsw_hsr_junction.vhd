-------------------------------------------------------------------------------
-- Title      : HSR Link Redundancy Entity - Junction
-- Project    : White Rabbit
-------------------------------------------------------------------------------
-- File       : wrsw_hsr_junction.vhd
-- Author     : 
-- Company    :  
-- Department : 
-- Created    : 2016-02-22
-- Last update: 2016-02-22
-- Platform   : FPGA-generic
-- Standard   : VHDL '93
-------------------------------------------------------------------------------
-- Description: this module acts as a carrefour where outbound traffic from
-- different data flows is routed and prioritised:
--   - Traffic from one HSR tagger will be sent through both HSR enabled EPs.
--   - Traffic from one Forwarding unit will be sent through the opposite
--     HSR enabled EP.
--   - Priority is given to traffic coming from Forwarding units
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
use work.endpoint_private_pkg.all;
use work.wrsw_hsr_lre_pkg.all;

entity wrsw_hsr_junction is

  generic(
    g_adr_width : integer := 2;
    g_dat_width : integer :=16
    );
  port(
  
	 	rst_n_i			: in	std_logic;
		clk_i				: in	std_logic;
		
		link_ok_i		: in  std_logic_vector(1 downto 0);
		
		-- Towards endpoints Tx
		ep_src_o		: out	t_wrf_source_out_array(1 downto 0);
		ep_src_i		: in	t_wrf_source_in_array(1 downto 0);
		
		-- From hsr taggers
		tagger_snk_i	: in	t_wrf_sink_in_array(1 downto 0);
		tagger_snk_o	: out t_wrf_sink_out_array(1 downto 0);
		
		-- From HSR forwarding units
		fwd_snk_fab_i	: in	t_ep_internal_fabric_array(1 downto 0);
		fwd_snk_dreq_o	: out std_logic_vector(1 downto 0);
		
		-- Stats counters
	   bound_ep0_count_o		: out std_logic_vector(31 downto 0);
	   bound_ep1_count_o		: out std_logic_vector(31 downto 0);
	   dup_ep0_count_o		: out std_logic_vector(31 downto 0);
	   dup_ep1_count_o		: out std_logic_vector(31 downto 0);
      clr_cnt_i 				: in std_logic
		

    );
end wrsw_hsr_junction;

architecture behavioral of wrsw_hsr_junction is
  
  component ep_rx_wb_master
    generic (
      g_ignore_ack   : boolean;
      g_cyc_on_stall : boolean := false);
    port (
      clk_sys_i  : in  std_logic;
      rst_n_i    : in  std_logic;
      snk_fab_i  : in  t_ep_internal_fabric;
      src_wb_i   : in  t_wrf_source_in;
      src_wb_o   : out t_wrf_source_out;
      snk_dreq_o : out std_logic);
  end component; 
  
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
  
  signal tagger_snk_in		: t_wrf_source_out_array(1 downto 0);
  signal ep_src_in			: t_wrf_source_in_array(1 downto 0);
  
  signal fwd_src_out			: t_wrf_source_out_array(1 downto 0);
  signal fwd_src_in			: t_wrf_source_in_array(1 downto 0);
  
  signal arb_src_out			: t_wrf_source_out_array(1 downto 0);
  signal arb_src_in			: t_wrf_source_in_array(1 downto 0);
  
  signal mux_ep_src_out		: t_wrf_source_out_array_array(1 downto 0);
  signal mux_ep_src_in		: t_wrf_source_in_array_array(1 downto 0);
  
  signal fwd_snk_dreq_out  : std_logic_vector(1 downto 0);
  
  signal ep_src_o_int		: t_wrf_source_out_array(1 downto 0);

  begin --rtl
	
	U_from_taggers : wrsw_hsr_arbfromtaggers
	port map(
		rst_n_i  		=> rst_n_i,
		clk_i				=> clk_i,
		link_ok_i		=> link_ok_i,
		ep_src_o 		=> arb_src_out,
		ep_src_i 		=> arb_src_in,
		tagger_snk_i 	=> tagger_snk_i,
		tagger_snk_o 	=> tagger_snk_o,
		bound_ep0_count_o => bound_ep0_count_o,
		bound_ep1_count_o => bound_ep1_count_o,
		dup_ep0_count_o => dup_ep0_count_o,
		dup_ep1_count_o => dup_ep1_count_o,
		clr_cnt_i		 => clr_cnt_i
	);
	
	gen_ep_master : for i in 0 to 1 generate

		U_ep_master : ep_rx_wb_master
		generic map(
			g_ignore_ack	=> true,
			g_cyc_on_stall => false)
		port map(
			clk_sys_i 		=> clk_i,
			rst_n_i			=> rst_n_i,
			snk_fab_i		=> fwd_snk_fab_i(i),
			snk_dreq_o		=> fwd_snk_dreq_out(i),
			
			src_wb_o			=> fwd_src_out(i),
			src_wb_i			=>	fwd_src_in(i)
		);

	end generate;
	
	fwd_snk_dreq_o <= fwd_snk_dreq_out;
	
	gen_mux : for i in 0 to 1 generate
	
		U_mux_ep_x : xhsr_mux
		port map(
			clk_sys_i   => clk_i,
			rst_n_i     => rst_n_i,
    
			ep_src_o    => ep_src_o_int(i),
			ep_src_i    => ep_src_i(i),
			mux_snk_o   => mux_ep_src_in(i),
			mux_snk_i   => mux_ep_src_out(i)
    );
	end generate;
	
	ep_src_o <= ep_src_o_int;
	
	mux_ep_src_out(0)(0) <= arb_src_out(0);
	mux_ep_src_out(1)(0) <= arb_src_out(1);
	mux_ep_src_out(0)(1) <= fwd_src_out(1);
	mux_ep_src_out(1)(1) <= fwd_src_out(0);
	
	arb_src_in(0) <= mux_ep_src_in(0)(0);
	arb_src_in(1) <= mux_ep_src_in(1)(0);
	fwd_src_in(1) <= mux_ep_src_in(0)(1);
	fwd_src_in(0) <= mux_ep_src_in(1)(1);
--	
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
	
	trig0(15 downto 0) <= ep_src_o_int(0).dat; -- ! changed
	trig0(31 downto 16) <= arb_src_out(1).dat;
	trig1(15 downto 0) <= fwd_src_out(1).dat;
	trig1(31 downto 16) <= fwd_src_out(0).dat;
	
	trig2(0) <= arb_src_in(0).stall;
	trig2(1) <= arb_src_in(1).stall;
	trig2(2) <= fwd_src_in(0).stall;
	trig2(3) <= fwd_src_in(1).stall;
	
	trig2(4) <= fwd_snk_dreq_out(0);
	trig2(5) <= fwd_snk_dreq_out(1);
	
	trig2(6) <= arb_src_out(0).cyc;
	trig2(7) <= arb_src_out(1).cyc;
	trig2(8) <= fwd_src_out(0).cyc;
	trig2(9) <= fwd_src_out(1).cyc;
	
	trig2(10) <= fwd_snk_fab_i(1).sof;
	trig2(11) <= fwd_snk_fab_i(1).eof;
	trig2(12) <= fwd_snk_fab_i(1).dvalid;
	trig2(28 downto 13) <= fwd_snk_fab_i(1).data;
	trig2(29) <= fwd_src_out(1).stb;
	trig3(15 downto 0) <= fwd_src_out(1).dat;
	trig3(16) <= fwd_src_in(1).ack;
	trig3(17) <= ep_src_o_int(0).stb;
	trig3(18) <= ep_src_o_int(0).cyc;
	trig3(19) <= ep_src_i(0).stall;
	trig3(20) <= ep_src_i(0).ack;
	trig3(22 downto 21) <= fwd_snk_fab_i(1).addr;
	trig3(24 downto 23) <= fwd_src_out(1).adr;
	trig3(26 downto 25) <= ep_src_o_int(0).adr;
	

end behavioral;
