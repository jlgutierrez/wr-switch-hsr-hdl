-------------------------------------------------------------------------------
-- Title      : HSR LRE top level package
-- Project    : White Rabbit Switch
-------------------------------------------------------------------------------
-- File       : wrsw_hsr_lre_pkg.vhd
-- Author     : José Luis Gutiérrez
-- Company    : University of Granada
-- Created    : 2016-02-08
-- Last update: 2016-02-08
-- Platform   : FPGA-generic
-- Standard   : VHDL
-------------------------------------------------------------------------------
--
-- Copyright (c) 2012 - 2014 CERN / BE-CO-HT
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
use ieee.STD_LOGIC_1164.all;

use work.wr_fabric_pkg.all;
use work.wishbone_pkg.all;
use work.wrsw_txtsu_pkg.all;
use work.wrsw_shared_types_pkg.all;
use work.endpoint_pkg.all;
use work.endpoint_private_pkg.all;
use work.lre_wbgen2_pkg.all;


package wrsw_hsr_lre_pkg is

  type t_ep_internal_fabric_array is array (natural range <>) of t_ep_internal_fabric;
  
  constant c_dummy_snk_in_array : t_wrf_sink_in_array (1 downto 0) :=(("XX", "XXXXXXXXXXXXXXXX", '0', '0', '0', "XX"),
    ("XX", "XXXXXXXXXXXXXXXX", '0', '0', '0', "XX"));
	 
  constant c_dummy_src_in_array : t_wrf_source_in_array(1 downto 0) := (('0', '0', '0', '0'),('0', '0', '0', '0'));
	 
  type t_wrf_source_out_array_array is array (natural range <>) of t_wrf_source_out_array(1 downto 0);
  type t_wrf_source_in_array_array is array (natural range <>) of t_wrf_source_in_array(1 downto 0);
  


  component xhsr_tagger
    generic (
	  g_adr_width : integer := 2;
	  g_dat_width : integer :=16
	  --g_num_ports : integer
	  );
    port (

    rst_n_i     : in  std_logic;
    clk_i	: in  std_logic;
    req_tag     : out std_logic;
    seq_n       : in std_logic_vector (15 downto 0);
    seq_valid   : in std_logic;
    snk_i	: in  t_wrf_sink_in;
    snk_o 	: out  t_wrf_sink_out;
    src_i 	: in  t_wrf_source_in;
    src_o 	: out  t_wrf_source_out);
  end component;
  
  component xhsr_tagger_debug
    generic (
	  g_adr_width : integer := 2;
	  g_dat_width : integer :=16
	  --g_num_ports : integer
	  );
    port (

    rst_n_i     : in  std_logic;
    clk_i	: in  std_logic;
    req_tag     : out std_logic;
    seq_n       : in std_logic_vector (15 downto 0);
    seq_valid   : in std_logic;
    snk_i	: in  t_wrf_sink_in;
    snk_o 	: out  t_wrf_sink_out;
    src_i 	: in  t_wrf_source_in;
    src_o 	: out  t_wrf_source_out);
  end component;  

  component xhsr_seq
    generic (
      g_dat_width : integer :=16);
    port (
      rst_n_i : in  std_logic;
      clk_i	: in  std_logic;
      request0 : in std_logic;
      request1 : in std_logic;
      seq_n0 : out std_logic_vector (15 downto 0);
      seq_n1 : out std_logic_vector (15 downto 0);
      valid0 : out std_logic;
      valid1 : out std_logic);
  end component;
  
  component xhsr_untagger
    generic (
	  g_adr_width : integer := 2;
	  g_dat_width : integer :=16
	  --g_num_ports : integer
	  );
    port (

    rst_n_i : in  std_logic;
    clk_i	: in  std_logic;
    snk_i	: in  t_wrf_sink_in;
    snk_o 	: out  t_wrf_sink_out;
    src_i 	: in  t_wrf_source_in;
    src_o 	: out  t_wrf_source_out);
  end component;	 
  
  component xhsr_fwd
   generic(
    g_adr_width : integer := 2;
    g_dat_width : integer :=16;
    g_size    : integer := 1520; -- things for the fifo
    g_with_fc : boolean := false -- things for the fifo
    );
  port(

    rst_n_i : in  std_logic;
    clk_i   : in  std_logic;

    snk_i : in  t_wrf_sink_in;
    snk_o : out  t_wrf_sink_out;

    src_i : in  t_wrf_source_in;
    src_o : out  t_wrf_source_out;
    
	 fwd_dreq_i : in  std_logic;
    fwd_fab_o : out  t_ep_internal_fabric;
    
	 mac_addr_i : in std_logic_vector(47 downto 0);
	 fwd_count_o : out std_logic_vector(31 downto 0);
	 
	 link_ok_i : in std_logic;
	 clr_cnt_i : in std_logic

    );
  end component;
  
  
  component xhsr_fwd_debug
   generic(
    g_adr_width : integer := 2;
    g_dat_width : integer :=16;
    g_size    : integer := 1520; -- things for the fifo
    g_with_fc : boolean := false -- things for the fifo
    );
  port(

    rst_n_i : in  std_logic;
    clk_i   : in  std_logic;

    snk_i : in  t_wrf_sink_in;
    snk_o : out  t_wrf_sink_out;

    src_i : in  t_wrf_source_in;
    src_o : out  t_wrf_source_out;
    
	 fwd_dreq_i : in  std_logic;
    fwd_fab_o : out  t_ep_internal_fabric;
    
	 mac_addr_i : in std_logic_vector(47 downto 0)

    );
  end component;
  
  
  
  component wrsw_hsr_junction
	generic(
		g_adr_width : integer := 2;
		g_dat_width : integer := 16
		);
	port(
		rst_n_i			: in	std_logic;
		clk_i				: in	std_logic;
		
		link_ok_i		: in	std_logic_vector(1 downto 0);
		
		ep_src_o		: out	t_wrf_source_out_array(1 downto 0);
		ep_src_i		: in	t_wrf_source_in_array(1 downto 0);

		tagger_snk_i	: in	t_wrf_sink_in_array(1 downto 0);
		tagger_snk_o	: out t_wrf_sink_out_array(1 downto 0);
		
		fwd_snk_fab_i	: in	t_ep_internal_fabric_array(1 downto 0);
		fwd_snk_dreq_o	: out std_logic_vector(1 downto 0);
		
		-- Stats counters
	   bound_ep0_count_o		: out std_logic_vector(31 downto 0);
	   bound_ep1_count_o		: out std_logic_vector(31 downto 0);
	   dup_ep0_count_o		: out std_logic_vector(31 downto 0);
	   dup_ep1_count_o		: out std_logic_vector(31 downto 0);
      clr_cnt_i : in std_logic

		);
	end component;
	
  component wrsw_hsr_arbfromtaggers
	port(
		rst_n_i			: in	std_logic;
		clk_i				: in	std_logic;
		
		link_ok_i		: in	std_logic_vector(1 downto 0);
		
		-- Towards endpoints Tx
		ep_src_o		: out	t_wrf_source_out_array(1 downto 0);
		ep_src_i		: in	t_wrf_source_in_array(1 downto 0);
		
		-- From hsr taggers
		tagger_snk_i	: in	t_wrf_sink_in_array(1 downto 0);
		tagger_snk_o	: out t_wrf_sink_out_array(1 downto 0);
		
		-- Stats counters
	   bound_ep0_count_o		: out std_logic_vector(31 downto 0);
	   bound_ep1_count_o		: out std_logic_vector(31 downto 0);
	   dup_ep0_count_o		: out std_logic_vector(31 downto 0);
	   dup_ep1_count_o		: out std_logic_vector(31 downto 0);
      clr_cnt_i : in std_logic
	);
	end component;
	
  component xhsr_mux
  port(
    clk_sys_i   : in  std_logic;
    rst_n_i     : in  std_logic;
    --ENDPOINT
    ep_src_o    : out t_wrf_source_out;
    ep_src_i    : in  t_wrf_source_in;
        --Muxed ports
    mux_snk_o   : out t_wrf_sink_out_array(1 downto 0);
    mux_snk_i   : in  t_wrf_sink_in_array(1 downto 0)
    );
  end component;
  
  component xhsr_dropper
  generic(
    g_ring_size : integer := 32;
    g_dat_width : integer :=16;
    g_size    : integer := 1024; 
    g_with_fc : boolean := false 
    );
  port(

    rst_n_i : in  std_logic;
    clk_i   : in  std_logic;
    
-------------------------------------------------------------------------------
-- pWB  : input (comes from ENDPOINT)
-------------------------------------------------------------------------------

    snk_i : in  t_wrf_sink_in_array(1 downto 0);
    snk_o : out  t_wrf_sink_out_array(1 downto 0);

-------------------------------------------------------------------------------
-- pWB : output (goes to UNTAGGER MODULE)
-------------------------------------------------------------------------------  

    src_i : in  t_wrf_source_in_array(1 downto 0);
    src_o : out  t_wrf_source_out_array(1 downto 0);

	 mac_addr_i : in std_logic_vector(47 downto 0);
	 
	 wb_adr_i                                 : in     std_logic_vector(6 downto 0);
	 wb_dat_i                                 : in     std_logic_vector(31 downto 0);
	 wb_dat_o                                 : out    std_logic_vector(31 downto 0);
	 wb_cyc_i                                 : in     std_logic;
	 wb_sel_i                                 : in     std_logic_vector(3 downto 0);
	 wb_stb_i                                 : in     std_logic;
	 wb_we_i                                  : in     std_logic;
	 wb_ack_o                                 : out    std_logic;
	 wb_stall_o                               : out    std_logic	 
	 
	
    );
   end component;
	
	component xhsr_dropper_mem
	generic(
    g_adr_width : integer := 2;
    g_dat_width : integer :=16
    );
   port(
  
	 	rst_n_i			: in	std_logic;
		clk_i				: in	std_logic;
		
		smac_a_i, smac_b_i			: in std_logic_vector(47 downto 0);
		seqnum_a_i, seqnum_b_i		: in std_logic_vector(15 downto 0);
		rd_sig_a_i, rd_sig_b_i		: in std_logic;
		wr_sig_a_i, wr_sig_b_i		: in std_logic;
		nodeid_a_i, nodeid_b_i		: in std_logic_vector(4 downto 0);
		
		nodeid_a_o, nodeid_b_o		: out std_logic_vector(4 downto 0);
		seqnum_a_o, seqnum_b_o		: out std_logic_vector(15 downto 0);
		valid_a_o, valid_b_o			: out std_logic;
		match_a_o, match_b_o			: out std_logic;
		mem_busy_o						: out std_logic;
		
		senaldebug_o					: out std_logic_vector(7 downto 0)

    );
    end component;
  
	component hsr_lre_regs
	  port (
		 rst_n_i                                  : in     std_logic;
		 clk_sys_i                                : in     std_logic;
		 wb_adr_i                                 : in     std_logic_vector(3 downto 0);
		 wb_dat_i                                 : in     std_logic_vector(31 downto 0);
		 wb_dat_o                                 : out    std_logic_vector(31 downto 0);
		 wb_cyc_i                                 : in     std_logic;
		 wb_sel_i                                 : in     std_logic_vector(3 downto 0);
		 wb_stb_i                                 : in     std_logic;
		 wb_we_i                                  : in     std_logic;
		 wb_ack_o                                 : out    std_logic;
		 wb_stall_o                               : out    std_logic;
		 regs_i                                   : in     t_lre_in_registers;
		 regs_o                                   : out    t_lre_out_registers
	  );
	end component;  

end wrsw_hsr_lre_pkg;
