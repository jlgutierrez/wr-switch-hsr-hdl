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
    fwd_fab_o : out  t_ep_internal_fabric

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
    fwd_fab_o : out  t_ep_internal_fabric

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
		fwd_snk_dreq_o	: out std_logic_vector(1 downto 0));
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
		tagger_snk_o	: out t_wrf_sink_out_array(1 downto 0));
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

end wrsw_hsr_lre_pkg;
