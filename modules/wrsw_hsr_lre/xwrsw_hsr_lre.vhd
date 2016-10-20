-------------------------------------------------------------------------------
-- Title      : HSR Link Redundancy Entity - top level
-- Project    : White Rabbit
-------------------------------------------------------------------------------
-- File       : xwrsw_hsr_lre.vhd
-- Author     : José Luis Gutiérrez
-- Company    : University of Granada 
-- Department : Computer Architecture and Technology
-- Created    : 2016-01-18
-- Last update: 2016-01-18
-- Platform   : FPGA-generic
-- Standard   : VHDL '93
-------------------------------------------------------------------------------
-- Description: Struct-ized wrapper for WR HSR Link Redundancy Entity (HSR-LRE)
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
use work.endpoint_private_pkg.all;
use work.wrsw_shared_types_pkg.all;
use work.mpm_pkg.all;
use work.wrsw_hsr_lre_pkg.all;
use work.wishbone_pkg.all;
use work.lre_wbgen2_pkg.all;


entity xwrsw_hsr_lre is

  generic(
    g_adr_width : integer := 2;
    g_dat_width : integer :=16;
    g_num_ports : integer
    );
  port(

    rst_n_i : in  std_logic;
    clk_i   : in  std_logic;
	 
	 link_ok_i : std_logic_vector(g_num_ports-1 downto 0);

-------------------------------------------------------------------------------
-- pWB  : input (comes from the Endpoint)
-------------------------------------------------------------------------------

    ep_snk_i : in  t_wrf_sink_in_array(g_num_ports-1 downto 0);   -- rx
    ep_src_i : in  t_wrf_source_in_array(g_num_ports-1 downto 0); -- tx
  

-------------------------------------------------------------------------------
-- pWB : output (goes to the Endpoint)
-------------------------------------------------------------------------------  

    ep_snk_o : out t_wrf_sink_out_array(g_num_ports-1 downto 0);   -- rx
    ep_src_o : out t_wrf_source_out_array(g_num_ports-1 downto 0); -- tx
    
-------------------------------------------------------------------------------
-- pWB  : output (goes from SWCORE)
-------------------------------------------------------------------------------

    swc_src_o : out t_wrf_source_out_array(g_num_ports-1 downto 0); -- rx
    swc_snk_o : out t_wrf_sink_out_array(g_num_ports-1 downto 0);   -- tx

-------------------------------------------------------------------------------
-- pWB : input (comes from SWCORE)
-------------------------------------------------------------------------------  

    swc_src_i : in  t_wrf_source_in_array(g_num_ports-1 downto 0);  -- rx
    swc_snk_i : in  t_wrf_sink_in_array(g_num_ports-1 downto 0);     -- tx

-------------------------------------------------------------------------------
-- Wishbone general registers
-------------------------------------------------------------------------------  
	 
	 wb_i		  : in  t_wishbone_slave_in;
	 wb_o		  : out t_wishbone_slave_out;
	 
-------------------------------------------------------------------------------
-- Wishbone dropper memory
-------------------------------------------------------------------------------  
	 
	 mem_wb_i		  : in  t_wishbone_slave_in;
	 mem_wb_o		  : out t_wishbone_slave_out   

    );
end xwrsw_hsr_lre;

architecture behavioral of xwrsw_hsr_lre is

  constant c_NUM_PORTS     : integer := g_num_ports; --GUTI: to be fix
  constant c_hsr_path0     : std_logic_vector(3 downto 0) := "0000"; 
  constant c_hsr_path1     : std_logic_vector(3 downto 0) := "0001";
  
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
  
  signal rst_n    : std_logic;
  
  signal dummy_snk_out : t_wrf_sink_out_array(g_num_ports-1 downto 0);
  signal dummy_src_out : t_wrf_source_out_array(g_num_ports-1 downto 0);
  signal dummy_src_in  : t_wrf_source_in_array(g_num_ports-1 downto 0);
  signal dummy_snk_in  : t_wrf_sink_in_array(g_num_ports-1 downto 0);
  
  signal fwd_fab		  : t_ep_internal_fabric_array(1 downto 0);
  signal fwd_dreq	     : std_logic_vector(1 downto 0);
 
  signal tagger_src_out	: t_wrf_source_out_array(g_num_ports-1 downto 0);
  signal tagger_src_in	: t_wrf_source_in_array(g_num_ports-1 downto 0);
  signal tagger_snk_out	: t_wrf_sink_out_array(g_num_ports-1 downto 0);
  signal tagger_snk_in	: t_wrf_sink_in_array(g_num_ports-1 downto 0);
  
  signal dropper_src_out : t_wrf_source_out_array(g_num_ports-1 downto 0);
  signal dropper_src_in	: t_wrf_source_in_array(g_num_ports-1 downto 0);  
  
  signal ep_snk_out 		: t_wrf_sink_out_array(g_num_ports-1 downto 0);
  signal ep_src_out 		: t_wrf_source_out_array(g_num_ports-1 downto 0);
  signal swc_src_out		: t_wrf_source_out_array(g_num_ports-1 downto 0); -- rx
  signal swc_snk_out		: t_wrf_sink_out_array(g_num_ports-1 downto 0);   -- tx
  
  signal swc_src_in		: t_wrf_source_in_array(g_num_ports-1 downto 0);
  signal swc_snk_in		: t_wrf_sink_in_array(g_num_ports-1 downto 0);
  signal ep_src_in 		: t_wrf_source_in_array(g_num_ports-1 downto 0);
  signal ep_snk_in 		: t_wrf_sink_in_array(g_num_ports-1 downto 0);

  signal fwd_src_out	: t_wrf_source_out_array(g_num_ports-1 downto 0);
  signal fwd_src_in	: t_wrf_source_in_array(g_num_ports-1 downto 0);

  -- taggers <-> sequencer
  signal hsr_seq_query : std_logic_vector(1 downto 0);
  type   t_array_seq_number is array (0 to 1) of std_logic_vector(15 downto 0); 
  signal seq_number : t_array_seq_number; 
  signal hsr_seq_valid : std_logic_vector(1 downto 0);
  
  signal regs_towb	: t_lre_in_registers;
  signal regs_fromwb : t_lre_out_registers;
  signal regs_rst		: std_logic;
  
  type t_fwd_ep_count is array (1 downto 0) of std_logic_vector(c_wishbone_data_width-1 downto 0);
  signal fwd_ep_count : t_fwd_ep_count;
  
  
  signal hsr_enable_d0 : std_logic;
  
  signal extended_ADDR : std_logic_vector(c_wishbone_address_width-1 downto 0);
  
  signal	 wb_in		:  t_wishbone_slave_in;
  signal	 wb_out     :  t_wishbone_slave_out;
  
  -- temporary signal, debug purposes
  signal mem_wb_o_dat : std_logic_vector(31 downto 0);

  begin --rtl
  
    --rst_n <= rst_n_i and regs_rst;

---- Uncomment this (and comment the rest all the way down) ---
---- to disconnect the whole HSR LRE:                       ---
--  ep_snk_o <= dummy_snk_out;
--  ep_src_o <= dummy_src_out;
--  
--  swc_src_o <= dummy_src_out;
--  swc_snk_o <= dummy_snk_out;
---------------------------------------------------------------

	rst_n <= rst_n_i and regs_fromwb.lcr_en_o;

	ep_snk_o  <= swc_src_i when regs_fromwb.lcr_en_o = '0' else ep_snk_out;
	ep_src_o  <= swc_snk_i when regs_fromwb.lcr_en_o = '0' else ep_src_out;
	
	swc_snk_o <= ep_src_i  when regs_fromwb.lcr_en_o = '0' else swc_snk_out;
	swc_src_o <= ep_snk_i  when regs_fromwb.lcr_en_o = '0' else swc_src_out;
	
	swc_src_in <= dummy_src_in when regs_fromwb.lcr_en_o = '0' else swc_src_i;
	swc_snk_in <= dummy_snk_in when regs_fromwb.lcr_en_o = '0' else swc_snk_i;
	
	ep_src_in  <= dummy_src_in when regs_fromwb.lcr_en_o = '0' else ep_src_i;
	ep_snk_in  <= dummy_snk_in when regs_fromwb.lcr_en_o = '0' else ep_snk_i;

  GEN_UNTAGGERS: for I in 0 to 1 generate
    U_XHSR_UNTAGGER: xhsr_untagger
      port map (
        rst_n_i => rst_n,
        clk_i   => clk_i,
        snk_i   => dropper_src_out(i),
        snk_o   => dropper_src_in(i),
        src_o	=>  swc_src_out(i),
        src_i   => swc_src_in(i));
    end generate;

  U_seq : xhsr_seq
    port map (
      rst_n_i => rst_n,
      clk_i   => clk_i,
      request0 => hsr_seq_query(0),
      request1 => hsr_seq_query(1),
      seq_n0 => seq_number(0),
      seq_n1 => seq_number(1),
      valid0 => hsr_seq_valid(0),
      valid1 => hsr_seq_valid(1)
    );
	 
	 regs_towb.seq_ep0_i(15 downto 0) <= seq_number(0);
	 regs_towb.seq_ep1_i(15 downto 0) <= seq_number(1);
--

  U_DROPPER : xhsr_dropper
	port map(
		rst_n_i => rst_n,
		clk_i => clk_i,
		snk_i => fwd_src_out,
		snk_o => fwd_src_in,
		src_i => dropper_src_in,
		src_o => dropper_src_out,
		mac_addr_i => regs_fromwb.mach_o & regs_fromwb.macl_o,
		
		wb_adr_i   => mem_wb_i.adr(8 downto 2),	
		wb_dat_i   => mem_wb_i.dat,                         
		wb_dat_o   => mem_wb_o.dat,                    
		wb_cyc_i   => mem_wb_i.cyc,                    
		wb_sel_i   => mem_wb_i.sel,                    
		wb_stb_i   => mem_wb_i.stb,                    
		wb_we_i    => mem_wb_i.we,                     
		wb_ack_o   => mem_wb_o.ack,                    
		wb_stall_o => mem_wb_o.stall,
      
      disc_ep0_o => regs_towb.disc_ep0_i,
      disc_ep1_o => regs_towb.disc_ep1_i,
      acc_ep0_o  => regs_towb.acc_ep0_i,
      acc_ep1_o  => regs_towb.acc_ep1_i

	);

----  Comment dropper and uncomment this to bypass dropper --
--                                                         --
--       dropper_src_out <= fwd_src_out;                   --
--       fwd_src_in <= dropper_src_in;                     --
--                                                         --
-------------------------------------------------------------
                                                    
  GEN_TAGGERS: for I in 0 to 1 generate
      U_XHSR_TAGGER: xhsr_tagger
      generic map(
         g_hsr_path_id => x"0"
      )
        port map (
          rst_n_i => rst_n,
          clk_i   => clk_i,
          snk_i   => swc_snk_in(i),
          snk_o   => swc_snk_out(i),
          src_o	=> tagger_src_out(i),
          src_i   => tagger_src_in(i),
          req_tag  => hsr_seq_query(i),
	  seq_n => seq_number(i),
	  seq_valid => hsr_seq_valid(i)
	);
    end generate;
    
--    U_XHSR_TAGGER_L: xhsr_tagger
--        generic map( 
--          g_hsr_path_id => c_hsr_path0
--        )
--        port map (
--          rst_n_i => rst_n_i,
--          clk_i   => clk_i,
--          snk_i   => swc_snk_i(0),
--          snk_o   => swc_snk_out(0),
--          src_o	=>  tagger_src_out(0),
--          src_i   => tagger_src_in(0),
--          req_tag  => hsr_seq_query(0),
--	      seq_n => seq_number(0),
--	      seq_valid => hsr_seq_valid(0)
--	);
	
--	U_XHSR_TAGGER_R: xhsr_tagger
--        generic map( 
--          g_hsr_path_id => c_hsr_path1
--        )
--        port map (
--          rst_n_i => rst_n_i,
--          clk_i   => clk_i,
--          snk_i   => swc_snk_i(1),
--          snk_o   => swc_snk_out(1),
--          src_o	=>  tagger_src_out(1),
--          src_i   =>  tagger_src_in(1),
--          req_tag  => hsr_seq_query(1),
--	      seq_n => seq_number(1),
--	      seq_valid => hsr_seq_valid(1)
--);    

--  U_XHSR_TAGGER0: xhsr_tagger
--        port map (
--          rst_n_i => rst_n_i,
--          clk_i   => clk_i,
--          snk_i   => swc_snk_i(0),
--          snk_o   => swc_snk_o(0),
--          src_o	=> tagger_src_out(0),
--          src_i   => tagger_src_in(0),
--          req_tag  => hsr_seq_query(0),
--	  seq_n => seq_number(0),
--	  seq_valid => hsr_seq_valid(0)
--	);
--	
--	U_XHSR_TAGGER1: xhsr_tagger_debug
--        port map (
--          rst_n_i => rst_n_i,
--          clk_i   => clk_i,
--          snk_i   => swc_snk_i(1),
--          snk_o   => swc_snk_o(1),
--          src_o	=> tagger_src_out(1),
--          src_i   => tagger_src_in(1),
--          req_tag  => hsr_seq_query(1),
--	  seq_n => seq_number(1),
--	  seq_valid => hsr_seq_valid(1)
--	);


  GEN_FWD: for I in 0 to 1 generate
  
	U_FWD : xhsr_fwd
		port map(
			rst_n_i => rst_n,
			clk_i => clk_i,
			snk_i => ep_snk_in(i),
			snk_o => ep_snk_out(i),
			src_o => fwd_src_out(i),
			src_i => fwd_src_in(i),
			fwd_dreq_i => fwd_dreq(i),
			fwd_fab_o => fwd_fab(i),
			mac_addr_i => regs_fromwb.mach_o & regs_fromwb.macl_o,
			fwd_count_o => fwd_ep_count(i),
			clr_cnt_i => regs_fromwb.lcr_clr_cnt_o,
			link_ok_i => link_ok_i((i+1) mod 2)
		);
   end generate;

   regs_towb.fwd_ep0_i <= fwd_ep_count(0);
   regs_towb.fwd_ep1_i <= fwd_ep_count(1);  

	 
	U_junction : wrsw_hsr_junction
		port map(
			rst_n_i			=> rst_n,
			clk_i				=> clk_i,
			link_ok_i		=> link_ok_i,
			ep_src_o			=> ep_src_out,
			ep_src_i			=> ep_src_in,
			tagger_snk_i 	=> tagger_src_out,
			tagger_snk_o 	=> tagger_src_in,
			fwd_snk_fab_i 	=> fwd_fab,
			fwd_snk_dreq_o	=> fwd_dreq,
			bound_ep0_count_o => regs_towb.bound_ep0_i,
			bound_ep1_count_o => regs_towb.bound_ep1_i,
			dup_ep0_count_o => regs_towb.dup_ep0_i,
			dup_ep1_count_o => regs_towb.dup_ep1_i,
			clr_cnt_i		 => regs_fromwb.lcr_clr_cnt_o);


   extended_ADDR <= std_logic_vector(resize(unsigned(wb_i.adr), c_wishbone_address_width));

	U_Slave_adapter : wb_slave_adapter
		 generic map (
			g_master_use_struct  => true,
			g_master_mode        => CLASSIC,
			g_master_granularity => WORD,
			g_slave_use_struct   => false,
			g_slave_mode         => PIPELINED,
			g_slave_granularity  => WORD)
		 port map (
			clk_sys_i  => clk_i,
			rst_n_i    => rst_n_i,
			sl_adr_i   => extended_ADDR,
			sl_dat_i   => wb_i.dat,
			sl_sel_i   => wb_i.sel,
			sl_cyc_i   => wb_i.cyc,
			sl_stb_i   => wb_i.stb,
			sl_we_i    => wb_i.we,
			sl_dat_o   => wb_o.dat,
			sl_ack_o   => wb_o.ack,
			sl_stall_o => wb_o.stall,
			master_i   => wb_out,
			master_o   => wb_in);	
			
	U_wbregs : hsr_lre_regs
	  port map(
		 rst_n_i    => rst_n_i,
		 clk_sys_i  => clk_i,
		 wb_adr_i   => wb_in.adr(5 downto 2),
		 wb_dat_i   => wb_in.dat,
		 wb_dat_o   => wb_out.dat,
		 wb_cyc_i   => wb_in.cyc,
		 wb_sel_i   => wb_in.sel,
		 wb_stb_i   => wb_in.stb,
		 wb_we_i    => wb_in.we,
		 wb_ack_o   => wb_out.ack,
		 wb_stall_o => wb_out.stall,
		 regs_i     => regs_towb,  --  : in     t_lre_in_registers;
		 regs_o     => regs_fromwb --  : out    t_lre_out_registers
	  );
     
     regs_towb.lcr_link_ok_i <= link_ok_i;

			

	-- DEBUG --
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

	trig0   <= mem_wb_i.adr;
	trig1   <= mem_wb_i.dat;                         
	-- trig2   <= mem_wb_o.dat;                        
	trig3(0)   <= mem_wb_i.cyc;                    
	trig3(4 downto 1)   <= mem_wb_i.sel;  
	trig3(5)   <= mem_wb_i.stb;             
	trig3(6)   <= mem_wb_i.we;                    
	--trig3(7)   <= mem_wb_o.ack;                    
	--trig3(8)  <= mem_wb_o.stall;
	
	--mem_wb_o.dat <= mem_wb_o_dat;


end behavioral;
