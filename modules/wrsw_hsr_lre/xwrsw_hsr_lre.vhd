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
    swc_snk_i : in  t_wrf_sink_in_array(g_num_ports-1 downto 0)     -- tx
   

    );
end xwrsw_hsr_lre;

architecture behavioral of xwrsw_hsr_lre is

  constant c_NUM_PORTS     : integer := g_num_ports; --GUTI: to be fix
  
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
  
  signal dummy_snk_out : t_wrf_sink_out_array(g_num_ports-1 downto 0);
  signal dummy_src_out : t_wrf_source_out_array(g_num_ports-1 downto 0);
  
  signal fwd_fab		  : t_ep_internal_fabric_array(1 downto 0);
  signal fwd_dreq	     : std_logic_vector(1 downto 0);
 
  signal tagger_src_out	: t_wrf_source_out_array(g_num_ports-1 downto 0);
  signal tagger_src_in	: t_wrf_source_in_array(g_num_ports-1 downto 0);
  signal tagger_snk_out	: t_wrf_sink_out_array(g_num_ports-1 downto 0);
  signal tagger_snk_in	: t_wrf_sink_in_array(g_num_ports-1 downto 0);

  -- taggers <-> sequencer
  signal hsr_seq_query : std_logic_vector(1 downto 0);
  type t_array_seq_number is array (0 to 1) of std_logic_vector(15 downto 0); 
  signal seq_number : t_array_seq_number; 
  signal hsr_seq_valid : std_logic_vector(1 downto 0);

  begin --rtl

---- Uncomment this (and comment the rest all the way down) ---
---- to disconnect the whole HSR LRE:                       ---
--  ep_snk_o <= dummy_snk_out;
--  ep_src_o <= dummy_src_out;
--  
--  swc_src_o <= dummy_src_out;
--  swc_snk_o <= dummy_snk_out;
---------------------------------------------------------------

--   process(clk_i)
--     begin
--    if rising_edge(clk_i) then
--
--		  ep_snk_o <= swc_src_i;
--      ep_src_o <= swc_snk_i;
--
--      swc_snk_o <= ep_src_i;
--      swc_src_o <= ep_snk_i;
--      

--
----		BYPASS_HSR: for J in 0 to 1 loop
------			ep_src_o(j) <= tagger_src_out(j);
------			tagger_src_in(j)	<= ep_src_i(j);
----			
----			swc_src_o(j) <= ep_snk_i(j);
----			ep_snk_o(j) <= swc_src_i(j);
----		end loop;
----
--      BYPASS_NON_HSR: for J in 2 to 4 loop
--        ep_snk_o(j)		<= swc_src_i(j);
--        swc_src_o(j)	<= ep_snk_i(j);
--		  ep_src_o(j) <= swc_snk_i(j);
--		  swc_snk_o(j) <= ep_src_i(j);
--      end loop;
		
--		BYPASS_NON_HSR_onemoretime: for J in 6 to c_NUM_PORTS loop
--        ep_snk_o(j)		<= swc_src_i(j);
--        swc_src_o(j)	<= ep_snk_i(j);
--		  ep_src_o(j) <= swc_snk_i(j);
--		  swc_snk_o(j) <= ep_src_i(j);
--      end loop;

		
		
--
--    end if;
--  end process;
  

		
		-- HSR-PORT INCOMING TRAFFIC BYPASSED
		-- AS THERE IS NO LRE FOR CHECKING DUPLICATES YET:
		
		ep_snk_o(g_num_ports-1 downto 0) <= swc_src_i(g_num_ports-1 downto 0);
		swc_src_o(g_num_ports-1 downto 0) <= ep_snk_i(g_num_ports-1 downto 0);
		

  U_seq : xhsr_seq
    port map (
      rst_n_i => rst_n_i,
      clk_i   => clk_i,
      request0 => hsr_seq_query(0),
      request1 => hsr_seq_query(1),
      seq_n0 => seq_number(0),
      seq_n1 => seq_number(1),
      valid0 => hsr_seq_valid(0),
      valid1 => hsr_seq_valid(1)
    );

  GEN_TAGGERS: for I in 0 to 1 generate
      U_XHSR_TAGGER: xhsr_tagger
        port map (
          rst_n_i => rst_n_i,
          clk_i   => clk_i,
          snk_i   => swc_snk_i(i),
          snk_o   => swc_snk_o(i),
          src_o	=> tagger_src_out(i),
          src_i   => tagger_src_in(i),
          req_tag  => hsr_seq_query(i),
	  seq_n => seq_number(i),
	  seq_valid => hsr_seq_valid(i)
	);
    end generate;

---- While debugging we cannot have these modules created via a generate: ---- 
--  GEN_FWD: for I in 0 to 1 generate
--  
--	U_FWD : xhsr_fwd
--		port map(
--			rst_n_i => rst_n_i,
--			clk_i => clk_i,
--			snk_i => ep_snk_i(i),
--			snk_o => ep_snk_o(i),
--			src_o => swc_src_o(i),
--			src_i => swc_src_i(i),
--			fwd_dreq_i => fwd_dreq(i),
--			fwd_fab_o => fwd_fab(i)
--		);
--  
--  end generate;
-------------------------------------------------------------------------------

	U_FWD0 : xhsr_fwd
		port map(
			rst_n_i => rst_n_i,
			clk_i => clk_i,
			snk_i => ep_snk_i(0),
			snk_o => ep_snk_o(0),
			src_o => swc_src_o(0),
			src_i => swc_src_i(0),
			fwd_dreq_i => fwd_dreq(0),
			fwd_fab_o => fwd_fab(0)
		);

	U_FWD1 : xhsr_fwd_debug
		port map(
			rst_n_i => rst_n_i,
			clk_i => clk_i,
			snk_i => ep_snk_i(1),
			snk_o => ep_snk_o(1),
			src_o => swc_src_o(1),
			src_i => swc_src_i(1),
			fwd_dreq_i => fwd_dreq(1),
			fwd_fab_o => fwd_fab(1)
		);

	 
	U_junction : wrsw_hsr_junction
		port map(
			rst_n_i			=> rst_n_i,
			clk_i				=> clk_i,
			link_ok_i		=> link_ok_i,
			ep_src_o			=> ep_src_o,
			ep_src_i			=> ep_src_i,
			tagger_snk_i 	=> tagger_src_out,
			tagger_snk_o 	=> tagger_src_in,
			fwd_snk_fab_i 	=> fwd_fab,
			fwd_snk_dreq_o	=> fwd_dreq);
			

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
--	
--trig0(1 downto 0) <= ep_snk_i(0).adr;
--trig0(17 downto 2) <= ep_snk_i(0).dat;
--trig0(18) <= ep_snk_i(0).cyc;
--trig0(19) <= ep_snk_i(0).stb;
--
--trig1(1 downto 0) <= ep_snk_i(1).adr;
--trig1(17 downto 2) <= ep_snk_i(1).dat;
--trig1(18) <= ep_snk_i(1).cyc;
--trig1(19) <= ep_snk_i(1).stb;

end behavioral;
