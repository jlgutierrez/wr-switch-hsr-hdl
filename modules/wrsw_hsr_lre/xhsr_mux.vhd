-------------------------------------------------------------------------------
-- Title      : Mux/arbiter for Wishbone Fabric in HSR LRE
-- Project    : 
-------------------------------------------------------------------------------
-- File       : xhsr_mux.vhd
-- Author     : Jose Lopez
-- Company    : Universidad de Granada
-- Created    : 2016-07-29
-- Last update: 2016-07-29
-- Platform   : 
-- Standard   : VHDL
-------------------------------------------------------------------------------
-- Description:
-- 
-- 
-- 
-- 
-- 
-- 
-------------------------------------------------------------------------------
-- Copyright (c) 2016 
-------------------------------------------------------------------------------
-- Revisions  :
-- Date        Version  Author          Description
-- 2016-07-29  1.0      joselj          Created
-------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_misc.all;

use ieee.numeric_std.all;

library work;
use work.wr_fabric_pkg.all;
use work.genram_pkg.all;

entity xhsr_mux is
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
end xhsr_mux;

architecture behaviour of xhsr_mux is

	signal request : std_logic_vector(1 downto 0);
	signal grant : std_logic_vector(1 downto 0);

begin

	request <= mux_snk_i(1).cyc & mux_snk_i(0).cyc;

	p_mux : process(clk_sys_i, rst_n_i)
	begin
	if (rst_n_i = '0') then
		grant <= (others => '0');
	elsif rising_edge(clk_sys_i) then
		grant(0) <= request(0);
		grant(1) <= request(1) and not(request(0));
	end if;	
	end process;
	
	gen_mux : for i in 0 to 1 generate
	
	mux_snk_o(i).ack <= ep_src_i.ack when grant(i) = '1' else '0';
	mux_snk_o(i).err <= ep_src_i.err when grant(i) = '1' else '0';
	
	end generate;

	mux_snk_o(0).stall <= ep_src_i.stall when grant(0) = '1' else
								 '1' when grant(1) = '1' else
								 '0';
								 
   mux_snk_o(1).stall <= ep_src_i.stall when grant(1) = '1' else
								 '1' when grant(0) = '1' else
								 '0';								 
	
	ep_src_o.cyc <= mux_snk_i(0).cyc when grant(0) = '1'
						 else mux_snk_i(1).cyc when grant(1) = '1'
						 else '0';
	
	ep_src_o.stb <= mux_snk_i(0).stb when grant(0) = '1' else
						 mux_snk_i(1).stb when grant(1) = '1' else '0';
	ep_src_o.adr <= mux_snk_i(0).adr when grant(0) = '1' else
						 mux_snk_i(1).adr when grant(1) = '1' else (others => '0');
	ep_src_o.dat <= mux_snk_i(0).dat when grant(0) = '1' else
						 mux_snk_i(1).dat when grant(1) = '1' else (others => '0');
	ep_src_o.sel <= mux_snk_i(0).sel when grant(0) = '1' else
						 mux_snk_i(1).sel when grant(1) = '1' else (others => '0');
	ep_src_o.we  <= '1';
	

end behaviour;

