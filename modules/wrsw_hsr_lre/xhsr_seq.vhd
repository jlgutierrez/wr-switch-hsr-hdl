-------------------------------------------------------------------------------
-- Title      : HSR Sequence Number Generator
-- Project    : White Rabbit
-------------------------------------------------------------------------------
-- File       : xhsr_seq.vhd
-- Author     : José Luis Gutiérrez
-- Company    : University of Granada 
-- Department : Computer Architecture and Technology
-- Created    : 2016-01-18
-- Last update: 2016-01-18
-- Platform   : FPGA-generic
-- Standard   : VHDL '93
-------------------------------------------------------------------------------
-- Description: HSR Sequence Number Generator.
-- It updates the counter whenever e0 or e1 asks for a new sequence number.
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

entity xhsr_seq is
 generic (g_dat_width : integer :=16);
 port (
	 clk_i   : in std_logic; 
	 rst_n_i : in std_logic; 
	 request0: in std_logic; -- e0 asks for seq_number
	 request1: in std_logic; -- e1 asks for seq_number
	 seq_n0 : out std_logic_vector(g_dat_width-1 downto 0); -- 16bits seq_number for hsr tagger e0
	 seq_n1 : out std_logic_vector(g_dat_width-1 downto 0); -- 16bits seq_number for hsr tagger e1
	 valid0: out std_logic; -- only valid if e0 and e1 are not asking for 
	 valid1: out std_logic -- the seq_number at the same time.
 );
end xhsr_seq;

architecture behavoural of xhsr_seq is
 signal cnt : integer := 0;
 signal next_ep : std_logic := '0';

begin

 hsr_generator : process (clk_i)
 begin
    if rising_edge(clk_i) then
	  if rst_n_i = '0' then
	    cnt <= 0;
	    next_ep <= '0';
	  else
	    if request0='1' and request1='1' and next_ep='0' then
		  cnt <= cnt + 1;
		  valid0 <= '1';
		  valid1 <= '0';
		  next_ep <= '1';
        elsif request0='1' and request1='1' and next_ep='1' then
          cnt <= cnt + 1;
		  valid0 <= '0';
		  valid1 <= '1';
		  next_ep <= '0';
        elsif request0='1' and request1='0' then
          cnt <= cnt + 1;
		  valid0 <= '1';
		  valid1 <= '0';
		elsif request0='0' and request1='1' then
		  cnt <= cnt + 1;
		  valid0 <= '0';
		  valid1 <= '1';
        else
          valid0 <= '0';
          valid1 <= '0';
		end if;
	  end if;
	end if;
 end process;

 seq_n0 <= std_logic_vector(to_unsigned(cnt,seq_n0'length));
 seq_n1 <= std_logic_vector(to_unsigned(cnt,seq_n1'length));

end behavoural;
