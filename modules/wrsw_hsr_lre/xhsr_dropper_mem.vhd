-------------------------------------------------------------------------------
-- Title      : HSR Link Redundancy Entity - Dropper memory unit
-- Project    : White Rabbit
-------------------------------------------------------------------------------
-- File       : xhsr_dropper_mem.vhd
-- Author     : Jose Lopez Jimenez
-- Company    : Universidad de Granada 
-- Department : 
-- Created    : 2016-09-28
-- Last update: 2016-09-28
-- Platform   : FPGA-generic
-- Standard   : VHDL '93
-------------------------------------------------------------------------------
-- Description: 
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
use work.memory_loader_pkg.all;
use work.wrsw_hsr_lre_pkg.all;


entity xhsr_dropper_mem is

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
	end xhsr_dropper_mem;

architecture behavioral of xhsr_dropper_mem is
  
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
  
  type	t_state						is (S_IDLE, S_READING, S_WRITING);
  signal	state_a, state_b	:		t_state;
  
	constant c_mem_width				: natural := 256;
	constant c_mem_size				: natural := 8;
	constant c_addr_size				: natural := f_log2_size(c_mem_size);
--	constant c_slot0_base			: std_logic_vector(c_addr_size-1 downto 0) := "00000000000";
--	constant c_slot1_base			: std_logic_vector(c_addr_size-1 downto 0) := "11001000000";

	signal	write_a, write_b					: std_logic;
	signal	byte_mask_a, byte_mask_b		: std_logic_vector(c_mem_width/8 - 1 downto 0);
	signal	addr_a, addr_b						: std_logic_vector(c_addr_size-1 downto 0);
	signal	din_a, din_b						: std_logic_vector(c_mem_width-1 downto 0);
	signal	dout_a, dout_b						: std_logic_vector(c_mem_width-1 downto 0);
   signal   nodeid_a_out, nodeid_b_out    : std_logic_vector(4 downto 0);
   signal   match_a_out, match_b_out      : std_logic;
	
	signal senaldebug : std_logic_vector(7 downto 0);
	
	signal	mem_busy_a, mem_busy_b			: std_logic;
	
	signal	addr_int_a, addr_int_b			: integer;
   signal   addr_int_a_d0, addr_int_b_d0  : integer;



	

  begin 

	senaldebug_o <= senaldebug;
   nodeid_a_o <= nodeid_a_out;
   match_a_o <= match_a_out;
   match_b_o <= match_b_out;
   nodeid_b_o <= nodeid_b_out;

	U_mem : generic_dpram
	generic map(
		g_data_width => c_mem_width,
		g_dual_clock => false,
		g_size		 => c_mem_size,
		g_with_byte_enable => true)
	port map(
		rst_n_i		=> rst_n_i,
		clka_i		=> clk_i,
		clkb_i		=> clk_i,
		
		bwea_i		=> byte_mask_a,
		wea_i			=> write_a,
		aa_i			=> addr_a,
		da_i			=> din_a,
		qa_o			=> dout_a,
		
		bweb_i		=> byte_mask_b,
		web_i			=> write_b,
		ab_i			=> addr_b,
		db_i			=> din_b,
		qb_o			=> dout_b	
	);
	
   addr_a <= std_logic_vector(to_unsigned(addr_int_a,3));
   addr_b <= std_logic_vector(to_unsigned(addr_int_b,3));
   
	p_mem_control_a : process(clk_i)
   variable match_int_a : std_logic := '0';
	begin
	
		if rst_n_i = '0' then
			senaldebug <= x"01";
		elsif rising_edge(clk_i) then
         addr_int_a_d0 <= addr_int_a;
			case state_a is
				
				when s_IDLE =>
					senaldebug <= x"02";
					write_a <= '0';
					mem_busy_a <= '0';
					valid_a_o <= '0';
					match_a_out <= '0';
               match_int_a := '0';
					if rd_sig_a_i = '1' then
						--addr_a <= "000";
						addr_int_a <= 0;
						state_a <= s_READING;
						mem_busy_a <= '1';
						senaldebug <= x"02";
					end if;
					
					if wr_sig_a_i = '1' then
						state_a <= s_WRITING;
						mem_busy_a <= '1';
						senaldebug <= x"03";
					end if;
				
				when s_READING =>
					senaldebug <= x"04";

					if(dout_a(47 downto 0) = smac_a_i) then
						match_a_out 	<= '1';
						nodeid_a_out 	<= std_logic_vector(to_unsigned(addr_int_a_d0,3)) & "00";
						seqnum_a_o  <= dout_a(63 downto 48);
						valid_a_o	<= '1';
						state_a 		<= s_IDLE;
						mem_busy_a 	<= '0';
						senaldebug <= x"05";
                  match_int_a := '1';
					elsif(dout_a(111 downto 64) = smac_a_i) then
						match_a_out	<= '1';
						nodeid_a_out	<= std_logic_vector(to_unsigned(addr_int_a_d0,3)) & "01";
						seqnum_a_o  <= dout_a(127 downto 112);
						valid_a_o	<= '1';
						state_a 		<= s_IDLE;
						mem_busy_a 	<= '0';
						senaldebug <= x"06";
                  match_int_a := '1';
					elsif(dout_a(175 downto 128) = smac_a_i) then
						match_a_out	<= '1';
						nodeid_a_out	<= std_logic_vector(to_unsigned(addr_int_a_d0,3)) & "10";
						seqnum_a_o  <= dout_a(191 downto 176);
						valid_a_o	<= '1';
						state_a 		<= s_IDLE;
						mem_busy_a 	<= '0';
						senaldebug <= x"07";
                  match_int_a := '1';
					elsif(dout_a(239 downto 192) = smac_a_i) then
						match_a_out	<= '1';
						nodeid_a_out	<= std_logic_vector(to_unsigned(addr_int_a_d0,3)) & "11";
						seqnum_a_o  <= dout_a(255 downto 240);
						valid_a_o	<= '1';
						state_a 		<= s_IDLE;
						mem_busy_a 	<= '0';
						senaldebug <= x"08";
                  match_int_a := '1';
					elsif addr_int_a > 7 then
						match_a_out	<= '0';
						valid_a_o	<= '1';
						state_a		<= s_IDLE;
						mem_busy_a 	<= '0';
						senaldebug <= x"09";
                  addr_int_a <= 0;
					else
                  addr_int_a  <= addr_int_a + 1;
               end if;				
               
				
				when s_WRITING =>
					
					senaldebug <= x"0a";
					addr_int_a <= to_integer(unsigned(nodeid_a_i(4 downto 2)));
               if addr_int_a = to_integer(unsigned(nodeid_a_i(4 downto 2))) then
                  write_a <= '1';
                  valid_a_o <= '1';
                  state_a <= s_IDLE;
  					mem_busy_a 	<= '0';

               end if;
					case nodeid_a_i(1 downto 0) is
						when "00" =>
							byte_mask_a <= x"000000ff";
							din_a			<= x"000000000000000000000000000000000000000000000000" & seqnum_a_i & smac_a_i;
							senaldebug <= x"0b";
						when "01" =>
							byte_mask_a <= x"0000ff00";
							din_a			<= x"00000000000000000000000000000000" & seqnum_a_i & smac_a_i & x"0000000000000000";
							senaldebug <= x"0c";
						when "10" =>
							byte_mask_a <= x"00ff0000";
							din_a 		<= x"0000000000000000" & seqnum_a_i & smac_a_i & x"00000000000000000000000000000000";
							senaldebug <= x"0d";
						when "11" =>
							byte_mask_a	<= x"ff000000";
							din_a			<= seqnum_a_i & smac_a_i & x"000000000000000000000000000000000000000000000000";
							senaldebug <= x"0e";
						when others =>
					end case;
										
					
				when others =>
				
			end case;
		end if;
	
	end process;
	


	p_mem_control_b : process(clk_i)
   variable match_int_b : std_logic := '0';
	begin
	
		if rst_n_i = '0' then
--			senaldebug <= x"01";
		elsif rising_edge(clk_i) then
         addr_int_b_d0 <= addr_int_b;
			case state_b is
				
				when s_IDLE =>
					-- senaldebug <= x"02";
					write_b <= '0';
					mem_busy_b <= '0';
					valid_b_o <= '0';
					match_b_out <= '0';
               match_int_b := '0';
					if rd_sig_b_i = '1' then
						--addr_b <= "000";
						addr_int_b <= 0;
						state_b <= s_READING;
						mem_busy_b <= '1';
						-- senaldebug <= x"02";
					end if;
					
					if wr_sig_b_i = '1' then
						state_b <= s_WRITING;
						mem_busy_b <= '1';
						-- senaldebug <= x"03";
					end if;
				
				when s_READING =>
					
					if(dout_b(47 downto 0) = smac_b_i) then
						match_b_out 	<= '1';
						nodeid_b_out 	<= std_logic_vector(to_unsigned(addr_int_b_d0,3)) & "00";
						seqnum_b_o  <= dout_b(63 downto 48);
						valid_b_o	<= '1';
						state_b 		<= s_IDLE;
						mem_busy_b 	<= '0';
						-- senaldebug <= x"05";
                  match_int_b := '1';
					elsif(dout_b(111 downto 64) = smac_b_i) then
						match_b_out	<= '1';
						nodeid_b_out	<= std_logic_vector(to_unsigned(addr_int_b_d0,3)) & "01";
						seqnum_b_o  <= dout_b(127 downto 112);
						valid_b_o	<= '1';
						state_b 		<= s_IDLE;
						mem_busy_b 	<= '0';
						-- senaldebug <= x"06";
                  match_int_b := '1';
					elsif(dout_b(175 downto 128) = smac_b_i) then
						match_b_out	<= '1';
						nodeid_b_out	<= std_logic_vector(to_unsigned(addr_int_b_d0,3)) & "10";
						seqnum_b_o  <= dout_b(191 downto 176);
						valid_b_o	<= '1';
						state_b 		<= s_IDLE;
						mem_busy_b 	<= '0';
						-- senaldebug <= x"07";
                  match_int_b := '1';
					elsif(dout_b(239 downto 192) = smac_b_i) then
						match_b_out	<= '1';
						nodeid_b_out	<= std_logic_vector(to_unsigned(addr_int_b_d0,3)) & "11";
						seqnum_b_o  <= dout_b(255 downto 240);
						valid_b_o	<= '1';
						state_b 		<= s_IDLE;
						mem_busy_b 	<= '0';
						-- senaldebug <= x"08";
                  match_int_b := '1';
					elsif addr_int_b > 7 then
						match_b_out	<= '0';
						valid_b_o	<= '1';
						state_b		<= s_IDLE;
						mem_busy_b 	<= '0';
						-- senaldebug <= x"09";
                  addr_int_b <= 0;
					else
                  addr_int_b  <= addr_int_b + 1;
               end if;				
               

				when s_WRITING =>
					
					-- senaldebug <= x"0a";
					addr_int_b <= to_integer(unsigned(nodeid_b_i(4 downto 2)));
               if addr_int_b = to_integer(unsigned(nodeid_b_i(4 downto 2))) then
                  write_b <= '1';
                  valid_b_o <= '1';
                  state_b <= s_IDLE;
  					mem_busy_b 	<= '0';

               end if;
					case nodeid_b_i(1 downto 0) is
						when "00" =>
							byte_mask_b <= x"000000ff";
							din_b			<= x"000000000000000000000000000000000000000000000000" & seqnum_b_i & smac_b_i;
							-- senaldebug <= x"0b";
						when "01" =>
							byte_mask_b <= x"0000ff00";
							din_b			<= x"00000000000000000000000000000000" & seqnum_b_i & smac_b_i & x"0000000000000000";
							-- senaldebug <= x"0c";
						when "10" =>
							byte_mask_b <= x"00ff0000";
							din_b 		<= x"0000000000000000" & seqnum_b_i & smac_b_i & x"00000000000000000000000000000000";
							-- senaldebug <= x"0d";
						when "11" =>
							byte_mask_b	<= x"ff000000";
							din_b			<= seqnum_b_i & smac_b_i & x"000000000000000000000000000000000000000000000000";
							-- senaldebug <= x"0e";
						when others =>
					end case;
										
					
				when others =>
				
			end case;
		end if;
	
	end process;
	
	mem_busy_o <= mem_busy_a or mem_busy_b;
	
--   cs_icon : chipscope_icon
--   port map(
--      CONTROL0   => CONTROL0
--   );
--   cs_ila : chipscope_ila
--   port map(
--      CLK      => clk_i,
--      CONTROL   => CONTROL0,
--      TRIG0      => TRIG0,
--      TRIG1      => TRIG1,
--      TRIG2      => TRIG2,
--      TRIG3      => TRIG3
--   );
   
   trig0(31 downto 8) <= smac_b_i(39 downto 16);
   trig0(4 downto 0) <= nodeid_b_out;
   trig1(15 downto 0) <= smac_b_i(15 downto 0);
   trig1(20 downto 16) <= nodeid_b_i;
   trig1(21) <= rd_sig_b_i;
   trig1(22) <= wr_sig_b_i;
   trig1(30 downto 23) <= senaldebug;
   trig1(31) <= write_b;
   trig2(2 downto 0) <= std_logic_vector(to_unsigned(addr_int_b,3));
   trig2(31 downto 3) <= din_b(92 downto 64);
   trig3(30 downto 0) <= dout_b(94 downto 64);
   trig3(31) <= match_b_out;

end behavioral;
