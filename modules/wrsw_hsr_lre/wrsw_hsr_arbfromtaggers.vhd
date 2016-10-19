-------------------------------------------------------------------------------
-- Title      : HSR Link Redundancy Entity - Manage from taggers
-- Project    : White Rabbit
-------------------------------------------------------------------------------
-- File       : wrsw_hsr_arbfromtaggers.vhd
-- Author     : 
-- Company    :  
-- Department : 
-- Created    : 2016-02-22
-- Last update: 2016-02-22
-- Platform   : FPGA-generic
-- Standard   : VHDL '93
-------------------------------------------------------------------------------
-- Description: This module receives traffic from two tx interfaces of the
--    switching core. There is a memory that can save the information of up to
--    two frames at the same time. The frames are analyzed on the fly to make
--    the decision of duplicating them (all frames but PTP when HSR mode is on)
--    or sending them only through the endpoint they were initially bound to be
--    sent.
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
use work.endpoint_private_pkg.all;
use work.mpm_pkg.all;
use work.wrsw_hsr_lre_pkg.all;
use work.genram_pkg.all;
use work.memory_loader_pkg.all;

entity wrsw_hsr_arbfromtaggers is

  generic(
    g_adr_width : integer := 2;
    g_dat_width : integer :=16
    );
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
	   clr_cnt_i 				: in  std_logic
    );
end wrsw_hsr_arbfromtaggers;

architecture behavioral of wrsw_hsr_arbfromtaggers is
  
  component ep_rx_wb_master
    generic (
      g_ignore_ack   : boolean;
      g_cyc_on_stall : boolean := false);
    port (
      clk_sys_i  : in  std_logic;
      rst_n_i    : in  std_logic;
      snk_fab_i  : in  t_ep_internal_fabric;
      snk_dreq_o : out std_logic;
      src_wb_i   : in  t_wrf_source_in;
      src_wb_o   : out t_wrf_source_out);
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
  signal senaldebug0,senaldebug1,senaldebug2 : std_logic_vector(7 downto 0) := (others => '0');
  
  signal tagger_snk_in		: t_wrf_source_out_array(1 downto 0);
  signal ep_src_in			: t_wrf_source_in_array(1 downto 0);
  
  type	t_wr_state					is (S_IDLE, S_WRITING, S_EOF, S_FULL);
  signal	wr_state						:	t_wr_state;
  
  type	t_rd_state					is (IDLE, READING, FINISH_CYCLE);
  signal rd_state0, rd_state1		:	t_rd_state;
  
  type	t_mem_slot_status is record
		available						:	std_logic;
		writing							:	std_logic;
		written							:	std_logic;
		reading_0						:	std_logic;
		reading_1						:	std_logic;
		finished_0						:	std_logic;
		finished_1						:	std_logic;
		is_hsr							:	std_logic;
		is_ptp							:	std_logic;
		is_multicast					:  std_logic;
		source							:	std_logic;
	end record;
	
	signal 	slot0, slot1			: t_mem_slot_status :=('1','0','0','0','0','0','0','0','0','0','0');
  
	constant c_mem_width				: natural := 21;
	constant c_mem_size				: natural := 2*800;
	constant c_addr_size				: natural := f_log2_size(c_mem_size);
	constant c_slot0_base			: std_logic_vector(c_addr_size-1 downto 0) := "00000000000";
	constant c_slot1_base			: std_logic_vector(c_addr_size-1 downto 0) := "11001000000";
	
	constant c_sig_sof				: std_logic_vector(c_mem_width-1-1 downto 0) := "11000000000011111111";
	constant c_sig_eof				: std_logic_vector(c_mem_width-1-1 downto 0) := "11000000000010101010";

	signal	write_a, write_b, write_c		: std_logic := '0';
	signal	addr_a, addr_b, addr_c			: std_logic_vector(c_addr_size-1 downto 0);
	signal	din_a, din_b, din_c				: std_logic_vector(c_mem_width-1 downto 0);
	signal	dout_a, dout_b, dout_c			: std_logic_vector(c_mem_width-1 downto 0);
	signal	wr_offset							: unsigned(c_addr_size-1 downto 0);
	signal	rd_offset_b							: unsigned(c_addr_size-1 downto 0);
	signal	rd_offset_c							: unsigned(c_addr_size-1 downto 0);

	signal   sof, eof					: std_logic_vector(1 downto 0) := (others => '0');
	signal   snk_cyc_d0				: std_logic_vector(1 downto 0) := (others => '0');
	signal	snk_valid				: std_logic_vector(1 downto 0) := "00";
	signal	stall						: std_logic_vector(1 downto 0) := (others => '0');
	
	signal	snk_dreq					: std_logic_vector(1 downto 0) := "00";
	signal	snk_fab_0, snk_fab_1	: t_ep_internal_fabric;
   signal	ep_src_o_int			: t_wrf_source_out_array(1 downto 0);
	
	signal	word_count				: std_logic_vector(10 downto 0) := (others => '0');
	
	signal   bound_ep0_count		: std_logic_vector(31 downto 0);
	signal   bound_ep1_count		: std_logic_vector(31 downto 0);
	signal   dup_ep0_count			: std_logic_vector(31 downto 0);
	signal   dup_ep1_count			: std_logic_vector(31 downto 0);
	

  begin 


---- Uncomment this (and comment the rest all the way down) --
---- to convert this arbiter into 'a wire'.                 --
--  ep_src_o <= tagger_snk_i;
--  tagger_snk_o <= ep_src_i;
--------------------------------------------------------------	

--	Memory sized for 2 max-length frames (1522 bytes) at 16 bits/word
-- plus 2 bits/word for wb adr field.
-- There is also some space reserved for future use.
--
-- There is the need (if we do not want to multiply the time needed
-- to send a frame) to use one port for writing and two ports for
-- reading, simultaneously. As BRAMs only have two ports, there are
-- two identical BRAMs with the same exact content all the time so
-- there is a reading port available from each BRAM.
	U_mem : generic_dpram
	generic map(
		g_data_width => c_mem_width,
		g_dual_clock => false,
		g_size		 => c_mem_size)
	port map(
		rst_n_i		=> rst_n_i,
		clka_i		=> clk_i,
		clkb_i		=> clk_i,
		
		wea_i			=> write_a,
		aa_i			=> addr_a,
		da_i			=> din_a,
		qa_o			=> dout_a,
		
		web_i			=> write_b,
		ab_i			=> addr_b,
		db_i			=> din_b,
		qb_o			=> dout_b	
	);
	
	U_mem_bis : generic_dpram
	generic map(
		g_data_width => c_mem_width,
		g_dual_clock => false,
		g_size		 => c_mem_size)
	port map(
		rst_n_i		=> rst_n_i,
		clka_i		=> clk_i,
		clkb_i		=> clk_i,
		
		wea_i			=> write_a,
		aa_i			=> addr_a,
		da_i			=> din_a,
		qa_o			=> dout_a,
		
		web_i			=> write_c,
		ab_i			=> addr_c,
		db_i			=> din_c,
		qb_o			=> dout_c	
	);	
	
	p_detect_frame : process(clk_i)
	begin
		if(rising_edge(clk_i)) then
			if rst_n_i = '0' then
				snk_cyc_d0 <= (others => '0');
			else
				snk_cyc_d0 <= tagger_snk_i(1).cyc & tagger_snk_i(0).cyc;
			end if;		
		end if;
	end process;
	
	sof <= not snk_cyc_d0 and tagger_snk_i(1).cyc & tagger_snk_i(0).cyc;
	eof <= snk_cyc_d0 and not(tagger_snk_i(1).cyc & tagger_snk_i(0).cyc);
	
	snk_valid(0) <= tagger_snk_i(0).cyc and tagger_snk_i(0).stb and tagger_snk_i(0).we and not stall(0);
	snk_valid(1) <= tagger_snk_i(1).cyc and tagger_snk_i(1).stb and tagger_snk_i(1).we and not stall(1);
	
	p_wr_fsm : process(clk_i)
		variable debug	: boolean := false;
		variable multicast0, multicast1 : std_logic := '0';
	begin
		if(rising_edge(clk_i)) then
			if(rst_n_i = '0') then
				write_a		<= '0';
				addr_a		<= (others => '0');
				wr_offset	<= (others => '0');
				wr_state		<= S_IDLE;
				word_count	<= (others => '0');
				stall       <= (others => '0');
								
				slot0.writing   <= '0';
				slot0.written   <= '0';
				slot0.is_ptp    <= '0';
				slot0.is_hsr    <= '0';
				slot0.available <= '1';
				
				slot1.writing   <= '0';
				slot1.written   <= '0';
				slot1.is_ptp    <= '0';
				slot1.is_hsr    <= '0';
				slot1.available <= '1';
				senaldebug0 <= x"00";
				
			else
			
				if(slot0.finished_0 = '1' and slot0.finished_1 = '1' and slot0.written = '1') then
					slot0.written <= '0';
				end if;
				
				if(slot1.finished_0 = '1' and slot1.finished_1 = '1' and slot1.written = '1') then
					slot1.written <= '0';
				end if;
				
				if(slot0.finished_0 = '1' and slot0.finished_1 = '1') then
					slot0.available <= '1';
				end if;
				
				if(slot1.finished_0 = '1' and slot1.finished_1 = '1') then
					slot1.available <= '1';
				end if;				
			
				case wr_state is
					
					when S_IDLE =>
					
						senaldebug0 <= x"D1";
						
						word_count	<= (others => '0');
						write_a		<= '0';
						if(sof(0) = '1') then -- IF LEV 1
						
							if(slot0.available = '1' or debug) then -- IF LEV 2
							
								senaldebug0 <= x"01";
								addr_a 			<= c_slot0_base;
								din_a 			<= snk_valid(0) & c_sig_sof;
								write_a 			<= '1';
								slot0.available <= '0';
								slot0.writing	<= '1';
								slot1.writing	<= '0';
								slot0.source	<= '0';
								wr_state 		<= S_WRITING;
								stall(1)			<= '1';
								wr_offset		<= to_unsigned(1, wr_offset'length);
								
							elsif(slot1.available = '1') then
							
								addr_a			<= c_slot1_base;
								din_a			<= snk_valid(0) & c_sig_sof;
								write_a			<= '1';
								slot1.writing 	<= '1';
								slot0.writing	<= '0';
								slot1.source 	<= '0';
								slot1.available <= '0';
								wr_state 		<= S_WRITING;
								stall(1)			<= '1';
								wr_offset		<= to_unsigned(1, wr_offset'length);
								senaldebug0 <= x"02";

								
							else
								
								wr_state 		<= S_FULL;
								stall 			<= (others => '1');
								senaldebug0 <= x"03";
								
							end if; -- ENDIF LEV 1
						
						elsif(sof(1) = '1') then
						
							if(slot0.available = '1' or debug) then -- IF LEV 2
							
								addr_a			<= c_slot0_base;
								din_a			<= snk_valid(1) & c_sig_sof;
								write_a			<= '1';
								slot0.writing  <= '1';
								slot1.writing	<= '0';
								slot0.source	<= '1';
								slot0.available <= '0';
								wr_state			<= S_WRITING;
								wr_offset		<= to_unsigned(1, wr_offset'length);
								senaldebug0 <= x"04";

								
							elsif(slot1.available = '1') then
							
								addr_a			<= c_slot1_base;
								din_a			<= snk_valid(1) & c_sig_sof;
								write_a			<= '1';
								slot1.writing	<= '1';
								slot0.writing	<= '0';
								slot1.source	<= '1';
								slot1.available <= '0';
								wr_state			<= S_WRITING;
								wr_offset		<= to_unsigned(1, wr_offset'length);
								senaldebug0 <= x"05";
								
							else
								
								wr_state			<= S_FULL;
								stall				<= (others => '1');
								senaldebug0 <= x"06";
								
							end if; -- ENDIF LEV 2
						
						end if; -- ENDIF LEV 1
					
					when S_WRITING =>
					
						senaldebug0 <= x"d0";
					
						if(slot0.writing = '1') then -- IF LEV 1
						
							wr_offset	<= unsigned(wr_offset) + 1;
							addr_a 		<= std_logic_vector( unsigned(c_slot0_base) + unsigned(wr_offset) );
							write_a 		<= '1';
							senaldebug0 <= x"07";
							if( (snk_valid(0) = '1' and tagger_snk_i(0).adr = c_WRF_DATA) or (snk_valid(1) = '1' and tagger_snk_i(1).adr = c_WRF_DATA) ) then
								word_count <= std_logic_vector(unsigned(word_count) + 1);
								senaldebug0 <= x"08";
							end if;
							
							
							if slot0.source = '0' then
								if ((snk_valid(0) = '1' and tagger_snk_i(0).adr = c_WRF_DATA) and unsigned(word_count) = (to_unsigned(0,word_count'length)) and tagger_snk_i(0).dat(8) = '1') then
									slot0.is_multicast <= '1';
								end if;
--								elsif ( broadcast0 = '1' and unsigned(word_count) = (to_unsigned(1,word_count'length)) and tagger_snk_i(0).dat /= x"FFFF") then
--									broadcast0 := '0';
--								elsif ( broadcast0 = '1' and unsigned(word_count) = (to_unsigned(2,word_count'length)) and tagger_snk_i(0).dat = x"FFFF" ) then
--									slot0.is_broadcast <= '1';
--								end if;
							end if;
							
							if slot0.source = '1' then
								if ((snk_valid(1) = '1' and tagger_snk_i(1).adr = c_WRF_DATA) and unsigned(word_count) = (to_unsigned(0,word_count'length)) and tagger_snk_i(1).dat(8) = '1') then
									slot0.is_multicast <= '1';
--								elsif ( broadcast1 = '1' and unsigned(word_count) = (to_unsigned(1,word_count'length)) and tagger_snk_i(1).dat /= x"FFFF") then
--									broadcast1 := '0';
--								elsif ( broadcast1 = '1' and unsigned(word_count) = (to_unsigned(2,word_count'length)) and tagger_snk_i(1).dat = x"FFFF" ) then
--									slot0.is_broadcast <= '1';
								end if;
							end if;

							-- We won't need the is_hsr flag. With the current design every frame will have a tag.
							if( unsigned(word_count) = (to_unsigned(9,word_count'length)) ) then
								if( slot0.source = '0' ) then
									if( tagger_snk_i(0).dat = x"88f7") then--tagger_snk_i(0).dat = x"892f" or tagger_snk_i(0).dat = x"88f7") then
										--slot0.is_hsr <= '1';
										slot0.is_ptp <= '1';
										slot0.written <= '1';
										senaldebug0 <= x"09";
									else
										--slot0.is_hsr <= '0';
										slot0.is_ptp <= '0';
										slot0.written <= '1';
										bound_ep0_count <= std_logic_vector(unsigned(bound_ep0_count) + 1);
										senaldebug0 <= x"0A";
									end if;
								elsif(slot0.source = '1') then
									if( tagger_snk_i(1).dat = x"88f7") then--tagger_snk_i(0).dat = x"892f" or tagger_snk_i(0).dat = x"88f7") then
										--slot0.is_hsr <= '1';
										slot0.is_ptp <= '1';
										slot0.written <= '1';
										senaldebug0 <= x"0B";
									else
										--slot0.is_hsr <= '0';
										slot0.is_ptp <= '0';
										slot0.written <= '1';
										bound_ep1_count <= std_logic_vector(unsigned(bound_ep1_count) + 1);										
										senaldebug0 <= x"0C";
									end if;
								end if;
							end if;
--							if( unsigned(word_count) = (to_unsigned(9,word_count'length)) ) then
--								if( slot0.source = '0' ) then
--									if( tagger_snk_i(0).dat = x"88f7" and slot0.is_hsr = '1' ) then
--										slot0.is_ptp <= '1';
--									else
--										slot0.is_ptp <= '0';
--									end if;
--								elsif(slot0.source = '1') then
--									if( tagger_snk_i(1).dat = x"88f7" and slot0.is_hsr = '1' ) then
--										slot0.is_ptp <= '1';
--									else
--										slot0.is_ptp <= '0';
--									end if;
--								end if;
--							end if;
--							
							if(slot0.source = '0') then -- IF LEV 2
								
								din_a <= snk_valid(0) & tagger_snk_i(0).adr & tagger_snk_i(0).sel & tagger_snk_i(0).dat;
								senaldebug0 <= x"0D";
								if(eof(0) = '1') then -- IF LEV 3
									
									din_a <= snk_valid(0) & c_sig_eof;
									wr_state <= S_EOF;
									slot0.writing <= '0';
									-- slot0.written <= '1';
									senaldebug0 <= x"0E";
									
								end if;  -- ENDIF LEV 3
								
							elsif(slot0.source = '1') then
							
								din_a <= snk_valid(1) & tagger_snk_i(1).adr & tagger_snk_i(1).sel & tagger_snk_i(1).dat;
								senaldebug0 <= x"0F";
								if(eof(1) = '1') then -- IF LEV 3
									
									din_a <= snk_valid(1) & c_sig_eof;
									wr_state <= S_EOF;
									slot0.writing <= '0';
									-- slot0.written <= '1';
									senaldebug0 <= x"10";
									
								end if; -- ENDIF LEV 3
							
							end if; -- ENDIF LEV 2
							
						elsif(slot1.writing = '1') then
						
							wr_offset	<= unsigned(wr_offset) + 1;
							addr_a 		<= std_logic_vector( unsigned(c_slot1_base) + unsigned(wr_offset) );
							write_a		<= '1';
							senaldebug0 <= x"11";
							if(snk_valid(0) = '1' or snk_valid(1) = '1') then
								word_count <= std_logic_vector(unsigned(word_count) + 1);
							end if;
							
							
							if slot1.source = '0' then
								if ((snk_valid(0) = '1' and tagger_snk_i(0).adr = c_WRF_DATA) and unsigned(word_count) = (to_unsigned(0,word_count'length)) and tagger_snk_i(0).dat(8) = '1') then
									slot1.is_multicast <= '1';
--								elsif ( broadcast0 = '1' and unsigned(word_count) = (to_unsigned(1,word_count'length)) and tagger_snk_i(0).dat /= x"FFFF") then
--									broadcast0 := '0';
--								elsif ( broadcast0 = '1' and unsigned(word_count) = (to_unsigned(2,word_count'length)) and tagger_snk_i(0).dat = x"FFFF" ) then
--									slot1.is_broadcast <= '1';
								end if;
							end if;
							
							if slot1.source = '1' then
								if ((snk_valid(1) = '1' and tagger_snk_i(1).adr = c_WRF_DATA) and unsigned(word_count) = (to_unsigned(0,word_count'length)) and tagger_snk_i(1).dat(8) = '1') then
									slot1.is_multicast <= '1';
--								elsif ( broadcast1 = '1' and unsigned(word_count) = (to_unsigned(1,word_count'length)) and tagger_snk_i(1).dat /= x"FFFF") then
--									broadcast1 := '0';
--								elsif ( broadcast1 = '1' and unsigned(word_count) = (to_unsigned(2,word_count'length)) and tagger_snk_i(1).dat = x"FFFF" ) then
--									slot1.is_broadcast <= '1';
								end if;
							end if;


							if( unsigned(word_count) = (to_unsigned(9,word_count'length)) ) then
								if( slot1.source = '0' ) then
									if( tagger_snk_i(0).dat = x"88f7") then--tagger_snk_i(0).dat = x"892f" or tagger_snk_i(0).dat = x"88f7") then
										-- slot1.is_hsr <= '1';
										slot1.is_ptp <= '1';
										slot1.written <= '1';
										senaldebug0 <= x"12";
									else
										-- slot1.is_hsr <= '0';
										slot1.is_ptp <= '0';
										slot1.written <= '1';
										bound_ep0_count <= std_logic_vector(unsigned(bound_ep0_count) + 1);										
										senaldebug0 <= x"13";
									end if;
								elsif(slot1.source = '1') then
									if( tagger_snk_i(1).dat = x"88f7") then--tagger_snk_i(0).dat = x"892f" or tagger_snk_i(0).dat = x"88f7") then
										-- slot1.is_hsr <= '1';
										slot1.is_ptp <= '1';
										slot1.written <= '1';
										senaldebug0 <= x"14";
									else
										-- slot1.is_hsr <= '0';
										slot1.is_ptp <= '0';
										slot1.written <= '1';
										bound_ep1_count <= std_logic_vector(unsigned(bound_ep1_count) + 1);										
										senaldebug0 <= x"15";
									end if;
								end if;
							end if;

							
							if(slot1.source = '0') then -- IF LEV 2
								
								din_a <= snk_valid(0) & tagger_snk_i(0).adr & tagger_snk_i(0).sel & tagger_snk_i(0).dat;
								senaldebug0 <= x"16";

								if(eof(0) = '1') then -- IF LEV 3
									
									din_a <= snk_valid(0) & c_sig_eof;
									wr_state <= S_EOF;
									slot1.writing <= '0';
									-- slot1.written <= '1';
									senaldebug0 <= x"17";
									
								end if; -- ENDIF LEV 3
								
							elsif(slot1.source = '1') then 
								
								senaldebug0 <= x"18";

								din_a <= snk_valid(1) & tagger_snk_i(1).adr & tagger_snk_i(1).sel & tagger_snk_i(1).dat;
								if(eof(1) = '1') then  -- IF LEV 3
									
									din_a <= snk_valid(1) & c_sig_eof;
									wr_state <= S_EOF;
									slot1.writing <= '0';
									-- slot1.written <= '1';
									senaldebug0 <= x"19";
									
								end if;  -- ENDIF LEV 3
								
							end if; -- ENDIF LEV 2
							
						end if; -- ENDIF LEV 1
					
					when S_EOF =>
						
						-- wr_offset	<= unsigned(wr_offset) + 1;
						-- addr_a 		<= std_logic_vector( unsigned(c_slot1_base) + unsigned(wr_offset) );
						-- write_a	<= '1';
						-- din_a			<= (others => '0');

						if(slot0.available = '0' and slot1.available = '0') then
							
							wr_state 	<= S_FULL;
							-- stall 		<= (others => '1');
							senaldebug0 <= x"1A";
							
						else
						
							wr_state 	<= S_IDLE;
							stall			<= (others => '0');
							senaldebug0 <= x"1B";
						
						end if;
						if(debug) then
							wr_state <= S_IDLE;
						end if;
										
					when S_FULL =>
						
						write_a	<= '0';
						stall	<= (others => '1');
						senaldebug0 <= x"1C";
						
						if(slot0.available = '1' or slot1.available = '1') then
							
							stall 		<= (others => '0');
							wr_state 	<= S_IDLE;
							senaldebug0 <= x"1D";
						
						end if;
						

						if(debug) then
							wr_state <= S_IDLE;
						end if;
					
					when others =>					
				
				end case;
				
				if clr_cnt_i = '1' then
					bound_ep0_count <= (others => '0');
					bound_ep1_count <= (others => '0');
				end if;
			
			end if;
		end if;
	end process;


 -- TODO: only write data when the link is active.

	p_rd0_fsm : process(clk_i)
		variable addr : std_logic_vector(c_addr_size-1 downto 0);
	begin
		if(rising_edge(clk_i)) then
			if(rst_n_i = '0') then
				
				rd_state0 <= IDLE;
				
				slot0.reading_0	<= '0';
				slot0.finished_0  <= '0';
				
				slot1.reading_0	<= '0';
				slot1.finished_0  <= '0';
				senaldebug1 <= x"01";
				
			else
				
				write_b		<= '0';
				case rd_state0 is
					
					when IDLE =>
					
						senaldebug1 <= x"06";
						rd_offset_b			<= (others => '0');


						if(slot0.available = '1' and slot0.finished_0 = '1' and slot0.finished_1 = '1') then
							slot0.finished_0 <= '0';
							senaldebug1 <= x"02";
						end if;
						
						if(slot1.available = '1' and slot1.finished_0 = '1' and slot1.finished_1 = '1') then
							slot1.finished_0 <= '0';
							senaldebug1 <= x"03";
						end if;
						
						if((slot0.written = '1') and (snk_dreq(0) = '1') and link_ok_i(0) = '1' and slot0.finished_0 = '0') then
							rd_state0			<= READING;
							slot0.reading_0	<= '1';
							addr					:= c_slot0_base;
							senaldebug1 <= x"04";
							if(slot0.source = '1' and (slot0.is_ptp = '1' or slot0.is_multicast = '1')) then
								rd_state0			<= FINISH_CYCLE;
								slot0.reading_0	<= '0';
								slot0.finished_0	<= '1';
								senaldebug1			<= x"0D";
							end if;
						elsif((slot1.written = '1') and (snk_dreq(0) = '1') and link_ok_i(0) = '1' and slot1.finished_0 = '0') then
							rd_state0			<= READING;
							slot1.reading_0	<= '1';
							senaldebug1 <= x"05";
							addr					:= c_slot1_base;
							if(slot1.source = '1' and (slot1.is_ptp = '1' or slot1.is_multicast = '1')) then
								rd_state0			<= FINISH_CYCLE;
								slot1.reading_0	<= '0';
								slot1.finished_0	<= '1';
								senaldebug1			<= x"0E";
							end if;
						elsif(slot0.written = '1' and link_ok_i(0) = '0') then
							rd_state0 <= FINISH_CYCLE;
							slot0.reading_0 <= '0';
							slot0.finished_0 <= '1';
						elsif(slot1.written = '1' and link_ok_i(0) = '0') then
							rd_state0 <= FINISH_CYCLE;
							slot1.reading_0 <= '0';
							slot1.finished_0 <= '1';
						end if;
						
						addr_b <= std_logic_vector( unsigned(addr) + unsigned(rd_offset_b) );
					
					when READING =>
						
						senaldebug1 <= x"D0";
						if(snk_dreq(0) = '1') then
							
							addr_b <= std_logic_vector( unsigned(addr) + unsigned(rd_offset_b) );
							senaldebug1 <= x"07";
							
							if(dout_b(19 downto 18) = "11" and dout_b(7 downto 0) = x"FF") then
							-- SOF
								snk_fab_0.sof		<= '1';
								snk_fab_0.eof		<= '0';
								snk_fab_0.bytesel	<= '0';
								senaldebug1 <= x"08";
							elsif(dout_b(19 downto 18) = "11" and dout_b(7 downto 0) = x"AA") then
							-- EOF
								addr_b <= addr;
								snk_fab_0.sof		<= '0';
								snk_fab_0.eof		<= '1';
								snk_fab_0.bytesel	<= '0';
								senaldebug1 <= x"09";
								if(slot0.reading_0 = '1') then
									slot0.reading_0	<= '0';
									slot0.finished_0	<= '1';
									if(slot0.is_ptp = '0') then
										dup_ep0_count <= std_logic_vector(unsigned(dup_ep0_count) + 1);
									end if;
									senaldebug1 <= x"0A";
								elsif(slot1.reading_0 = '1') then
									slot1.reading_0	<= '0';
									slot1.finished_0 	<= '1';
									if(slot1.is_ptp = '0') then
										dup_ep0_count <= std_logic_vector(unsigned(dup_ep0_count) + 1);
									end if;
									senaldebug1 <= x"0B";
								end if;
								rd_state0 			<= FINISH_CYCLE;
							
                     else
								snk_fab_0.sof		<= '0';
								snk_fab_0.eof		<= '0';
								snk_fab_0.bytesel	<= not dout_b(16); -- intrigued? See state DATA of FSM in ep_rx_wb_master.
								snk_fab_0.data		<= dout_b(15 downto 0);
								snk_fab_0.addr		<= dout_b(19 downto 18);
								snk_fab_0.dvalid	<= dout_b(20);
								senaldebug1 <= x"0C";
							
							end if;
							
							rd_offset_b <= unsigned(rd_offset_b) + 1;
							
						end if;
						
					when FINISH_CYCLE =>
					
						snk_fab_0.dvalid <= '0';
						snk_fab_0.sof <= '0';
						snk_fab_0.eof <= '0';
						rd_state0	  <= IDLE;
						senaldebug1 <= x"10";
					
					when others =>
					
				end case;
				
				if clr_cnt_i = '1' then
					dup_ep0_count <= (others => '0');
				end if;
			
			end if;
		end if;
	end process;
	
	p_rd1_fsm : process(clk_i)
		variable addr : std_logic_vector(c_addr_size-1 downto 0);
	begin
		if(rising_edge(clk_i)) then
			if(rst_n_i = '0') then
				
				rd_state1 <= IDLE;
				
				slot0.reading_1	<= '0';
				slot0.finished_1  <= '0';
				
				slot1.reading_1	<= '0';
				slot1.finished_1  <= '0';
				senaldebug2 <= x"01";
			else
			
				write_c		<= '0';
				senaldebug2 <= x"02";
				case rd_state1 is
					
					when IDLE =>
						
						senaldebug2 <= x"D1";
						rd_offset_c 		<= (others => '0');
					
						if(slot0.available = '1' and slot0.finished_0 = '1' and slot0.finished_1 = '1') then
							slot0.finished_1 <= '0';
							senaldebug2 <= x"03";
						end if;
						
						if(slot1.available = '1' and slot1.finished_0 = '1' and slot1.finished_1 = '1') then
							slot1.finished_1 <= '0';
							senaldebug2 <= x"04";
						end if;
						
						if((slot0.written = '1') and (snk_dreq(1) = '1') and link_ok_i(1) = '1' and slot0.finished_1 = '0') then
							rd_state1			<= READING;
							slot0.reading_1	<= '1';
							addr					:= c_slot0_base;
							senaldebug2 <= x"05";
							if(slot0.source = '0' and (slot0.is_ptp = '1' or slot0.is_multicast = '1')) then
								rd_state1			<= FINISH_CYCLE;
								slot0.reading_1	<= '0';
								slot0.finished_1	<= '1';
								senaldebug2 <= x"06";
							end if;							
						elsif((slot1.written = '1') and (snk_dreq(1) = '1') and link_ok_i(1) = '1' and slot1.finished_1 = '0') then
							rd_state1			<= READING;
							slot1.reading_1	<= '1';
							addr					:= c_slot1_base;
							senaldebug2 <= x"07";
							if(slot1.source = '0' and (slot1.is_ptp = '1' or slot1.is_multicast = '1')) then
								rd_state1			<= FINISH_CYCLE;
								slot1.reading_1	<= '0';
								slot1.finished_1	<= '1';
								senaldebug2 <= x"08";
							end if;
						elsif(slot0.written = '1' and link_ok_i(1) = '0') then
							rd_state1 <= FINISH_CYCLE;
							slot0.reading_1 <= '0';
							slot0.finished_1 <= '1';
						elsif(slot1.written = '1' and link_ok_i(1) = '0') then
							rd_state1 <= FINISH_CYCLE;
							slot1.reading_1 <= '0';
							slot1.finished_1 <= '1';
						end if;
					
					when READING =>
					
						senaldebug2 <= x"D0";
					
						if(snk_dreq(1) = '1') then
							
							addr_c <= std_logic_vector( unsigned(addr) + unsigned(rd_offset_c) );
							senaldebug2 <= x"09";
							
							if(dout_c(19 downto 18) = "11" and dout_c(7 downto 0) = x"FF") then
							-- SOF
								snk_fab_1.sof		<= '1';
								snk_fab_1.eof		<= '0';
								snk_fab_1.bytesel	<= '0';
								senaldebug2 <= x"0A";
							elsif(dout_c(19 downto 18) = "11" and dout_c(7 downto 0) = x"AA") then
							-- EOF
								addr_c <= addr;
								snk_fab_1.sof		<= '0';
								snk_fab_1.eof		<= '1';
								snk_fab_1.bytesel	<= '0';
								if(slot0.reading_1 = '1') then
									senaldebug2 <= x"0B";
									slot0.reading_1	<= '0';
									slot0.finished_1	<= '1';
									if(slot0.is_ptp = '0') then
										dup_ep1_count <= std_logic_vector(unsigned(dup_ep1_count) + 1);
									end if;
								elsif(slot1.reading_1 = '1') then
									slot1.reading_1	<= '0';
									slot1.finished_1 	<= '1';
									if(slot1.is_ptp = '0') then
										dup_ep1_count <= std_logic_vector(unsigned(dup_ep1_count) + 1);									
									end if;
									senaldebug2 <= x"0C";
								end if;
								rd_state1 			<= FINISH_CYCLE;
                        
                     elsif rd_offset_c = 8 then
                     
                        snk_fab_1.sof		<= '0';
								snk_fab_1.eof		<= '0';
								snk_fab_1.bytesel	<= not dout_c(16); -- intrigued? See state DATA of FSM in ep_rx_wb_master.
								
                        -- Reading FSM number 2 is responsible for inserting the correct LaneID bit
                        -- (one of the fields of the Path ID):
                        snk_fab_1.data		<= (dout_c(15 downto 0) and x"0001");
								snk_fab_1.addr		<= dout_c(19 downto 18);
								snk_fab_1.dvalid	<= dout_c(20);
								senaldebug1 <= x"0C";
                        
							else
								snk_fab_1.sof		<= '0';
								snk_fab_1.eof		<= '0';
								snk_fab_1.bytesel	<= not dout_c(16); -- intrigued? See state DATA of FSM in ep_rx_wb_master.
								snk_fab_1.data		<= dout_c(15 downto 0);
								snk_fab_1.addr		<= dout_c(19 downto 18);
								snk_fab_1.dvalid	<= dout_c(20);
								senaldebug2 <= x"0D";
							
							end if;
							
							rd_offset_c <= unsigned(rd_offset_c) + 1;
							
						end if;
					
					when FINISH_CYCLE =>
						
						senaldebug2 <= x"0E";
						snk_fab_1.sof <= '0';
						snk_fab_1.eof <= '0';
						snk_fab_1.dvalid <= '0';
						rd_state1	  <= IDLE;
					
					when others =>
					
				end case;
				
				if clr_cnt_i = '1' then
					dup_ep1_count <= (others => '0');
				end if;
			
			end if;
		end if;
	end process;

	bound_ep0_count_o		<= bound_ep0_count;
	bound_ep1_count_o		<= bound_ep1_count;
	dup_ep0_count_o		<= dup_ep0_count;
	dup_ep1_count_o		<= dup_ep0_count;
	
--	snk_fab_0.dvalid <= dout_b(20) when (slot0.reading_0 = '1' or slot1.reading_0 = '1') else '0';
--	snk_fab_1.dvalid <= dout_c(20) when (slot0.reading_1 = '1' or slot1.reading_1 = '1') else '0';
	

  p_gen_ack : process(clk_i)
  begin
    if rising_edge(clk_i) then
      tagger_snk_o(0).ack <= snk_valid(0);
		tagger_snk_o(1).ack <= snk_valid(1);
    end if;
  end process;
  
  tagger_snk_o(0).stall	<= stall(0);
  tagger_snk_o(1).stall <= stall(1);
  
	U_master_ep0 : ep_rx_wb_master
		generic map(
			g_ignore_ack	=> true,
			g_cyc_on_stall => false)
		port map(
			clk_sys_i 		=> clk_i,
			rst_n_i			=> rst_n_i,
			snk_fab_i		=> snk_fab_0,
			snk_dreq_o		=> snk_dreq(0),
			
			src_wb_o			=> ep_src_o_int(0),
			src_wb_i			=>	ep_src_i(0)
		);
	
	U_master_ep1 : ep_rx_wb_master
		generic map(
			g_ignore_ack	=> true,
			g_cyc_on_stall => false)
		port map(
			clk_sys_i 		=> clk_i,
			rst_n_i			=> rst_n_i,
			snk_fab_i		=> snk_fab_1,
			snk_dreq_o		=> snk_dreq(1),
			
			src_wb_o			=> ep_src_o_int(1),
			src_wb_i			=> ep_src_i(1)
		);

	 ep_src_o <= ep_src_o_int;

--	ep_src_o <= tagger_snk_i;
--	tagger_snk_o <= ep_src_i;
	
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


	
	
	

end behavioral;
