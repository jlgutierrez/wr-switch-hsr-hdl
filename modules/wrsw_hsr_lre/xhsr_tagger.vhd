-------------------------------------------------------------------------------
-- Title      : HSR Link Redundancy Entity - top level
-- Project    : White Rabbit
-------------------------------------------------------------------------------
-- File       : xhsr_tagger.vhd
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
use work.wrsw_shared_types_pkg.all;
use work.mpm_pkg.all;
use work.genram_pkg.all;
use work.endpoint_private_pkg.all;




entity xhsr_tagger is
  generic(
    g_adr_width : integer := 2;
    g_dat_width : integer :=16;
    g_size    : integer := 1024; -- things for the fifo
    g_with_fc : boolean := false -- things for the fifo
    --g_num_ports : integer
    );
  port(

    rst_n_i : in  std_logic;
    clk_i   : in  std_logic;
    
-------------------------------------------------------------------------------
-- pWB  : output (comes from SWCORE)
-------------------------------------------------------------------------------
-- joselj 2016-02-25: input comes

    snk_i : in  t_wrf_sink_in;
    snk_o : out  t_wrf_sink_out;

-------------------------------------------------------------------------------
-- pWB : input (goes to ENDPOINT)
-------------------------------------------------------------------------------  
-- joselj 2016-02-25: output goes

    src_i : in  t_wrf_source_in;
    src_o : out  t_wrf_source_out;
    
    
-------------------------------------------------------------------------------
-- input/output (comes/goes to HSR_SEQUENCER)
-------------------------------------------------------------------------------  
	req_tag : out std_logic;
	seq_n : in std_logic_vector (15 downto 0);
	seq_valid : in std_logic

    );
end xhsr_tagger;

architecture behavoural of xhsr_tagger is

type t_state is (WAIT_FRAME, 
					DATA,
					FLUSH_STALL,
					DISCARD_FRAME,
					INSERT_TAG,
					END_FRAME);
					
type t_seq_state is (IDLE, WAIT_REQ);

  signal seq_state : t_seq_state;
  
  -- new crap
  signal hdr_offset : std_logic_vector(13 downto 0);
  signal at_ethertype     : std_logic;
  signal is_tagged        : std_logic;
  signal stored_ethertype : std_logic_vector(15 downto 0);
  signal is_tag_inserted: std_logic;
  signal stall_req  : std_logic;
  
  -- general signals
  signal state   : t_state;
  signal counter : unsigned(2 downto 0);

  signal vut_rd_vid : std_logic_vector(11 downto 0);
  signal vut_wr_vid : std_logic_vector(11 downto 0);
  signal vut_untag, vut_untag_reg  : std_logic;

  signal vut_stored_tag       : std_logic_vector(15 downto 0);
  signal vut_stored_ethertype : std_logic_vector(15 downto 0);

  signal mem_addr_muxed : std_logic_vector(9 downto 0);
  signal mem_rdata      : std_logic_vector(17 downto 0);
  signal src_stall_d0    : std_logic;
  signal flush_ethtype  : std_logic;
  
  signal stored_fab       : t_ep_internal_fabric;
  signal bis_stored_fab   : t_ep_internal_fabric;
  signal bis_bis_stored_fab   : t_ep_internal_fabric;
  
  signal snk_fab_conv  :  t_ep_internal_fabric; -- comes from swc-wb
  signal src_fab_o  :  t_ep_internal_fabric; -- goes to fab/wb conv
  signal ep_src_o_int : t_wrf_source_out;
  signal snk_dreq : std_logic := '0';
  
  -- hsr signals
  signal hsr_eth_type : std_logic_vector(15 downto 0) := x"892F";
  signal seq_number : std_logic_vector(15 downto 0) := x"0000";
  signal hsr_path : std_logic_vector(3 downto 0) := x"0";
  signal hsr_size : std_logic_vector(11 downto 0) := x"000";
  signal hsr_inserted : std_logic;
  signal request_tag : std_logic := '0';
  signal current_seq_n : std_logic_vector(15 downto 0);
  signal seq_n_is_valid : std_logic;
  
  -- fifo constants
  constant c_drop_threshold    : integer := g_size - 2;
  constant c_release_threshold : integer := g_size * 7 / 8;
  
  -- fifo signals  
  signal q_in, q_out             : std_logic_vector(22 downto 0);
  signal q_usedw                 : std_logic_vector(f_log2_size(g_size)-1 downto 0);
  signal q_empty                 : std_logic;
  signal q_reset                 : std_logic;
  signal q_rd                    : std_logic;
  signal q_drop                  : std_logic;
  signal q_in_valid, q_out_valid : std_logic;
  signal q_aempty, q_afull       : std_logic;
  
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
  
  -- CONTROL SIGNALS --
  signal sof_p1, eof_p1, abort_p1, error_p1 : std_logic;
  signal snk_cyc_d0                         : std_logic;
  signal snk_stb_d0							: std_logic;
  
  
  type t_write_state is(WAIT_FRAME, DATA);
  
  signal snk_valid : std_logic;
  
  -- DEBUG --
  signal debug_state : std_logic_vector (2 downto 0);
  signal debug_insert : std_logic_vector (2 downto 0);
  signal fab_2_wb_o : t_wrf_source_out;
  signal debug_seq : std_logic_vector (2 downto 0);
  signal ack_counter : integer := 0;
  signal eof_counter : integer := 0;
  signal sof_counter : integer := 0;
  
  begin  -- behavioral
  
  at_ethertype <= hdr_offset(7) and snk_i.stb;
  --at_ethertype <= hdr_offset(6) and snk_valid;

  -- not used, but probably for the future... --
  BUF_FIFO : generic_sync_fifo
    generic map (
      g_data_width => 23, -- data, sof, eof, addr, dvalid, bytesel & error
      g_size       => g_size,
      g_with_almost_empty => true,
      g_with_almost_full  => true,
      g_almost_empty_threshold  => c_release_threshold,
      g_almost_full_threshold   => c_drop_threshold,
      g_with_count              => g_with_fc)
    port map (
      rst_n_i        => rst_n_i,
      clk_i          => clk_i,
      d_i            => q_in,
      we_i           => q_in_valid,
      q_o            => q_out,
      rd_i           => q_rd,
      empty_o        => q_empty,
      full_o         => open,
      almost_empty_o => q_aempty,
      almost_full_o  => q_afull,
      count_o        => q_usedw);
      
      
  p_detect_frame : process(clk_i)
  begin
    if rising_edge(clk_i) then
      if rst_n_i = '0' then
        snk_cyc_d0 <= '0';
      else
        snk_cyc_d0 <= snk_i.cyc;
        snk_stb_d0 <= snk_i.stb; -- new test
      end if;
    end if;
  end process;

  -- convert wb to fab
  sof_p1 <= not snk_cyc_d0 and snk_i.cyc;
  eof_p1 <= snk_cyc_d0 and not snk_i.cyc;
  
  p_debug_counter : process(clk_i)
  begin
    if rising_edge(clk_i) then
      if rst_n_i = '0' then
      else
        if sof_p1 = '1' then
           sof_counter <= sof_counter+1;
        end if;
        if eof_p1 = '1' then
           eof_counter <= eof_counter+1;
        end if;        
      end if;
    end if;
  end process;
  
  --snk_valid <= snk_i.cyc and snk_i.stb and snk_i.we and not src_i.stall;
  snk_valid <= snk_i.cyc and snk_i.stb and snk_i.we; 

p_tag : process(clk_i)
    variable v_stored_fab                    : t_ep_internal_fabric;
    variable v_src_fab                       : t_ep_internal_fabric;
    variable v_next_state                    : t_state;
    variable LSDUsize						 : std_logic_vector(15 downto 4) := "000000101101";
    variable hsr_path						 : std_logic_vector(3 downto 0) := "0001";
    variable wait_for_seq_valid				 : std_logic := '0';
  begin
    if rising_edge(clk_i) then
      if rst_n_i = '0' then
        hdr_offset(hdr_offset'left downto 1) <= (others => '0');
        hdr_offset(0)    <= '1';
        state            <= WAIT_FRAME;
        stall_req        <= '0';
        is_tag_inserted  <= '0';
        is_tagged        <= '0';
        src_fab_o.eof    <= '0';
        src_fab_o.error  <= '0';
        src_fab_o.dvalid <= '0';
        wait_for_seq_valid := '0';
        debug_state <= "000";
        debug_insert <= "000";
        
      else
        --if(snk_o.err = '1') then -- jummm...
        --  state <= DISCARD_FRAME;
        --else

          case state is
            when WAIT_FRAME =>
              stall_req       <= '0';
              src_fab_o.eof   <= '0';
              src_fab_o.error <= '0';
              is_tagged       <= '0';
              is_tag_inserted <= '0';
              debug_state <= "001";

              if(sof_p1 = '1') then
                hdr_offset(hdr_offset'left downto 1) <= (others => '0');
                hdr_offset(0)                        <= '1';
                state                                <= DATA;
              end if;

            when DATA =>
			  debug_state <= "010";
              v_src_fab.eof   := '0';
              v_src_fab.error := '0';
              v_next_state    := DATA;

              if (snk_valid = '1' and src_i.stall = '1') then
                v_stored_fab.bytesel := not snk_i.sel(0);
                v_stored_fab.data    := snk_i.dat;
                v_stored_fab.addr    := snk_i.adr;
                v_stored_fab.dvalid  := '1';
                v_next_state         := FLUSH_STALL;
              else
                v_src_fab.addr    := snk_i.adr;
                v_src_fab.data    := snk_i.dat;
                --v_src_fab.dvalid  := not src_i.stall and snk_valid;
                v_src_fab.dvalid  := not src_i.stall and snk_i.stb;
                v_src_fab.bytesel := not snk_i.sel(0);
              end if;

              if(at_ethertype = '1') then
                stored_ethertype <= snk_i.dat;

                if(snk_i.dat = x"892f") then  -- got a HSR tagged frame (should never happen!!)
                  is_tagged <= '1';
                else
                  is_tag_inserted  <= '1';
                  v_src_fab.dvalid := '0';
                  request_tag <= '1';
                  v_next_state     := INSERT_TAG;
                  is_tagged <= '0';
                end if;
              end if;

              if(eof_p1 = '1') then
                if(src_i.stall = '0') then
                  v_src_fab.eof := '1';
                  v_next_state  := WAIT_FRAME;
                else
                  v_next_state := END_FRAME;
                end if;
              end if;

              if(snk_valid = '1') then -- only snk_i.stb?
                hdr_offset <= hdr_offset(hdr_offset'left-1 downto 0) & '0';
              end if;

              if(v_next_state = INSERT_TAG) then
                debug_state <= "111";
                src_fab_o.eof     <= '0';              
                src_fab_o.dvalid  <= '1';
                src_fab_o.error   <= '0';
                src_fab_o.addr    <= c_WRF_DATA;
                src_fab_o.data    <= x"892f";
                src_fab_o.bytesel <= '0';
              else
                src_fab_o.eof     <= v_src_fab.eof;              
                src_fab_o.dvalid  <= v_src_fab.dvalid;
                src_fab_o.error   <= v_src_fab.error;
                src_fab_o.addr    <= v_src_fab.addr;
                src_fab_o.data    <= v_src_fab.data;
                src_fab_o.bytesel <= v_src_fab.bytesel;
              end if;

              stored_fab <= v_stored_fab;
              state      <= v_next_state;
              
            when END_FRAME =>
              debug_state <= "110";
              if(src_i.stall = '0')then
                src_fab_o.eof    <= '1';
                src_fab_o.dvalid <= '0';
                state            <= WAIT_FRAME;
              end if;
              
            when DISCARD_FRAME =>
              debug_state <= "100";
              if(src_i.stall = '0') then
                src_fab_o.error  <= '1';
                src_fab_o.dvalid <= '0';
                state            <= WAIT_FRAME;
              end if;

            when FLUSH_STALL =>
              debug_state <= "011";
              if(src_i.stall = '0')then
                src_fab_o.addr    <= stored_fab.addr;
                src_fab_o.data    <= stored_fab.data;
                src_fab_o.dvalid  <= stored_fab.dvalid;
                src_fab_o.bytesel <= stored_fab.bytesel;
                state             <= DATA;
              else
                src_fab_o.DATA   <= (others => 'X');
                src_fab_o.dvalid <= '0';
              end if;

            when INSERT_TAG =>
              debug_state <= "101";
              debug_insert <= "000";
              src_fab_o.dvalid <= '0';
              request_tag <= '0';

              if(src_i.stall = '0') then
				
				if(hdr_offset(8) = '1') then -- insert HSR PATH + LSDUsize (TBD: LSDUsize is not correctly generated...)
				   debug_insert <= "001";
				   src_fab_o.eof     <= '0';              
				   src_fab_o.dvalid  <= '1';
                   src_fab_o.error   <= '0';
                   src_fab_o.addr    <= c_WRF_DATA;
                   src_fab_o.data <= hsr_path&LSDUsize;
                   src_fab_o.bytesel <= '0';
                   -- we store whatever it's comming.. (twice...)
                   stored_fab.bytesel <= not snk_i.sel(0);
                   stored_fab.data    <= snk_i.dat;
                   stored_fab.addr    <= snk_i.adr;
                   stored_fab.dvalid  <= '1';
                   stall_req <= '1';
				end if;
				
				--if(hdr_offset(9) = '1' AND seq_valid= '1') then -- insertar n_seq
				if(hdr_offset(9) = '1') then -- insert HSR sequence number
				   debug_insert <= "010";
				   src_fab_o.eof     <= '0';              
				   src_fab_o.dvalid  <= '1';
                   src_fab_o.error   <= '0';
                   src_fab_o.addr    <= c_WRF_DATA;
                   src_fab_o.data    <= current_seq_n;
                   src_fab_o.bytesel <= '0';
                   -- we store whatever it's comming.. (twice...)
                   bis_stored_fab.bytesel <= not snk_i.sel(0);
                   bis_stored_fab.data    <= snk_i.dat;
                   bis_stored_fab.addr    <= snk_i.adr;
                   bis_stored_fab.dvalid  <= '1';
                --elsif(hdr_offset(9) = '1' AND seq_valid= '0') then -- wait for n_seq
                --   debug_insert <= "010";
                --   stall_req <= '1';
                --   wait_for_seq_valid := '1';
                end if;
				
				if(hdr_offset(10) = '1') then
				  debug_insert <= "011";
                  src_fab_o.addr   <= c_WRF_DATA;
                  src_fab_o.data   <= stored_ethertype;
                  src_fab_o.dvalid <= '1';
                end if;
                
                if(hdr_offset(11) = '1') then -- insertar 1st stored data
                  debug_insert <= "100";
				  src_fab_o.addr    <= stored_fab.addr;
                  src_fab_o.data    <= stored_fab.data;
                  src_fab_o.dvalid  <= stored_fab.dvalid;
                  src_fab_o.bytesel <= stored_fab.bytesel;
                  stall_req <= '0';
				end if;
				
				if(hdr_offset(12) = '1') then -- insertar 2nd stored data
				  debug_insert <= "101";
				  src_fab_o.addr    <= bis_stored_fab.addr;
                  src_fab_o.data    <= bis_stored_fab.data;
                  src_fab_o.dvalid  <= bis_stored_fab.dvalid;
                  src_fab_o.bytesel <= bis_stored_fab.bytesel;
                  state            <= DATA;       
				end if;

				--if (wait_for_seq_valid = '0') then 
                  hdr_offset <= hdr_offset(hdr_offset'left-1 downto 0) & '0';
                --end if;
              else
                src_fab_o.dvalid <= '0';
              end if;
          end case;
        --end if;
      end if;
    end if;
  end process;
	
  --process(clk_i)
  --begin
    --if rising_edge(clk_i) then	  
      
      --snk_o <= src_i;
      --src_o <= snk_i;
      
    --end if;
  --end process;
  
    U_master_ep0 : ep_rx_wb_master
	generic map(
		g_ignore_ack	=> true)
	port map(
		clk_sys_i 		=> clk_i,
		rst_n_i			=> rst_n_i,
		snk_fab_i		=> src_fab_o, -- src_fab_o
		--snk_fab_i	    => snk_fab_conv,
		snk_dreq_o		=> snk_dreq, --snk_dreq
		
		--src_wb_o			=> src_o,
		src_wb_o			=> fab_2_wb_o,
		src_wb_i			=>	src_i
	);
	
	src_o <= fab_2_wb_o; -- debug...
  
  p_gen_ack : process(clk_i)
  begin
    if rising_edge(clk_i) then
      snk_o.ack <= '1'; -- shitty hack (TB FIX)
      if (snk_valid = '1') then
         ack_counter <= ack_counter+1;
      end if;
    end if;
  end process;
  
  snk_o.stall <= src_i.stall OR not snk_dreq OR stall_req;
  src_fab_o.sof <= sof_p1;
  
  p_req_hsr_seq : process(clk_i)
  begin
    if rising_edge(clk_i) then
      if rst_n_i = '0' then
	    seq_state <= IDLE;
	    req_tag <= '0'; 
	    seq_n_is_valid <= '0';
	    debug_seq <= "000";
	  else
	     case seq_state is

	       when IDLE =>
			 debug_seq <= "001";
	         if (request_tag = '0') then
	           req_tag <= '0';
	           debug_seq <= "010";
	         else 
	           req_tag <= '1';
	           seq_n_is_valid <= '0';
	           seq_state <= WAIT_REQ;
	           debug_seq <= "011";
	         end if;         
			
	       when WAIT_REQ =>
	         debug_seq <= "100";
	         req_tag <= '0'; -- removing request too soon maybe?
	         if (seq_valid = '1') then
	           current_seq_n <= seq_n;
	           seq_n_is_valid <= '1';
	           req_tag <= '0';
	           seq_state <= IDLE;
	           debug_seq <= "101";
	         end if;
	       
	     end case;
	  end if;
    end if;
  end process;
  
    -- DEBUG --	
    --cs_icon : chipscope_icon
	--port map(
		--CONTROL0	=> CONTROL0
	--);
	--cs_ila : chipscope_ila
	--port map(
		--CLK		=> clk_i,
		--CONTROL	=> CONTROL0,
		--TRIG0		=> TRIG0,
		--TRIG1		=> TRIG1,
		--TRIG2		=> TRIG2,
		--TRIG3		=> TRIG3
	--);
	
    trig0(2 downto 0) <= debug_state;
    trig0(3) <= sof_p1;
    trig0(4) <= eof_p1;
    
	trig0(20 downto 5)	<= snk_i.dat;
	trig0(21)				<= snk_i.cyc;
	trig0(22)				<= snk_i.stb;
	trig0(24 downto 23)	<= snk_i.adr;
	trig0(25) <= snk_valid;	
	trig0(26) <= snk_dreq; -- wisbone master asks for stall...
	trig0(27) <= src_i.stall;
	trig0(28) <= snk_dreq;
	trig0(29) <= stall_req;
	
	
	trig1(15 downto 0)	<= fab_2_wb_o.dat;
	--trig1(7 downto 0) <= std_logic_vector(to_unsigned(ack_counter,8));
	trig1(16)				<= fab_2_wb_o.cyc;
	trig1(17)				<= fab_2_wb_o.stb;
	trig1(19 downto 18)	<= fab_2_wb_o.adr;
	
	trig2(13 downto 0) <= hdr_offset;
	trig2(21 downto 14) <= std_logic_vector(to_unsigned(sof_counter,8));
	trig2(29 downto 22) <= std_logic_vector(to_unsigned(eof_counter,8));
	
	
	trig3(2 downto 0) <= debug_insert;
	trig3(3) <= seq_valid;
	trig3(4) <= seq_n_is_valid;
	trig3(5) <= request_tag;
	trig3(21 downto 6) <= current_seq_n;
	trig3(24 downto 22) <= debug_seq;

	


end behavoural;
