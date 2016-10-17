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
    g_with_fc : boolean := false; -- things for the fifo
    g_hsr_path_id : std_logic_vector(3 downto 0)
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
  signal cnt	: integer := 0;
  signal n_words	: integer := 0;
    
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
  signal request_nseq : std_logic := '0';
  signal current_seq_n : std_logic_vector(15 downto 0);
  signal seq_n_is_valid : std_logic;
  
  -- fifo constants
  constant c_drop_threshold    : integer := g_size - 2;
  constant c_release_threshold : integer := g_size * 7 / 8;
  
  -- fifo signals  
  signal q_in, q_out             : std_logic_vector(25 downto 0);
  signal q_usedw                 : std_logic_vector(f_log2_size(g_size)-1 downto 0);
  signal q_empty                 : std_logic;
  signal q_reset                 : std_logic;
  signal q_rd                    : std_logic;
  signal q_drop                  : std_logic;
  signal q_in_valid, q_out_valid : std_logic;
  signal q_aempty, q_afull       : std_logic;
  
  -- fifo signals  
  signal qs_in, qs_out            : std_logic_vector(11 downto 0);
  signal qs_usedw                 : std_logic_vector(f_log2_size(g_size)-1 downto 0);
  signal qs_empty                 : std_logic;
  signal qs_reset                 : std_logic;
  signal qs_rd                    : std_logic;
  signal qs_drop                  : std_logic;
  signal qs_in_valid, qs_out_valid : std_logic;
  signal qs_aempty, qs_afull       : std_logic;
  
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
  signal snk_cyc_d1                         : std_logic;
  signal snk_stb_d0							: std_logic;
  signal snk_sel_d0							: std_logic_vector(1 downto 0);
  
  signal wait_complete						: std_logic;
  signal inserting_tag						: std_logic;
  
  
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

  BUF_FIFO : generic_sync_fifo
    generic map (
      g_data_width => 26, --  snk_valid & eof_p1 & sof_p1 & snk_i.sel & snk_i.we & snk_i.stb & snk_i.cyc & snk_i.dat & snk_i.adr;
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
      
  BUF_FIFO_SIZE : generic_sync_fifo
    generic map (
      g_data_width => 12, -- size up to 2048
      g_size       => g_size,
      g_with_almost_empty => true,
      g_with_almost_full  => true,
      g_almost_empty_threshold  => c_release_threshold,
      g_almost_full_threshold   => c_drop_threshold,
      g_with_count              => g_with_fc)
    port map (
      rst_n_i        => rst_n_i,
      clk_i          => clk_i,
      d_i            => qs_in,
      we_i           => qs_in_valid,
      q_o            => qs_out,
      rd_i           => qs_rd,
      empty_o        => qs_empty,
      full_o         => open,
      almost_empty_o => qs_aempty,
      almost_full_o  => qs_afull,
      count_o        => qs_usedw);
      
      
  p_detect_frame : process(clk_i)
  begin
    if rising_edge(clk_i) then
      if rst_n_i = '0' then
        snk_cyc_d0 <= '0';
        snk_cyc_d1 <= '0';
        snk_sel_d0 <= "00";
      else
        snk_cyc_d0 <= snk_i.cyc;
        snk_cyc_d1 <= snk_cyc_d0;
        snk_sel_d0 <= snk_i.sel;
      end if;
    end if;
  end process;

  sof_p1 <= not snk_cyc_d0 and snk_i.cyc;
  eof_p1 <= snk_cyc_d0 and not snk_i.cyc;
 
  snk_valid <= snk_i.cyc and snk_i.stb and snk_i.we and not src_i.stall;
  q_in <= snk_valid & eof_p1 & sof_p1 & snk_i.sel & snk_i.we & snk_i.stb & snk_i.cyc & snk_i.dat & snk_i.adr;
  q_in_valid <= snk_cyc_d0 or snk_cyc_d1 or snk_i.cyc;
  
  --snk_o.ack <= snk_i.cyc and snk_i.stb and snk_i.we and not src_i.stall;

p_gen_ack : process(clk_i)
  begin
    if rising_edge(clk_i) then
      snk_o.ack <= snk_i.cyc and snk_i.stb and snk_i.we and not src_i.stall;
    end if;
  end process;
   
p_store_size : process(clk_i)
  variable is_ptp : std_logic := '0';
  begin
    if rising_edge(clk_i) then
      if rst_n_i = '0' then
        n_words <= 0;
        qs_in_valid <= '0';
        wait_complete <= '0';
        cnt <= 0;
        is_ptp := '0';
      else
		 
		 if qs_in_valid = '1' then
            qs_in_valid <= '0';
            wait_complete <= '0';
         end if;
         
		 if sof_p1 = '1' then
		    n_words <= 0;
		    wait_complete <= '1';
		    cnt <= 0;
		    is_ptp := '0';
		 end if;
		 
		 if cnt = 7 then
		   if snk_i.dat = x"88F7" then
		     is_ptp := '1';
		   end if;
		 end if;
         
         if snk_valid = '1' and ( snk_sel_d0 = "01"  or snk_sel_d0 = "10" ) then
            n_words <= n_words + 1;
         end if;
         
         if snk_valid = '1' and snk_sel_d0 = "11" then
            n_words <= n_words + 2;
         end if;
         
         if eof_p1 = '1' then
           qs_in_valid <= '1';
           if is_ptp = '1' then -- I know this is crap but it seems to work!
			 qs_in <= std_logic_vector(to_unsigned((n_words)-14,12));
           else 
             qs_in <= std_logic_vector(to_unsigned((n_words)-10,12));
           end if;
           
         end if;
         
         if snk_valid = '1' then
           cnt <= cnt + 1;
         end if;
        
         
      end if;
      
    end if;
      
  end process;
  
p_pop : process(clk_i)
  variable LSDUsize						 : std_logic_vector(15 downto 4) := "000000101101";
  begin
    if rising_edge(clk_i) then
      if rst_n_i = '0' then
		inserting_tag <= '0';
		request_nseq <= '0';
		qs_rd <= '0';
      else
      
		q_rd <= not src_i.stall and not q_empty and not wait_complete and not inserting_tag;
	    src_o.adr <= q_out(1 downto 0);
	    src_o.dat <= q_out(17 downto 2);
	    src_o.cyc <= q_out(18);
	    src_o.stb <= q_out(19);
	    src_o.we <= q_out(20);
	    src_o.sel <= q_out(22 downto 21);
	    
	    if(q_out(23) = '1') then -- fifo's sof
		 hdr_offset(hdr_offset'left downto 1) <= (others => '0');
		 hdr_offset(0)                        <= '1';
		 inserting_tag <= '0';
		end if;
		
		if hdr_offset(1) = '1' and src_i.stall = '0' then
          qs_rd <= '1'; -- pop frame's size
          request_nseq <= '1';
        end if;
        
        if hdr_offset(2) = '1' and src_i.stall = '0' then
          qs_rd <= '0'; -- we've got frame's size
        end if;
        
        if seq_n_is_valid = '1' and request_nseq = '1' then
		  request_nseq <= '0'; -- we've got the seq number
		end if;
                
        if hdr_offset(3) = '1' then
          LSDUsize := qs_out(11 downto 0); 
        end if;

        if hdr_offset(5) = '1' then
          inserting_tag <= '1';
        end if;
        
        if hdr_offset(7) = '1' then -- insert HSR Ethertype
          src_o.dat <= x"892f";
        end if;
        if hdr_offset(8) = '1' then -- insert HSR PATH and LSDU Size
          src_o.dat <= g_hsr_path_id&LSDUsize;
          inserting_tag <= '0';
        end if;
        if hdr_offset(9) = '1' then -- insert HSR sequence number
          src_o.dat <= current_seq_n;
          request_nseq <= '0'; -- just in case
        end if;
		
		if q_out(25) = '1' and wait_complete = '0' and src_i.stall = '0' then 
		   hdr_offset <= hdr_offset(hdr_offset'left-1 downto 0) & '0';
		end if;
		
      end if;
      
    end if;
      
  end process;
  
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
	         if (request_nseq = '0') then
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
       
	--trig0(20 downto 5)	<= snk_i.dat;
	--trig0(21)				<= snk_i.cyc;
	--trig0(22)				<= snk_i.stb;
	--trig0(24 downto 23)	<= snk_i.adr;
	--trig0(25) <= snk_valid;	
	--trig0(26) <= snk_dreq; 
	--trig0(27) <= src_i.stall;
	--trig0(28) <= snk_dreq;
	--trig0(29) <= stall_req;
	
	
	--trig1(15 downto 0)	<= fab_2_wb_o.dat;
	----trig1(7 downto 0) <= std_logic_vector(to_unsigned(ack_counter,8));
	--trig1(16)				<= fab_2_wb_o.cyc;
	--trig1(17)				<= fab_2_wb_o.stb;
	--trig1(19 downto 18)	<= fab_2_wb_o.adr;
	
	--trig2(13 downto 0) <= hdr_offset;
	--trig2(21 downto 14) <= std_logic_vector(to_unsigned(sof_counter,8));
	--trig2(29 downto 22) <= std_logic_vector(to_unsigned(eof_counter,8));
	
	
	--trig3(2 downto 0) <= debug_insert;
	--trig3(3) <= seq_valid;
	--trig3(4) <= seq_n_is_valid;
	--trig3(5) <= request_nseq;
	--trig3(21 downto 6) <= current_seq_n;
	--trig3(24 downto 22) <= debug_seq;

	


end behavoural;
