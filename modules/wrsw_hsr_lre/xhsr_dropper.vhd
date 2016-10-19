-------------------------------------------------------------------------------
-- Title      : HSR Dropping Module
-- Project    : White Rabbit
-------------------------------------------------------------------------------
-- File       : xhsr_dropper.vhd
-- Author     : José López
-- Company    : University of Granada 
-- Department : Computer Architecture and Technology
-- Created    : 2016-09-16
-- Last update: 2016-09-16
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
use work.wr_fabric_pkg.all;
use work.wrsw_shared_types_pkg.all;
use work.mpm_pkg.all;
use work.endpoint_private_pkg.all;
use work.wrsw_hsr_lre_pkg.all;
use work.dropper_wbgen2_pkg.all;



entity xhsr_dropper is
  generic(
    g_ring_size : integer := 32;
    g_dat_width : integer :=16;
    g_size    : integer := 1024; -- things for the fifo
    g_with_fc : boolean := false -- things for the fifo
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
    wb_stall_o                               : out    std_logic;
    
    disc_ep0_o                               : out    std_logic_vector(31 downto 0);
    disc_ep1_o                               : out    std_logic_vector(31 downto 0);
    acc_ep0_o                                : out    std_logic_vector(31 downto 0);
    acc_ep1_o                                : out    std_logic_vector(31 downto 0)    
   
    );
end xhsr_dropper;




architecture behavioral of xhsr_dropper is
   
   component hsr_lre_dropper_regs
     port (
       rst_n_i                                  : in     std_logic;
       clk_sys_i                                : in     std_logic;
       wb_adr_i                                 : in     std_logic_vector(6 downto 0);
       wb_dat_i                                 : in     std_logic_vector(31 downto 0);
       wb_dat_o                                 : out    std_logic_vector(31 downto 0);
       wb_cyc_i                                 : in     std_logic;
       wb_sel_i                                 : in     std_logic_vector(3 downto 0);
       wb_stb_i                                 : in     std_logic;
       wb_we_i                                  : in     std_logic;
       wb_ack_o                                 : out    std_logic;
       wb_stall_o                               : out    std_logic;
   -- Ports for RAM: Dropper general RAM
       dropper_ram_addr_i                       : in     std_logic_vector(5 downto 0);
   -- Read data output
       dropper_ram_data_o                       : out    std_logic_vector(31 downto 0);
   -- Read strobe input (active high)
       dropper_ram_rd_i                         : in     std_logic;
   -- Write data input
       dropper_ram_data_i                       : in     std_logic_vector(31 downto 0);
   -- Write strobe (active high)
       dropper_ram_wr_i                         : in     std_logic;
       regs_i                                   : in     t_dropper_in_registers;
       regs_o                                   : out    t_dropper_out_registers
       
     );
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
  signal TRIG0      : std_logic_vector(31 downto 0);
  signal TRIG1      : std_logic_vector(31 downto 0);
  signal TRIG2      : std_logic_vector(31 downto 0);
  signal TRIG3      : std_logic_vector(31 downto 0);
  
  ------------------------------------------------
  -- Type declarations
  ------------------------------------------------

  type   t_eof_delayed              is array(2 downto 0) of std_logic_vector(1 downto 0);
  type   t_fifo_io_array            is array(1 downto 0) of std_logic_vector(21 downto 0);
  type   t_fifo_count               is array(1 downto 0) of std_logic_vector(f_log2_size(1520)-1 downto 0);
  type   t_seqnum_array             is array(1 downto 0) of std_logic_vector(15 downto 0);
  type   t_hdr_count_array          is array(1 downto 0) of std_logic_vector(3 downto 0);
  type   t_macaddr_array            is array(1 downto 0) of std_logic_vector(47 downto 0);
  type   t_counter                  is array(1 downto 0) of std_logic_vector(31 downto 0);
  type   t_nodeid_pair              is array(1 downto 0) of std_logic_vector(4 downto 0);
  type   t_fsm_state                is (s_IDLE, s_PARSING, s_LOOKUP, s_INSERT, s_UPDATE, s_DROP, s_ACCEPT);
  type   t_fsm_state_array          is array(1 downto 0) of t_fsm_state;
  type   t_wbmem_state              is (s_IDLE, s_INSERT, s_UPDATE);
  type   t_available_pos_array      is array(1 downto 0) of std_logic_vector(g_ring_size-1 downto 0);
  type   t_integer_pair             is array(1 downto 0) of integer;
  type   t_wb_addr_pair_int         is array(1 downto 0) of std_logic_vector(5 downto 0);

  -------------------------------------------------
  -- FIFO input-related signals
  -------------------------------------------------
  
  signal sof, eof                   : std_logic_vector(1 downto 0);
  signal eof_d                      : t_eof_delayed;  

  signal fifo_rd, fifo_rd_d0        : std_logic_vector(1 downto 0);
  signal fifo_wr                    : std_logic_vector(1 downto 0);
  signal fifo_full                  : std_logic_vector(1 downto 0);
  signal snk_valid                  : std_logic_vector(1 downto 0);
  
  signal fifo_empty                 : std_logic_vector(1 downto 0);
  signal fifo_empty_delay0          : std_logic_vector(3 downto 0);
  signal fifo_empty_delay1          : std_logic_vector(3 downto 0);
  
  signal fifo_in,fifo_out           : t_fifo_io_array;  
  signal delay_snk_in0              : t_wrf_sink_in_array(4 downto 0);
  signal delay_snk_in1              : t_wrf_sink_in_array(4 downto 0);  
  
  -------------------------------------------------
  -- FSMs internal signals
  -------------------------------------------------

  signal available_pos_int          : t_available_pos_array;
  signal available_pos              : std_logic_vector(g_ring_size-1 downto 0) := (others => '0');
  signal first_available            : t_integer_pair;
  
  signal state                      : t_fsm_state_array;
  signal state_wbmem0, state_wbmem1 : t_wbmem_state;  

  signal rd_grant                   : std_logic_vector(1 downto 0);
  
  signal hdr_count                  : t_hdr_count_array;
  signal hdr_done                   : std_logic_vector(1 downto 0);
  signal is_hsr                     : std_logic_vector(1 downto 0);
  
  signal check_dmac, check_smac     : t_integer_pair;
  signal check_smac_req             : std_logic_vector(1 downto 0);

  signal write_node                 : std_logic_vector(1 downto 0);
  signal nodeid_wr                  : t_nodeid_pair;
  signal nodeid_rd                  : t_nodeid_pair;
  signal curr_nodeid                : t_nodeid_pair;
  signal seqnum_rd                  : t_seqnum_array;

  signal mem_valid                  : std_logic_vector(1 downto 0);
  signal mem_busy                   : std_logic;
  signal wr_update_sig              : std_logic_vector(1 downto 0);
  signal wr_insert_sig              : std_logic_vector(1 downto 0);

  signal delete_index               : integer;
  signal delete_cmd                 : std_logic;
  signal reset_mem                  : std_logic;  
 
  --Wb slave memory signals:
  signal wb_mem_addr                : std_logic_vector(5 downto 0);
  signal wb_mem_data_out            : std_logic_vector(31 downto 0);
  signal wb_mem_data_in             : std_logic_vector(31 downto 0);  
  signal wb_mem_rd                  : std_logic;
  signal wb_mem_wr                  : std_logic;
  signal wb_mem_busy                : std_logic_vector(1 downto 0);
  signal wb_mem_wr_int              : std_logic_vector(1 downto 0);
  signal wb_mem_data_in_int         : t_counter;
  signal wb_mem_addr_int            : t_wb_addr_pair_int;
  signal regs_fromwb                : t_dropper_out_registers;
  signal word_count                 : std_logic_vector(1 downto 0);
  
  ----
  
  -------------------------------------------------
  -- Ethernet header fields
  -------------------------------------------------

  signal curr_seqnum                : t_seqnum_array;  
  signal src_mac, dst_mac           : t_macaddr_array;
  
  -------------------------------------------------
  -- Output-related signals
  -------------------------------------------------

  signal src_out                    : t_wrf_source_out_array(1 downto 0);
  signal regs_towb                  : t_dropper_in_registers;  
  
  -------------------------------------------------
  -- Counters, debug, diagnose
  -------------------------------------------------

  signal accept_count, drop_count   : t_counter;
  signal match                      : std_logic_vector(1 downto 0);
  signal accept, drop               : std_logic_vector(1 downto 0);

  signal senaldebug                 : t_seqnum_array;
  signal senaldebug_mem             : std_logic_vector(7 downto 0);
  
  --------------------------------------------------
  -- Functions
  --------------------------------------------------
  
  function f_hot_to_bin(x : std_logic_vector(g_ring_size-1 downto 0))
   return integer is
   variable rv : integer := 32;
   begin
      for i in g_ring_size-1 downto 0 loop
         if x(i) = '0' then
            rv := i;
         end if;
      end loop;
   return rv;
  end function;
   
   function f_check_dmac(dmac, my_mac : std_logic_vector(47 downto 0))
    return integer is
    variable rv : integer;
    begin
      if dmac = my_mac then             -- Unicast
         rv := 1;
      elsif dmac = x"011B19000000" then -- PTP Eth. multicast address
         rv := 2;
      elsif dmac = x"0180C200000E" then -- PTP peer-delay multicast address
         rv := 3;
      elsif dmac = x"01154E000100" then -- HSR supervision multicast address
         rv := 4;
      elsif dmac(40) = '1' then         -- Generic multicast/broadcast address
         rv := 5;      
      else
         rv := 0;
      end if;
      return rv;         
    end function;
   
  
  begin -- behavioral
   
   U_dropper_mem : xhsr_dropper_mem
   port map(
   rst_n_i      => rst_n_i,
   clk_i        => clk_i,
   smac_a_i     => src_mac(0),
   smac_b_i     => src_mac(1),
   seqnum_a_i   => curr_seqnum(0),
   seqnum_b_i   => curr_seqnum(1),
   rd_sig_a_i   => (check_smac_req(0) and (not mem_busy)),
   rd_sig_b_i   => (check_smac_req(1) and (not mem_busy)),
   wr_sig_a_i   => write_node(0),
   wr_sig_b_i   => write_node(1),
   nodeid_a_i   => nodeid_wr(0),
   nodeid_b_i   => nodeid_wr(1),
   nodeid_a_o   => nodeid_rd(0),
   nodeid_b_o   => nodeid_rd(1),
   seqnum_a_o   => seqnum_rd(0),
   seqnum_b_o   => seqnum_rd(1),
   valid_a_o    => mem_valid(0),
   valid_b_o    => mem_valid(1),
   match_a_o    => match(0),
   match_b_o    => match(1),
   mem_busy_o   => mem_busy,
   senaldebug_o => open--senaldebug_mem
   );
   
   U_wb_mem : hsr_lre_dropper_regs
   port map(
    rst_n_i       => rst_n_i,
    clk_sys_i     => clk_i,
    wb_adr_i      => wb_adr_i,
    wb_dat_i      => wb_dat_i,
    wb_dat_o      => wb_dat_o,
    wb_cyc_i      => wb_cyc_i,
    wb_sel_i      => wb_sel_i,
    wb_stb_i      => wb_stb_i,
    wb_we_i       => wb_we_i,
    wb_ack_o      => wb_ack_o,
    wb_stall_o    => wb_stall_o,
    
    dropper_ram_addr_i => wb_mem_addr,       -- slv 5 downto 0
    dropper_ram_data_o => wb_mem_data_out, -- slv 31 downto 0
    dropper_ram_rd_i   => wb_mem_rd,
    dropper_ram_data_i => wb_mem_data_in,  -- slv 31 downto 0
    dropper_ram_wr_i   => wb_mem_wr,
    
    regs_i             => regs_towb,       -- t_dropper_in_registers;
    regs_o             => regs_fromwb        -- t_dropper_out_registers
  );
  
  regs_towb.avail_i    <= available_pos;
  delete_index         <= to_integer(unsigned(regs_fromwb.del_index_o));
  delete_cmd           <= regs_fromwb.del_cmd_o;
  reset_mem            <= regs_fromwb.del_rst_o;
   

  p_delay : process(clk_i)
  begin
   if rising_edge(clk_i) then

      delay_snk_in0     <= delay_snk_in0(delay_snk_in0'high-1 downto 0) & snk_i(0);

      delay_snk_in1     <= delay_snk_in1(delay_snk_in1'high-1 downto 0) & snk_i(1);

      fifo_empty_delay0 <= fifo_empty_delay0(fifo_empty_delay0'high-1 downto 0) & fifo_empty(0);

      fifo_empty_delay1 <= fifo_empty_delay1(fifo_empty_delay1'high-1 downto 0) & fifo_empty(1);
      
      eof_d             <= eof_d(eof_d'high-1 downto 0) & eof;
      
   end if;  
  end process;

  sof <= (not (delay_snk_in1(0).cyc & delay_snk_in0(0).cyc)) and (snk_i(1).cyc & snk_i(0).cyc);
  eof <= (delay_snk_in1(0).cyc & delay_snk_in0(0).cyc) and not (snk_i(1).cyc & snk_i(0).cyc);

  snk_valid(0) <= (delay_snk_in0(0).cyc and delay_snk_in0(0).stb and delay_snk_in0(0).we);-- and not src_i(0).stall;
  snk_valid(1) <= (delay_snk_in1(0).cyc and delay_snk_in1(0).stb and delay_snk_in1(0).we); -- and not src_i(1).stall;
 
  gen_fifo : for i in 1 downto 0 generate
     BUF_FIFO : generic_sync_fifo
       generic map (
         g_data_width              => 22,
         g_size                    => 1520,--g_size,
         g_with_almost_empty       => false,
         g_with_almost_full        => false,
         g_almost_empty_threshold  => 20,
         g_almost_full_threshold   => 1400,
         g_with_count              => true)
       port map (
         rst_n_i        => rst_n_i,
         clk_i          => clk_i,
         d_i            => fifo_in(i),
         we_i           => fifo_wr(i),
         q_o            => fifo_out(i),
         rd_i           => fifo_rd(i),
         empty_o        => fifo_empty(i),
         full_o         => fifo_full(i),
         almost_empty_o => open,
         almost_full_o  => open,
         count_o        => open);
   end generate gen_fifo;


   fifo_wr(1) <= delay_snk_in1(2).cyc or delay_snk_in1(1).cyc or delay_snk_in1(0).cyc or snk_i(1).cyc;
   fifo_wr(0) <= delay_snk_in0(2).cyc or delay_snk_in0(1).cyc or delay_snk_in0(0).cyc or snk_i(0).cyc;
   
   fifo_rd(1) <= (not fifo_empty_delay1(2)) and (not src_i(1).stall) and rd_grant(1);
   fifo_rd(0) <= (not fifo_empty_delay0(2)) and (not src_i(0).stall) and rd_grant(0);
   
   fifo_in(1) <= delay_snk_in1(0).cyc & delay_snk_in1(0).stb & delay_snk_in1(0).adr & delay_snk_in1(0).sel & delay_snk_in1(0).dat;
   fifo_in(0) <= delay_snk_in0(0).cyc & delay_snk_in0(0).stb & delay_snk_in0(0).adr & delay_snk_in0(0).sel & delay_snk_in0(0).dat;
   

   gen_p_header : for i in 1 downto 0 generate

      p_header : process(clk_i)
      
      begin
      
         if(rst_n_i = '0') then
            accept_count(i)      <= (others => '0');
            drop_count(i)        <= (others => '0');
            state(i)             <= s_IDLE;
            hdr_done(i)          <= '0';
            hdr_count(i)         <= (others => '0');
            src_mac(i)           <= (others => '0');
            dst_mac(i)           <= (others => '0');
            senaldebug(i)        <= x"0001";
            available_pos_int(i) <=  (others => '0');
                     
         elsif(rising_edge(clk_i)) then
         
            if(delete_cmd = '1') then
               available_pos_int(i)(delete_index) <= '0';
            end if;
            
            if(reset_mem = '1') then
               available_pos_int(i) <= (others => '0');
            end if;

         
            case state(i) is
         
               when s_IDLE =>
                  
                  check_smac_req(i) <= '0';
                  hdr_done(i)       <= '0';
                  hdr_count(i)      <= (others => '0');

                  wr_update_sig(i)  <= '0';
                  wr_insert_sig(i)  <= '0';
                  
                  drop(i)           <= '0';
                  accept(i)         <= '0';
                  
                  senaldebug(i)     <= x"0002";
                                    
                  if sof(i) = '1' then
                     state(i)       <= s_PARSING;
                     senaldebug(i)  <= x"0003";
                  end if;
                  
               when s_PARSING =>
               
                  if snk_valid(i) = '1' and snk_i(i).adr = c_WRF_DATA then
                     hdr_count(i) <= std_logic_vector(unsigned(hdr_count(i)) + 1);            
                     case hdr_count(i) is
                        when x"0" =>
                              dst_mac(i)(47 downto 32) <= snk_i(i).dat;
                              is_hsr(i)                <= '0';
                              senaldebug(i)            <= x"0004";
                        when x"1" =>
                              dst_mac(i)(31 downto 16) <= snk_i(i).dat;
                              senaldebug(i)            <= x"0005";
                        when x"2" =>
                              dst_mac(i)(15 downto 0)  <= snk_i(i).dat;
                              senaldebug(i)            <= x"0006";
                        when x"3" =>
                              check_dmac(i)            <= f_check_dmac(dst_mac(i),mac_addr_i);
                              src_mac(i)(47 downto 32) <= snk_i(i).dat;
                              senaldebug(i)            <= x"0007";
                        when x"4" =>
                              
                              src_mac(i)(31 downto 16) <= snk_i(i).dat;
                              senaldebug(i)            <= x"0008";
                              
                        when x"5" =>
                              src_mac(i)(15 downto 0)  <= snk_i(i).dat;
                              senaldebug(i)            <= x"0009";
                        when x"6" =>
                              if snk_i(i).dat = x"892f" then
                                 is_hsr(i)             <= '1';
                                 senaldebug(i)         <= x"000A";
                              else
                                 is_hsr(i)             <= '0';
                                 senaldebug(i)         <= x"000B";
                              end if;
                                                            
                        when x"8" =>
                              if(is_hsr(i) = '1') then
                               curr_seqnum(i)          <= snk_i(i).dat;
                               senaldebug(i)           <= x"000C";
                              end if;
                              
                              hdr_done(i)              <= '1';
                              if(is_hsr(i) = '1') then
                                 state(i)                 <= s_LOOKUP;
                              else
                                 state(i)                 <= s_DROP;
                              end if;
                              
                              -- Differential treatment to PTP frames:
--                              if (check_dmac(i) = 2) or (check_dmac(i) = 3) then
--                                 state(i) <= s_ACCEPT;
--                                 senaldebug(i)     <= x"000D";
--                              end if;
                        
                        when others =>
                              
                     end case;
                  end if;
               
               when s_LOOKUP => 
                  
                  senaldebug(i)            <= x"000E";
                  if(mem_busy = '0') then
                     check_smac_req(i)     <= '1';
                     senaldebug(i)         <= x"000F";
                  else
                     check_smac_req(i)     <= '0';
                  end if;
                  
                  if check_smac_req(i) = '1' then
                     check_smac_req(i)     <= '0';
                     senaldebug(i)         <= x"0020";
                  end if;
                  
                  if mem_valid(i) = '1' then
                     check_smac_req(i)     <= '0';
                     if match(i) = '1' and available_pos(to_integer(unsigned(nodeid_rd(i)))) = '1' then

                        curr_nodeid(i)     <= nodeid_rd(i);
                        senaldebug(i)      <= x"0010";
                     
                        if unsigned(curr_seqnum(i)) = unsigned(seqnum_rd(i)) then
                        
                           state(i)        <= s_DROP;
                           senaldebug(i)   <= x"0011";
                           drop(i)         <= '1';
                           
                        elsif unsigned(curr_seqnum(i)) = (unsigned(seqnum_rd(i))+1) then
                        
                           state(i)        <= s_UPDATE;
                           senaldebug(i)   <= x"0012";
                        
                        elsif unsigned(curr_seqnum(i)) > (unsigned(seqnum_rd(i))+1) then

                           state(i)        <= s_UPDATE;
                           senaldebug(i)   <= x"0013";
                           -- TODO: ISSUE ALERT
                        
                        else
                        
                           drop(i)         <= '1';
                           state(i)        <= s_DROP;
                           senaldebug(i)   <= x"0014";
                           -- TODO: ISSUE ALERT
                        end if;
                           
                     elsif f_hot_to_bin(available_pos) /= 32 then
                        
                        state(i)           <= s_INSERT;
                        senaldebug(i)      <= x"0015";
                        first_available(i) <= f_hot_to_bin(available_pos);
                        
                     else
                     
                        state(i)           <= s_DROP;
                        senaldebug(i)      <= x"0016";
                        -- TODO: ISSUE ALERT
                     
                     end if;
                        
                  end if;
                  
                  
               when s_UPDATE =>

                  if mem_busy = '0' then
                  
                     write_node(i)          <= '1';
                     nodeid_wr(i)           <= curr_nodeid(i);
                     senaldebug(i)          <= x"0017";
                     accept_count(i)        <= std_logic_vector(unsigned(accept_count(i))+1);
                     
                  else
                  
                     write_node(i)          <= '0';
                     senaldebug(i)          <= x"0021";
                  
                  end if;                  
                  
                  if mem_valid(i) = '1' then
                     accept(i)              <= '1';
                     wr_update_sig(i)       <= '1';
                     write_node(i)          <= '0';
                     state(i)               <= s_IDLE;
                     senaldebug(i)          <= x"0018";
                  end if;

               when s_INSERT =>
                  
                  senaldebug(i)             <= x"0019";
                  
                  if(first_available(i) = 32) then
                     -- TODO: ISSUE ALERT
                     state(i)               <= s_DROP;
                     drop(i)                <= '1';
                     senaldebug(i)          <= x"001A";
                  else
                     
                     senaldebug(i)          <= x"001B";
                     nodeid_wr(i)           <= std_logic_vector(to_unsigned(first_available(i),5));
                     
                     if mem_busy = '0' then
                        write_node(i)       <= '1';
                        accept_count(i)     <= std_logic_vector(unsigned(accept_count(i))+1);
                     else
                        write_node(i)       <= '0';
                     end if;
                     
                     wb_mem_wr_int(i)       <= '1';
                  
                     if mem_valid(i) = '1' then
                        available_pos_int(i)(first_available(i)) <= '1';
                        write_node(i)       <= '0';
                        accept(i)           <= '1';
                        state(i)            <= s_ACCEPT;
                        senaldebug(i)       <= x"0023";
                        wr_insert_sig(i)    <= '1';
                     end if;
                     
                     
                  end if;
               
               when s_ACCEPT =>
                  
                  state(i)                  <= s_IDLE;
                  senaldebug(i)             <= x"001C";
                  if eof_d(i)(0) = '1' then
                     state(i)               <= s_IDLE;
                     senaldebug(i)          <= x"001D";
                  end if;
               
               when s_DROP =>
                  
                  drop_count(i)        <= std_logic_vector(unsigned(drop_count(i))+1);
                  state(i)                  <= s_IDLE;
                  senaldebug(i)             <= x"001E";
                  -- Previously if EOF:
                  if eof_d(i)(0) = '1' then
                     state(i)               <= s_IDLE;
                     senaldebug(i)          <= x"001F";
                  end if;
                  
               when others =>
         
            end case;
            
         end if;
      
      end process;
   
   
      p_fwd_drop : process(clk_i)
      begin
         if rst_n_i = '0' then
         
            rd_grant(i)          <= '0';
            fifo_rd_d0(i)        <= '0';
         
         elsif rising_edge(clk_i) then
         
            fifo_rd_d0(i)        <= fifo_rd(i);
         
            case state(i) is
            
               when s_IDLE =>
               
                  if(rd_grant(i) = '1' and fifo_empty(i) = '0') then
                     rd_grant(i) <= '1';
                  else
                     rd_grant(i) <= '0';
                  end if;
               
               when s_PARSING =>
                  
                  rd_grant(i)    <= '0';
               
               when s_LOOKUP =>
                  
                  rd_grant(i)    <= '0';
                  
               when s_INSERT =>
               
                  rd_grant(i)    <= '0';
                  
               when s_UPDATE =>
                  
                  rd_grant(i)    <= '1';
               
               when s_ACCEPT =>
               
                  rd_grant(i)    <= '1';
               
               when s_DROP =>
               
                  rd_grant(i)    <= '1';
               
               when others =>
               
            end case;
         
         end if;
      end process;
      


      U_ack_gen : process(clk_i)
      begin
      if rst_n_i = '0' then
      
      elsif rising_edge(clk_i) then
         if fifo_wr(i) = '1' then
            snk_o(i).ack         <= '1';
         else
            snk_o(i).ack         <= '0';
         end if;
      end if;
      end process;

      src_out(i).cyc      <= fifo_out(i)(21); --when (fifo_rd_d0(i) = '1' and (state(i) /= s_DROP)) else '0';
      src_out(i).stb      <= fifo_out(i)(20); -- when (fifo_rd_d0(i) = '1' and (state(i) /= s_DROP)) else '0';
      src_out(i).adr      <= fifo_out(i)(19 downto 18); -- when (fifo_rd_d0(i) = '1' and (state(i) /= s_DROP)) else (others => '0');
      src_out(i).sel      <= fifo_out(i)(17 downto 16); -- when (fifo_rd_d0(i) = '1' and (state(i) /= s_DROP)) else (others => '0');
      src_out(i).dat      <= fifo_out(i)(15 downto 0); -- when (fifo_rd_d0(i) = '1' and (state(i) /= s_DROP)) else (others => '0');
      src_out(i).we       <= '1';
      
   end generate gen_p_header;
   
   available_pos          <= available_pos_int(0) or available_pos_int(1);
   acc_ep0_o              <= accept_count(0);
   acc_ep1_o              <= accept_count(1);
   disc_ep0_o             <= drop_count(0);
   disc_ep1_o             <= drop_count(1);

   
   p_wb_mem_input2 : process(clk_i)
   begin
   if rst_n_i = '0' then
      wb_mem_busy         <= "00";
      state_wbmem0        <= s_IDLE;
      state_wbmem1        <= s_IDLE;
   elsif rising_edge(clk_i) then
      wb_mem_wr           <= '0';
      --senaldebug_mem      <= x"01";
      
      case state_wbmem0 is
         when s_IDLE =>
            if wr_update_sig(0) = '1' then
               state_wbmem0      <= s_UPDATE;
            elsif
               wr_insert_sig(0) = '1' then
               state_wbmem0      <= s_INSERT;
            end if;
         when s_UPDATE =>
            if wb_mem_busy(1) = '0' then
               wb_mem_busy(0)    <= '1';
               --if state(0) = s_UPDATE then
               if word_count = "00" then
                     wb_mem_wr          <= '1';
                     wb_mem_addr        <= curr_nodeid(0) & "0";
                     wb_mem_data_in     <= src_mac(0)(47 downto 16);
                     word_count         <= "01";
--                     senaldebug_mem     <= x"02";
               elsif word_count = "01" then
                     wb_mem_wr          <= '1';
                     wb_mem_addr        <= std_logic_vector(unsigned(wb_mem_addr)+1);
                     wb_mem_data_in     <= src_mac(0)(15 downto 0) & curr_seqnum(0);
                     word_count         <= "00";
                     wb_mem_busy(0)     <= '0';
--                     senaldebug_mem     <= x"03";
                     state_wbmem0       <= s_IDLE;
               end if;
            end if;
         when s_INSERT =>
            if wb_mem_busy(1) = '0' then
               wb_mem_busy(0)           <= '1';
               if word_count = "00" then
                  wb_mem_wr             <= '1';
                  wb_mem_addr           <= std_logic_vector(to_unsigned(first_available(0),5)) & "0";
                  wb_mem_data_in        <= src_mac(0)(47 downto 16);
                  word_count            <= "01";
--                  senaldebug_mem        <= x"04";
               elsif word_count = "01" then
                  wb_mem_wr             <= '1';
                  wb_mem_addr           <= std_logic_vector(unsigned(wb_mem_addr)+1);
                  wb_mem_data_in        <= src_mac(0)(15 downto 0) & curr_seqnum(0);
                  word_count            <= "00";
                  wb_mem_busy(0)        <= '0';
--                  senaldebug_mem        <= x"05";
                  state_wbmem0          <= s_IDLE;
               end if;
            end if;
         when others =>
         
      end case;
      
      case state_wbmem1 is
         when s_IDLE =>
            if wr_update_sig(1) = '1' then
               state_wbmem1             <= s_UPDATE;
            elsif
               wr_insert_sig(1) = '1' then
               state_wbmem1             <= s_INSERT;
            end if;
         when s_UPDATE =>
            if wb_mem_busy(0) = '0' then
               wb_mem_busy(1)           <= '1';
               --if state(0) = s_UPDATE then
               if word_count = "00" then
                     wb_mem_wr          <= '1';
                     wb_mem_addr        <= curr_nodeid(1) & "0";
                     wb_mem_data_in     <= src_mac(1)(47 downto 16);
                     word_count         <= "01";
--                     senaldebug_mem     <= x"02";
               elsif word_count = "01" then
                     wb_mem_wr          <= '1';
                     wb_mem_addr        <= std_logic_vector(unsigned(wb_mem_addr)+1);
                     wb_mem_data_in     <= src_mac(1)(15 downto 0) & curr_seqnum(1);
                     word_count         <= "00";
                     wb_mem_busy(1)     <= '0';
--                     senaldebug_mem     <= x"03";
                     state_wbmem1       <= s_IDLE;
               end if;
            end if;
         when s_INSERT =>
            if wb_mem_busy(0) = '0' then
               wb_mem_busy(1)           <= '1';
               if word_count = "00" then
                  wb_mem_wr             <= '1';
                  wb_mem_addr           <= std_logic_vector(to_unsigned(first_available(1),5)) & "0";
                  wb_mem_data_in        <= src_mac(1)(47 downto 16);
                  word_count            <= "01";
--                  senaldebug_mem        <= x"04";
               elsif word_count = "01" then
                  wb_mem_wr             <= '1';
                  wb_mem_addr           <= std_logic_vector(unsigned(wb_mem_addr)+1);
                  wb_mem_data_in        <= src_mac(1)(15 downto 0) & curr_seqnum(1);
                  word_count            <= "00";
                  wb_mem_busy(1)        <= '0';
--                  senaldebug_mem        <= x"05";
                  state_wbmem1          <= s_IDLE;
               end if;
            end if;
         when others =>
         
      end case;
      
   end if;
   end process;   
   

   
   src_o <= src_out;
--  src_o <= snk_i;
--  snk_o <= src_i;
   
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
   
   
--   trig0(0) <= src_out(0).cyc;
--   trig0(1) <= src_out(0).stb;
--   trig0(3 downto 2) <= src_out(0).adr;
--   trig0(19 downto 4) <= src_out(0).dat;
--   trig0(20) <= src_i(0).ack;
--   trig0(21) <= src_i(0).stall;
--   
--   trig1(0) <= fifo_full(0);
--   trig1(2) <= fifo_empty(0);
--   trig1(3) <= fifo_wr(0);
--   trig1(4) <= fifo_rd(0);
--   trig1(12 downto 5) <= senaldebug(0)(7 downto 0);
--   trig1(20 downto 13) <= senaldebug(1)(7 downto 0);
--   
--   trig2(0) <= snk_i(0).cyc;
--   trig2(1) <= snk_i(0).stb;
--   trig2(3 downto 2) <= snk_i(0).adr;
--   trig2(19 downto 4) <= snk_i(0).dat;
--   --trig2(20) <= src_i(0).ack;
--   --trig2(21) <= src_i(0).stall;
--   trig3(1 downto 0) <= sof;
--   trig3(3 downto 2) <= snk_valid;
--   trig3(7 downto 4) <= hdr_count(0);
--   trig3(9 downto 8) <= is_hsr;
--   trig3(13 downto 10) <= std_logic_vector(to_unsigned(check_dmac(0),4));
--   trig3(21 downto 14) <= std_logic_vector(to_unsigned(check_smac(0),8));
--   trig3(29 downto 22) <= std_logic_vector(to_unsigned(first_available(0),8));
   
   trig0(0) <= src_out(0).cyc;
   trig0(1) <= src_out(0).stb;
   trig0(3 downto 2) <= src_out(0).adr;
   trig0(19 downto 4) <= src_out(0).dat;
   trig0(20) <= src_i(0).ack;
   trig0(21) <= src_i(0).stall;
   trig0(29 downto 22) <= senaldebug(0)(7 downto 0);
   
   trig1(23 downto 0) <= available_pos(23 downto 0);
   trig1(25 downto 24) <= wr_update_sig;
   trig1(27 downto 26) <= wr_insert_sig;
   trig1(29 downto 28) <= wb_mem_busy;
   
   trig2(0) <= write_node(0);
   trig2(1) <= write_node(1);
   trig2(2) <= check_smac_req(0);
   trig2(3) <= check_smac_req(1);
   trig2(4) <= mem_valid(0);
   trig2(5) <= mem_valid(1);
   trig2(6) <= match(0);
   trig2(7) <= match(1);
   trig2(8) <= mem_busy;
   trig2(16 downto 9) <= std_logic_vector(to_unsigned(first_available(0),8));
   
   trig2(22 downto 18) <= curr_nodeid(0)(4 downto 0);
   
   trig3(4 downto 0) <= nodeid_wr(0);
   trig3(9 downto 5) <= nodeid_rd(0);
   
   trig3(17 downto 10) <= senaldebug_mem;
   trig3(31 downto 18) <= seqnum_rd(0)(13 downto 0);

--   trig0(0) <= src_out(0).cyc;
--   trig0(1) <= src_out(0).stb;
--   trig0(3 downto 2) <= src_out(0).adr;
--   trig0(19 downto 4) <= src_out(0).dat;
--   trig0(20) <= src_i(0).ack;
--   trig0(21) <= src_i(0).stall;
--   trig0(29 downto 22) <= senaldebug(0)(7 downto 0);
--   
--   trig1 <= available_pos;
--   
--   trig2(1 downto 0) <= wr_update_sig;
--   trig2(3 downto 2) <= wr_insert_sig;
--   trig2(5 downto 4) <= wb_mem_busy;
--   
--   
--   trig2(19 downto 6) <= curr_seqnum(0)(13 downto 0);
--   trig2(25 downto 20) <= wb_mem_addr;
--   trig2(27) <= wb_mem_wr;
--   trig2(29 downto 28) <= drop;
--   trig2(31 downto 30) <= accept;
--   
--   
--   
--   trig3(4 downto 0) <= nodeid_rd(0);
--   trig3(12 downto 5) <= senaldebug_mem;
--   trig3(31 downto 13) <= wb_mem_data_in(18 downto 0);
end behavioral;
