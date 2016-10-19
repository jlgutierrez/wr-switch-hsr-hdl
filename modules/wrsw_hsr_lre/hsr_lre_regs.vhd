---------------------------------------------------------------------------------------
-- Title          : Wishbone slave core for WR Switch HSR LRE registers
---------------------------------------------------------------------------------------
-- File           : hsr_lre_regs.vhd
-- Author         : auto-generated by wbgen2 from hsr_lre_regs.wb
-- Created        : Tue Oct 11 09:44:37 2016
-- Standard       : VHDL'87
---------------------------------------------------------------------------------------
-- THIS FILE WAS GENERATED BY wbgen2 FROM SOURCE FILE hsr_lre_regs.wb
-- DO NOT HAND-EDIT UNLESS IT'S ABSOLUTELY NECESSARY!
---------------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.lre_wbgen2_pkg.all;


entity hsr_lre_regs is
  port (
    rst_n_i                                  : in     std_logic;
    clk_sys_i                                : in     std_logic;
    wb_adr_i                                 : in     std_logic_vector(3 downto 0);
    wb_dat_i                                 : in     std_logic_vector(31 downto 0);
    wb_dat_o                                 : out    std_logic_vector(31 downto 0);
    wb_cyc_i                                 : in     std_logic;
    wb_sel_i                                 : in     std_logic_vector(3 downto 0);
    wb_stb_i                                 : in     std_logic;
    wb_we_i                                  : in     std_logic;
    wb_ack_o                                 : out    std_logic;
    wb_stall_o                               : out    std_logic;
    regs_i                                   : in     t_lre_in_registers;
    regs_o                                   : out    t_lre_out_registers
  );
end hsr_lre_regs;

architecture syn of hsr_lre_regs is

signal lre_lcr_rst_n_dly0                       : std_logic      ;
signal lre_lcr_rst_n_int                        : std_logic      ;
signal lre_lcr_en_int                           : std_logic      ;
signal lre_lcr_mode_int                         : std_logic_vector(3 downto 0);
signal lre_lcr_hsr_path_int                     : std_logic_vector(3 downto 0);
signal lre_lcr_clr_cnt_dly0                     : std_logic      ;
signal lre_lcr_clr_cnt_int                      : std_logic      ;
signal lre_mach_int                             : std_logic_vector(15 downto 0);
signal lre_macl_int                             : std_logic_vector(31 downto 0);
signal ack_sreg                                 : std_logic_vector(9 downto 0);
signal rddata_reg                               : std_logic_vector(31 downto 0);
signal wrdata_reg                               : std_logic_vector(31 downto 0);
signal bwsel_reg                                : std_logic_vector(3 downto 0);
signal rwaddr_reg                               : std_logic_vector(3 downto 0);
signal ack_in_progress                          : std_logic      ;
signal wr_int                                   : std_logic      ;
signal rd_int                                   : std_logic      ;
signal allones                                  : std_logic_vector(31 downto 0);
signal allzeros                                 : std_logic_vector(31 downto 0);

begin
-- Some internal signals assignments. For (foreseen) compatibility with other bus standards.
  wrdata_reg <= wb_dat_i;
  bwsel_reg <= wb_sel_i;
  rd_int <= wb_cyc_i and (wb_stb_i and (not wb_we_i));
  wr_int <= wb_cyc_i and (wb_stb_i and wb_we_i);
  allones <= (others => '1');
  allzeros <= (others => '0');
-- 
-- Main register bank access process.
  process (clk_sys_i, rst_n_i)
  begin
    if (rst_n_i = '0') then 
      ack_sreg <= "0000000000";
      ack_in_progress <= '0';
      rddata_reg <= "00000000000000000000000000000000";
      lre_lcr_rst_n_int <= '0';
      lre_lcr_en_int <= '0';
      lre_lcr_mode_int <= "0000";
      lre_lcr_hsr_path_int <= "0000";
      lre_lcr_clr_cnt_int <= '0';
      lre_mach_int <= "0000000000000000";
      lre_macl_int <= "00000000000000000000000000000000";
    elsif rising_edge(clk_sys_i) then
-- advance the ACK generator shift register
      ack_sreg(8 downto 0) <= ack_sreg(9 downto 1);
      ack_sreg(9) <= '0';
      if (ack_in_progress = '1') then
        if (ack_sreg(0) = '1') then
          lre_lcr_rst_n_int <= '0';
          lre_lcr_clr_cnt_int <= '0';
          ack_in_progress <= '0';
        else
        end if;
      else
        if ((wb_cyc_i = '1') and (wb_stb_i = '1')) then
          case rwaddr_reg(3 downto 0) is
          when "0000" => 
            if (wb_we_i = '1') then
              lre_lcr_rst_n_int <= wrdata_reg(0);
              lre_lcr_en_int <= wrdata_reg(1);
              lre_lcr_mode_int <= wrdata_reg(5 downto 2);
              lre_lcr_hsr_path_int <= wrdata_reg(9 downto 6);
              lre_lcr_clr_cnt_int <= wrdata_reg(12);
            end if;
            rddata_reg(0) <= '0';
            rddata_reg(1) <= lre_lcr_en_int;
            rddata_reg(5 downto 2) <= lre_lcr_mode_int;
            rddata_reg(9 downto 6) <= lre_lcr_hsr_path_int;
            rddata_reg(11 downto 10) <= regs_i.lcr_link_ok_i;
            rddata_reg(12) <= '0';
            rddata_reg(13) <= 'X';
            rddata_reg(14) <= 'X';
            rddata_reg(15) <= 'X';
            rddata_reg(16) <= 'X';
            rddata_reg(17) <= 'X';
            rddata_reg(18) <= 'X';
            rddata_reg(19) <= 'X';
            rddata_reg(20) <= 'X';
            rddata_reg(21) <= 'X';
            rddata_reg(22) <= 'X';
            rddata_reg(23) <= 'X';
            rddata_reg(24) <= 'X';
            rddata_reg(25) <= 'X';
            rddata_reg(26) <= 'X';
            rddata_reg(27) <= 'X';
            rddata_reg(28) <= 'X';
            rddata_reg(29) <= 'X';
            rddata_reg(30) <= 'X';
            rddata_reg(31) <= 'X';
            ack_sreg(2) <= '1';
            ack_in_progress <= '1';
          when "0001" => 
            if (wb_we_i = '1') then
              lre_mach_int <= wrdata_reg(15 downto 0);
            end if;
            rddata_reg(15 downto 0) <= lre_mach_int;
            rddata_reg(16) <= 'X';
            rddata_reg(17) <= 'X';
            rddata_reg(18) <= 'X';
            rddata_reg(19) <= 'X';
            rddata_reg(20) <= 'X';
            rddata_reg(21) <= 'X';
            rddata_reg(22) <= 'X';
            rddata_reg(23) <= 'X';
            rddata_reg(24) <= 'X';
            rddata_reg(25) <= 'X';
            rddata_reg(26) <= 'X';
            rddata_reg(27) <= 'X';
            rddata_reg(28) <= 'X';
            rddata_reg(29) <= 'X';
            rddata_reg(30) <= 'X';
            rddata_reg(31) <= 'X';
            ack_sreg(0) <= '1';
            ack_in_progress <= '1';
          when "0010" => 
            if (wb_we_i = '1') then
              lre_macl_int <= wrdata_reg(31 downto 0);
            end if;
            rddata_reg(31 downto 0) <= lre_macl_int;
            ack_sreg(0) <= '1';
            ack_in_progress <= '1';
          when "0011" => 
            if (wb_we_i = '1') then
            end if;
            rddata_reg(31 downto 0) <= regs_i.fwd_ep0_i;
            ack_sreg(0) <= '1';
            ack_in_progress <= '1';
          when "0100" => 
            if (wb_we_i = '1') then
            end if;
            rddata_reg(31 downto 0) <= regs_i.fwd_ep1_i;
            ack_sreg(0) <= '1';
            ack_in_progress <= '1';
          when "0101" => 
            if (wb_we_i = '1') then
            end if;
            rddata_reg(31 downto 0) <= regs_i.disc_ep0_i;
            ack_sreg(0) <= '1';
            ack_in_progress <= '1';
          when "0110" => 
            if (wb_we_i = '1') then
            end if;
            rddata_reg(31 downto 0) <= regs_i.disc_ep1_i;
            ack_sreg(0) <= '1';
            ack_in_progress <= '1';
          when "0111" => 
            if (wb_we_i = '1') then
            end if;
            rddata_reg(31 downto 0) <= regs_i.acc_ep0_i;
            ack_sreg(0) <= '1';
            ack_in_progress <= '1';
          when "1000" => 
            if (wb_we_i = '1') then
            end if;
            rddata_reg(31 downto 0) <= regs_i.acc_ep1_i;
            ack_sreg(0) <= '1';
            ack_in_progress <= '1';
          when "1001" => 
            if (wb_we_i = '1') then
            end if;
            rddata_reg(31 downto 0) <= regs_i.bound_ep0_i;
            ack_sreg(0) <= '1';
            ack_in_progress <= '1';
          when "1010" => 
            if (wb_we_i = '1') then
            end if;
            rddata_reg(31 downto 0) <= regs_i.bound_ep1_i;
            ack_sreg(0) <= '1';
            ack_in_progress <= '1';
          when "1011" => 
            if (wb_we_i = '1') then
            end if;
            rddata_reg(31 downto 0) <= regs_i.dup_ep0_i;
            ack_sreg(0) <= '1';
            ack_in_progress <= '1';
          when "1100" => 
            if (wb_we_i = '1') then
            end if;
            rddata_reg(31 downto 0) <= regs_i.dup_ep1_i;
            ack_sreg(0) <= '1';
            ack_in_progress <= '1';
          when "1101" => 
            if (wb_we_i = '1') then
            end if;
            rddata_reg(31 downto 0) <= regs_i.seq_ep0_i;
            ack_sreg(0) <= '1';
            ack_in_progress <= '1';
          when "1110" => 
            if (wb_we_i = '1') then
            end if;
            rddata_reg(31 downto 0) <= regs_i.seq_ep1_i;
            ack_sreg(0) <= '1';
            ack_in_progress <= '1';
          when others =>
-- prevent the slave from hanging the bus on invalid address
            ack_in_progress <= '1';
            ack_sreg(0) <= '1';
          end case;
        end if;
      end if;
    end if;
  end process;
  
  
-- Drive the data output bus
  wb_dat_o <= rddata_reg;
-- LRE reset
  process (clk_sys_i, rst_n_i)
  begin
    if (rst_n_i = '0') then 
      lre_lcr_rst_n_dly0 <= '0';
      regs_o.lcr_rst_n_o <= '0';
    elsif rising_edge(clk_sys_i) then
      lre_lcr_rst_n_dly0 <= lre_lcr_rst_n_int;
      regs_o.lcr_rst_n_o <= lre_lcr_rst_n_int and (not lre_lcr_rst_n_dly0);
    end if;
  end process;
  
  
-- LRE enable
  regs_o.lcr_en_o <= lre_lcr_en_int;
-- HSR / PRP mode
  regs_o.lcr_mode_o <= lre_lcr_mode_int;
-- HSR Path
  regs_o.lcr_hsr_path_o <= lre_lcr_hsr_path_int;
-- Link ok
-- Clear counters
  process (clk_sys_i, rst_n_i)
  begin
    if (rst_n_i = '0') then 
      lre_lcr_clr_cnt_dly0 <= '0';
      regs_o.lcr_clr_cnt_o <= '0';
    elsif rising_edge(clk_sys_i) then
      lre_lcr_clr_cnt_dly0 <= lre_lcr_clr_cnt_int;
      regs_o.lcr_clr_cnt_o <= lre_lcr_clr_cnt_int and (not lre_lcr_clr_cnt_dly0);
    end if;
  end process;
  
  
-- MAC Address
  regs_o.mach_o <= lre_mach_int;
-- MAC Address
  regs_o.macl_o <= lre_macl_int;
-- Fwd frames
-- Fwd frames
-- dropped frames
-- dropped frames
-- Accepted frames
-- Accepted frames
-- EP0-bound frames
-- EP1-bound frames
-- Duplicated frames
-- Duplicated frames
-- Sequence number
-- Sequence number
  rwaddr_reg <= wb_adr_i;
  wb_stall_o <= (not ack_sreg(0)) and (wb_stb_i and wb_cyc_i);
-- ACK signal generation. Just pass the LSB of ACK counter.
  wb_ack_o <= ack_sreg(0);
end syn;