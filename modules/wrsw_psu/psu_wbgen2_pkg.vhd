---------------------------------------------------------------------------------------
-- Title          : Wishbone slave core for WR PTP Support Unit Controller
---------------------------------------------------------------------------------------
-- File           : psu_wbgen2_pkg.vhd
-- Author         : auto-generated by wbgen2 from psu_wishbone_controller.wb
-- Created        : Wed Mar 25 20:00:25 2015
-- Standard       : VHDL'87
---------------------------------------------------------------------------------------
-- THIS FILE WAS GENERATED BY wbgen2 FROM SOURCE FILE psu_wishbone_controller.wb
-- DO NOT HAND-EDIT UNLESS IT'S ABSOLUTELY NECESSARY!
---------------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

package psu_wbgen2_pkg is
  
  
  -- Input registers (user design -> WB slave)
  
  type t_psu_in_registers is record
    psr_holdover_on_i                        : std_logic;
    psr_hd_msg_rx_i                          : std_logic;
    psr_hd_on_spll_i                         : std_logic;
    psr_active_ref_spll_i                    : std_logic_vector(23 downto 0);
    ptd_tx_ram_dat_valid_i                   : std_logic;
    ptd_tx_ram_rd_dat_i                      : std_logic_vector(17 downto 0);
    end record;
  
  constant c_psu_in_registers_init_value: t_psu_in_registers := (
    psr_holdover_on_i => '0',
    psr_hd_msg_rx_i => '0',
    psr_hd_on_spll_i => '0',
    psr_active_ref_spll_i => (others => '0'),
    ptd_tx_ram_dat_valid_i => '0',
    ptd_tx_ram_rd_dat_i => (others => '0')
    );
    
    -- Output registers (WB slave -> user design)
    
    type t_psu_out_registers is record
      pcr_psu_ena_o                            : std_logic;
      pcr_psu_clr_tx_msg_o                     : std_logic;
      pcr_inj_prio_o                           : std_logic_vector(2 downto 0);
      pcr_holdover_clk_class_o                 : std_logic_vector(7 downto 0);
      ptcr_seqid_dup_drop_o                    : std_logic;
      ptcr_seqid_wrg_drop_o                    : std_logic;
      ptcr_clkcl_wrg_drop_o                    : std_logic;
      ptcr_prtid_wrg_drop_o                    : std_logic;
      prcr_seqid_dup_det_o                     : std_logic;
      prcr_seqid_wrg_det_o                     : std_logic;
      prcr_prtid_wrg_det_o                     : std_logic;
      rxpm_port_mask_o                         : std_logic_vector(31 downto 0);
      txpm_port_mask_o                         : std_logic_vector(31 downto 0);
      ptd_dbg_holdover_on_o                    : std_logic;
      ptd_tx_ram_rd_adr_o                      : std_logic_vector(9 downto 0);
      end record;
    
    constant c_psu_out_registers_init_value: t_psu_out_registers := (
      pcr_psu_ena_o => '0',
      pcr_psu_clr_tx_msg_o => '0',
      pcr_inj_prio_o => (others => '0'),
      pcr_holdover_clk_class_o => (others => '0'),
      ptcr_seqid_dup_drop_o => '0',
      ptcr_seqid_wrg_drop_o => '0',
      ptcr_clkcl_wrg_drop_o => '0',
      ptcr_prtid_wrg_drop_o => '0',
      prcr_seqid_dup_det_o => '0',
      prcr_seqid_wrg_det_o => '0',
      prcr_prtid_wrg_det_o => '0',
      rxpm_port_mask_o => (others => '0'),
      txpm_port_mask_o => (others => '0'),
      ptd_dbg_holdover_on_o => '0',
      ptd_tx_ram_rd_adr_o => (others => '0')
      );
    function "or" (left, right: t_psu_in_registers) return t_psu_in_registers;
    function f_x_to_zero (x:std_logic) return std_logic;
    function f_x_to_zero (x:std_logic_vector) return std_logic_vector;
end package;

package body psu_wbgen2_pkg is
function f_x_to_zero (x:std_logic) return std_logic is
begin
if x = '1' then
return '1';
else
return '0';
end if;
end function;
function f_x_to_zero (x:std_logic_vector) return std_logic_vector is
variable tmp: std_logic_vector(x'length-1 downto 0);
begin
for i in 0 to x'length-1 loop
if(x(i) = 'X' or x(i) = 'U') then
tmp(i):= '0';
else
tmp(i):=x(i);
end if; 
end loop; 
return tmp;
end function;
function "or" (left, right: t_psu_in_registers) return t_psu_in_registers is
variable tmp: t_psu_in_registers;
begin
tmp.psr_holdover_on_i := f_x_to_zero(left.psr_holdover_on_i) or f_x_to_zero(right.psr_holdover_on_i);
tmp.psr_hd_msg_rx_i := f_x_to_zero(left.psr_hd_msg_rx_i) or f_x_to_zero(right.psr_hd_msg_rx_i);
tmp.psr_hd_on_spll_i := f_x_to_zero(left.psr_hd_on_spll_i) or f_x_to_zero(right.psr_hd_on_spll_i);
tmp.psr_active_ref_spll_i := f_x_to_zero(left.psr_active_ref_spll_i) or f_x_to_zero(right.psr_active_ref_spll_i);
tmp.ptd_tx_ram_dat_valid_i := f_x_to_zero(left.ptd_tx_ram_dat_valid_i) or f_x_to_zero(right.ptd_tx_ram_dat_valid_i);
tmp.ptd_tx_ram_rd_dat_i := f_x_to_zero(left.ptd_tx_ram_rd_dat_i) or f_x_to_zero(right.ptd_tx_ram_rd_dat_i);
return tmp;
end function;
end package body;
