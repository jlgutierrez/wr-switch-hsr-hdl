-------------------------------------------------------------------------------
-- Title      : Routing Table Unit's Components Package 
-- Project    : White Rabbit Switch
-------------------------------------------------------------------------------
-- File       : wrsw_rtu_components_pkg.vhd
-- Author     : Maciej Lipinski
-- Company    : CERN BE-Co-HT
-- Created    : 2010-05-09
-- Last update: 2012-06-25
-- Platform   : FPGA-generic
-- Standard   : VHDL
-------------------------------------------------------------------------------
-- Routing Table Unit components
-- 
--
-------------------------------------------------------------------------------
--
-- Copyright (c) 2010 Maciej Lipinski / CERN
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
-- Revisions  :
-- Date        Version  Author          Description
-- 2010-05-09  1.0      lipinskimm          Created
-------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;

library work;

use work.wishbone_pkg.all;              -- for test part (to be moved)
use work.wrsw_shared_types_pkg.all;
use work.rtu_wbgen2_pkg.all;

package rtu_private_pkg is

----------------------------------------------------------------------------------------
--| RTU top level
----------------------------------------------------------------------------------------

-- Number of switch ports (including NIC)
  
  constant c_wrsw_mac_addr_width     : integer                           := 48;
  constant c_wrsw_vid_width          : integer                           := 12;
  constant c_wrsw_prio_width         : integer                           := 3;
  constant c_wrsw_prio_levels        : integer                           := 8;
  constant c_wrsw_fid_width          : integer                           := 8;
  constant c_wrsw_hash_width         : integer                           := 9;
  constant c_wrsw_crc_width          : integer                           := 16;
  constant c_wrsw_cam_addr_width     : integer                           := 8;
  constant c_wrsw_entry_words_number : std_logic_vector(5 downto 0)      := "000101";
  constant c_wrsw_rtu_debugging      : std_logic                         := '0';
  constant c_default_hash_poly       : std_logic_vector(16 - 1 downto 0) := x"1021";

  constant c_PACKED_REQUEST_WIDTH : integer :=
    c_wrsw_mac_addr_width  -- src MAC
    + c_wrsw_mac_addr_width  -- dst MAC
    + c_wrsw_vid_width  -- VID
    + c_wrsw_prio_width  -- priority
    + 1  -- has_VID
    + 1; -- has_priority

  constant c_PACKED_RESPONSE_WIDTH  : integer :=
    c_rtu_max_ports                    -- DPM size
    + 1                                 -- drop bit
    + c_wrsw_prio_width;                -- priority
  
  
  type t_rtu_htab_entry is record
    valid     : std_logic;
    is_bpdu   : std_logic;
    go_to_cam : std_logic;
    cam_addr  : std_logic_vector(c_wrsw_cam_addr_width-1 downto 0);
    fid       : std_logic_vector(c_wrsw_fid_width-1 downto 0);
    mac       : std_logic_vector(47 downto 0);

    bucket_entry : std_logic_vector(1 downto 0);

    port_mask_src            : std_logic_vector(c_rtu_max_ports-1 downto 0);
    port_mask_dst            : std_logic_vector(c_rtu_max_ports-1 downto 0);
    drop_when_src            : std_logic;
    drop_when_dst            : std_logic;
    drop_unmatched_src_ports : std_logic;

    prio_src          : std_logic_vector(c_wrsw_prio_width-1 downto 0);
    has_prio_src      : std_logic;
    prio_override_src : std_logic;

    prio_dst          : std_logic_vector(c_wrsw_prio_width-1 downto 0);
    has_prio_dst      : std_logic;
    prio_override_dst : std_logic;
    
  end record;

  
  type t_rtu_vlan_tab_entry is record
    drop          : std_logic;
    prio_override : std_logic;
    prio          : std_logic_vector(c_wrsw_prio_width-1 downto 0);
    has_prio      : std_logic;
    fid           : std_logic_vector(7 downto 0);
    port_mask     : std_logic_vector(c_rtu_max_ports-1 downto 0);
  end record;

  component wrsw_rtu
    generic(
      g_num_ports: integer);
    port (
      clk_sys_i           : in  std_logic;
      clk_match_i         : in  std_logic;
      rst_n_i             : in  std_logic;
      rtu_idle_o          : out std_logic_vector(g_num_ports-1 downto 0);
      rq_strobe_p_i       : in  std_logic_vector(g_num_ports-1 downto 0);
      rq_smac_i           : in  std_logic_vector(c_wrsw_mac_addr_width * g_num_ports - 1 downto 0);
      rq_dmac_i           : in  std_logic_vector(c_wrsw_mac_addr_width * g_num_ports -1 downto 0);
      rq_vid_i            : in  std_logic_vector(c_wrsw_vid_width * g_num_ports - 1 downto 0);
      rq_has_vid_i        : in  std_logic_vector(g_num_ports -1 downto 0);
      rq_prio_i           : in  std_logic_vector(c_wrsw_prio_width * g_num_ports -1 downto 0);
      rq_has_prio_i       : in  std_logic_vector(g_num_ports -1 downto 0);
      rsp_valid_o         : out std_logic_vector (g_num_ports-1 downto 0);
      rsp_dst_port_mask_o : out std_logic_vector(c_rtu_max_ports * g_num_ports - 1 downto 0);
      rsp_drop_o          : out std_logic_vector(g_num_ports -1 downto 0);
      rsp_prio_o          : out std_logic_vector (g_num_ports * c_wrsw_prio_width-1 downto 0);
      rsp_ack_i           : in  std_logic_vector(g_num_ports -1 downto 0);
      port_almost_full_o  : out std_logic_vector(g_num_ports -1 downto 0);
      port_full_o         : out std_logic_vector(g_num_ports -1 downto 0);
      wb_addr_i           : in  std_logic_vector(13 downto 0);
      wb_data_i           : in  std_logic_vector(31 downto 0);
      wb_data_o           : out std_logic_vector(31 downto 0);
      wb_sel_i            : in  std_logic_vector(3 downto 0);
      wb_cyc_i            : in  std_logic;
      wb_stb_i            : in  std_logic;
      wb_ack_o            : out std_logic;
      wb_irq_o            : out std_logic;
      wb_we_i             : in  std_logic);
  end component;
----------------------------------------------------------------------------------------
--| RTU port
----------------------------------------------------------------------------------------

  component rtu_port
    generic (
      g_num_ports  : integer;
      g_port_index : integer);
    port (
      clk_i                  : in  std_logic;
      rst_n_i                : in  std_logic;
      rtu_gcr_g_ena_i        : in  std_logic;
      rtu_idle_o             : out std_logic;
      rq_strobe_p_i          : in  std_logic;
      rq_smac_i              : in  std_logic_vector(c_wrsw_mac_addr_width - 1 downto 0);
      rq_dmac_i              : in  std_logic_vector(c_wrsw_mac_addr_width - 1 downto 0);
      rq_vid_i               : in  std_logic_vector(c_wrsw_vid_width - 1 downto 0);
      rq_has_vid_i           : in  std_logic;
      rq_prio_i              : in  std_logic_vector(c_wrsw_prio_width -1 downto 0);
      rq_has_prio_i          : in  std_logic;
      rsp_valid_o            : out std_logic;
      rsp_dst_port_mask_o    : out std_logic_vector(c_rtu_max_ports - 1 downto 0);
      rsp_drop_o             : out std_logic;
      rsp_prio_o             : out std_logic_vector (c_wrsw_prio_width-1 downto 0);
      rsp_ack_i              : in  std_logic;
      rq_fifo_write_o        : out std_logic;
      rq_fifo_full_i         : in  std_logic;
      rq_fifo_data_o         : out std_logic_vector(c_PACKED_REQUEST_WIDTH - 1 downto 0);
      rsp_write_i            : in  std_logic;
      rsp_match_data_i       : in  std_logic_vector(g_num_ports + c_PACKED_RESPONSE_WIDTH - 1 downto 0);
      rr_request_wr_access_o : out std_logic;
      rr_access_ena_i        : in  std_logic;
      port_almost_full_o     : out std_logic;
      port_full_o            : out std_logic;
      rq_rsp_cnt_dec_i       : in  std_logic;
      rtu_pcr_pass_bpdu_i    : in  std_logic;
      rtu_pcr_pass_all_i     : in  std_logic;
      rtu_pcr_fix_prio_i     : in  std_logic;
      rtu_pcr_prio_val_i     : in  std_logic_vector(c_wrsw_prio_width - 1 downto 0));
  end component;
  
----------------------------------------------------------------------------------------
--| Round Robin Arbiter
----------------------------------------------------------------------------------------

  component rtu_rr_arbiter is
    generic (
      g_width : natural);
    port (
      clk_i, rst_n_i : in  std_logic;
      req_i          : in  std_logic_vector(g_width - 1 downto 0);
      gnt_o          : out std_logic_vector(g_width - 1 downto 0)
      );
  end component;

----------------------------------------------------------------------------------------
--| WISHBONE
----------------------------------------------------------------------------------------

  component rtu_lookup_engine
    generic (
      g_num_ports : integer;
      g_hash_size : integer := c_wrsw_hash_width);
    port (
      clk_match_i      : in  std_logic;
      clk_sys_i        : in  std_logic;
      rst_n_i          : in  std_logic;
      mfifo_rd_req_o   : out std_logic;
      mfifo_rd_empty_i : in  std_logic;
      mfifo_ad_sel_i   : in  std_logic;
      mfifo_ad_val_i   : in  std_logic_vector(31 downto 0);
      mfifo_trigger_i  : in  std_logic;
      mfifo_busy_o     : out std_logic;
      start_i          : in  std_logic;
      ack_i            : in  std_logic;
      found_o          : out std_logic;
      hash_i           : in  std_logic_vector(g_hash_size-1 downto 0);
      mac_i            : in  std_logic_vector(c_wrsw_mac_addr_width -1 downto 0);
      fid_i            : in  std_logic_vector(c_wrsw_fid_width - 1 downto 0);
      drdy_o           : out std_logic;
      port_i           : in std_logic_vector(g_num_ports -1 downto 0); -- ML (24/03/2013): aging bugfix
      src_dst_i        : in std_logic;                                 -- ML (24/03/2013): aging bugfix
      entry_o          : out t_rtu_htab_entry);
  end component;

----------------------------------------------------------------------------------------
--| CRC-based hash calculation
----------------------------------------------------------------------------------------
  component rtu_crc
    port (
      mac_addr_i : in  std_logic_vector(c_wrsw_mac_addr_width - 1 downto 0);
      fid_i      : in  std_logic_vector(c_wrsw_fid_width - 1 downto 0);
      crc_poly_i : in  std_logic_vector(c_wrsw_crc_width - 1 downto 0);
      hash_o     : out std_logic_vector(c_wrsw_hash_width - 1 downto 0)
      );
  end component;

  component rtu_match
    generic (
      g_num_ports : integer);
    port (
      clk_i                : in  std_logic;
      rst_n_i              : in  std_logic;
      rq_fifo_read_o       : out std_logic;
      rq_fifo_empty_i      : in  std_logic;
      rq_fifo_input_i      : in  std_logic_vector(g_num_ports + c_PACKED_REQUEST_WIDTH - 1 downto 0);
      rsp_fifo_write_o     : out std_logic;
      rsp_fifo_full_i      : in  std_logic;
      rsp_fifo_output_o    : out std_logic_vector(g_num_ports + c_PACKED_RESPONSE_WIDTH - 1 downto 0);
      htab_start_o         : out std_logic;
      htab_ack_o           : out std_logic;
      htab_found_i         : in  std_logic;
      htab_hash_o          : out std_logic_vector(c_wrsw_hash_width - 1 downto 0);
      htab_mac_o           : out std_logic_vector(c_wrsw_mac_addr_width -1 downto 0);
      htab_fid_o           : out std_logic_vector(c_wrsw_fid_width - 1 downto 0);
      htab_drdy_i          : in  std_logic;
      htab_entry_i         : in  t_rtu_htab_entry;
      htab_port_o          : out std_logic_vector(g_num_ports-1 downto 0); -- ML (24/03/2013): aging bugfix
      htab_src_dst_o       : out std_logic;                                -- ML (24/03/2013): aging bugfix
      rtu_ufifo_wr_req_o   : out std_logic;
      rtu_ufifo_wr_full_i  : in  std_logic;
      rtu_ufifo_wr_empty_i : in  std_logic;
      rtu_ufifo_dmac_lo_o  : out std_logic_vector(31 downto 0);
      rtu_ufifo_dmac_hi_o  : out std_logic_vector(15 downto 0);
      rtu_ufifo_smac_lo_o  : out std_logic_vector(31 downto 0);
      rtu_ufifo_smac_hi_o  : out std_logic_vector(15 downto 0);
      rtu_ufifo_vid_o      : out std_logic_vector(c_wrsw_vid_width - 1 downto 0);
      rtu_ufifo_prio_o     : out std_logic_vector(2 downto 0);
      rtu_ufifo_pid_o      : out std_logic_vector(7 downto 0);
      rtu_ufifo_has_vid_o  : out std_logic;
      rtu_ufifo_has_prio_o : out std_logic;
      rtu_aram_main_addr_o : out std_logic_vector(7 downto 0);
      rtu_aram_main_data_i : in  std_logic_vector(31 downto 0);
      rtu_aram_main_rd_o   : out std_logic;
      rtu_aram_main_data_o : out std_logic_vector(31 downto 0);
      rtu_aram_main_wr_o   : out std_logic;
      vlan_tab_addr_o      : out std_logic_vector(c_wrsw_vid_width - 1 downto 0);
      vlan_tab_entry_i     : in  t_rtu_vlan_tab_entry;
      rtu_gcr_g_ena_i      : in  std_logic;
      rtu_pcr_pass_all_i   : in  std_logic_vector(g_num_ports - 1 downto 0);
      rtu_pcr_learn_en_i   : in  std_logic_vector(g_num_ports - 1 downto 0);
      rtu_pcr_pass_bpdu_i  : in  std_logic_vector(g_num_ports - 1 downto 0);
      rtu_pcr_b_unrec_i    : in  std_logic_vector(g_num_ports - 1 downto 0);
      rtu_crc_poly_i       : in  std_logic_vector(c_wrsw_crc_width - 1 downto 0));
  end component;

  component rtu_wishbone_slave
    port (
      rst_n_i         : in  std_logic;
      clk_sys_i       : in  std_logic;
      wb_adr_i        : in  std_logic_vector(8 downto 0);
      wb_dat_i        : in  std_logic_vector(31 downto 0);
      wb_dat_o        : out std_logic_vector(31 downto 0);
      wb_cyc_i        : in  std_logic;
      wb_sel_i        : in  std_logic_vector(3 downto 0);
      wb_stb_i        : in  std_logic;
      wb_we_i         : in  std_logic;
      wb_ack_o        : out std_logic;
      wb_stall_o      : out std_logic;
      wb_int_o        : out std_logic;
      clk_match_i     : in  std_logic;
      irq_nempty_i    : in  std_logic;
      rtu_aram_addr_i : in  std_logic_vector(7 downto 0);
      rtu_aram_data_o : out std_logic_vector(31 downto 0);
      rtu_aram_rd_i   : in  std_logic;
      rtu_aram_data_i : in  std_logic_vector(31 downto 0);
      rtu_aram_wr_i   : in  std_logic;
      regs_i          : in  t_rtu_in_registers;
      regs_o          : out t_rtu_out_registers);
  end component;

  function f_unmarshall_htab_entry (w0, w1, w2, w3, w4 : std_logic_vector) return t_rtu_htab_entry;

end package rtu_private_pkg;


package body rtu_private_pkg is


  function f_unmarshall_htab_entry (w0, w1, w2, w3, w4 : std_logic_vector) return t_rtu_htab_entry is
    variable t : t_rtu_htab_entry;
  begin
    t.bucket_entry := "ZZ";
    t.valid        := w0(0);
    t.is_bpdu      := w0(2);
    t.go_to_cam    := w0(3);
    t.cam_addr     := w2(c_wrsw_cam_addr_width-1 downto 0);
    t.has_prio_src := w2(16);
    t.prio_src     := w2(17 + c_wrsw_prio_width -1 downto 17);
    t.fid          := w0(4 + c_wrsw_fid_width - 1 downto 4);

    t.prio_override_src        := w2(20);
    t.drop_when_src            := w2(21);
    t.drop_unmatched_src_ports := w2(22);
    t.has_prio_dst             := w2(23);
    t.prio_dst                 := w2(24 + c_wrsw_prio_width - 1 downto 24);
    t.prio_override_dst        := w2(27);
    t.drop_when_dst            := w2(28);

    -- src/dst masks
    t.port_mask_src(15 downto 0)                  := w3(15 downto 0);
    t.port_mask_dst(15 downto 0)                  := w3(31 downto 16);
    t.port_mask_src(c_rtu_max_ports-1 downto 16) := w4(c_rtu_max_ports-16-1 downto 0);
    t.port_mask_dst(c_rtu_max_ports-1 downto 16) := w4(c_rtu_max_ports-1 downto 16);

    -- zbt adddr
    -- MAC addr (used only interally, no need to output)   
    t.mac(47 downto 32) := w0(31 downto 16);
    t.mac(31 downto 0)  := w1(31 downto 0);

    return t;
  end function f_unmarshall_htab_entry;



end rtu_private_pkg;
