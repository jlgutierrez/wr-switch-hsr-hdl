SRCS_VHDL = swc_swcore_pkg.vhd \
swc_block_alloc.vhd \
swc_core.vhd \
swc_input_block.vhd \
swc_lost_pck_dealloc.vhd \
swc_multiport_linked_list.vhd \
swc_multiport_page_allocator.vhd \
swc_multiport_pck_pg_free_module.vhd \
swc_ob_prio_queue.vhd \
swc_output_block.vhd \
swc_packet_mem.vhd \
swc_packet_mem_read_pump.vhd \
swc_packet_mem_write_pump.vhd \
swc_page_alloc.vhd \
swc_pck_pg_free_module.vhd \
swc_pck_transfer_arbiter.vhd \
swc_pck_transfer_input.vhd \
swc_pck_transfer_output.vhd \
swc_prio_encoder.vhd \
swc_rr_arbiter.vhd 

WORK = work

#directories in which we should search for the VHDL/verilog source files
VPATH =

include ../../scripts/modules.mk



