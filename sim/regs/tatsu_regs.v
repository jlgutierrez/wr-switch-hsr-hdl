`define ADDR_TATSU_TCR                 5'h0
`define TATSU_TCR_VALIDATE_OFFSET 0
`define TATSU_TCR_VALIDATE 32'h00000001
`define TATSU_TCR_DISABLE_OFFSET 1
`define TATSU_TCR_DISABLE 32'h00000002
`define TATSU_TCR_DROP_ENA_OFFSET 8
`define TATSU_TCR_DROP_ENA 32'h00000100
`define TATSU_TCR_MIN_RPT_OFFSET 16
`define TATSU_TCR_MIN_RPT 32'h00ff0000
`define TATSU_TCR_STARTED_OFFSET 24
`define TATSU_TCR_STARTED 32'h01000000
`define TATSU_TCR_DELAYED_OFFSET 25
`define TATSU_TCR_DELAYED 32'h02000000
`define TATSU_TCR_STG_OK_OFFSET 26
`define TATSU_TCR_STG_OK 32'h04000000
`define TATSU_TCR_STG_ERR_OFFSET 27
`define TATSU_TCR_STG_ERR 32'h08000000
`define TATSU_TCR_STG_ERR_TAI_OFFSET 28
`define TATSU_TCR_STG_ERR_TAI 32'h10000000
`define TATSU_TCR_STG_ERR_CYC_OFFSET 29
`define TATSU_TCR_STG_ERR_CYC 32'h20000000
`define TATSU_TCR_STG_ERR_RPT_OFFSET 30
`define TATSU_TCR_STG_ERR_RPT 32'h40000000
`define TATSU_TCR_STG_ERR_SNC_OFFSET 31
`define TATSU_TCR_STG_ERR_SNC 32'h80000000
`define ADDR_TATSU_TSR0                5'h4
`define TATSU_TSR0_QNT_OFFSET 0
`define TATSU_TSR0_QNT 32'h0000ffff
`define TATSU_TSR0_PRIO_OFFSET 16
`define TATSU_TSR0_PRIO 32'h00ff0000
`define TATSU_TSR0_HTAI_OFFSET 24
`define TATSU_TSR0_HTAI 32'hff000000
`define ADDR_TATSU_TSR1                5'h8
`define TATSU_TSR1_LTAI_OFFSET 0
`define TATSU_TSR1_LTAI 32'hffffffff
`define ADDR_TATSU_TSR2                5'hc
`define TATSU_TSR2_CYC_OFFSET 0
`define TATSU_TSR2_CYC 32'h0fffffff
`define ADDR_TATSU_TSR3                5'h10
`define TATSU_TSR3_CYC_OFFSET 0
`define TATSU_TSR3_CYC 32'h0fffffff
`define ADDR_TATSU_TSR4                5'h14
`define TATSU_TSR4_PORTS_OFFSET 0
`define TATSU_TSR4_PORTS 32'hffffffff
