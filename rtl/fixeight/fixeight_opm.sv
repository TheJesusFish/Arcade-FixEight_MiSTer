// SPDX-License-Identifier: BSD-3-Clause
// FixEight-facing adapter for the pinned IKAOPM YM2151 implementation.

module fixeight_opm (
    input  logic               reset,
    input  logic               clk,
    input  logic               cen,
    input  logic               cs_n,
    input  logic               wr_n,
    input  logic               a0,
    input  logic [7:0]         din,
    output logic [7:0]         dout,
    output logic               sample,
    output logic signed [15:0] left,
    output logic signed [15:0] right
);

logic sample_left;
logic read_n;
assign read_n = cs_n | ~wr_n;

IKAOPM #(
    .FULLY_SYNCHRONOUS(1),
    .FAST_RESET(1),
    .USE_BRAM(1)
) u_opm (
    .i_EMUCLK        (clk),
    .i_phiM_PCEN_n   (~cen),
    .i_IC_n          (~reset),
    .o_phi1          (),
    .i_CS_n          (cs_n),
    .i_RD_n          (read_n),
    .i_WR_n          (wr_n),
    .i_A0            (a0),
    .i_D             (din),
    .o_D             (dout),
    .o_D_OE          (),
    .o_CT2           (),
    .o_CT1           (),
    .o_IRQ_n         (),
    .o_SH1           (),
    .o_SH2           (),
    .o_SO            (),
    .o_EMU_R_SAMPLE  (),
    .o_EMU_R_EX      (),
    .o_EMU_R         (right),
    .o_EMU_L_SAMPLE  (sample_left),
    .o_EMU_L_EX      (),
    .o_EMU_L         (left)
);

assign sample = sample_left;

endmodule
