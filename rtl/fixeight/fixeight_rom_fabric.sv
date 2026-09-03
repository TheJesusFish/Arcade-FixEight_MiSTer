// Three-bank JTFrame SDRAM read fabric:
// bank 0 main program, bank 1 paired GP graphics, bank 2 OKI samples.
module fixeight_rom_fabric #(
    parameter integer AW = 22
) (
    input  logic              clk,
    input  logic              reset,
    input  logic              invalidate,

    input  logic              main_req,
    input  logic [18:0]       main_word_addr,
    output logic [15:0]       main_data,
    output logic              main_ack,

    input  logic              tile_req,
    input  logic [21:0]       tile_logical_addr,
    output logic [15:0]       tile_data,
    output logic              tile_ack,

    input  logic              object_req,
    input  logic [21:0]       object_logical_addr,
    output logic [15:0]       object_data,
    output logic              object_ack,

    input  logic [17:0]       oki_addr,
    output logic [7:0]        oki_data,
    output logic              oki_ack,

    output logic [AW-1:0]     ba0_addr,
    output logic [AW-1:0]     ba1_addr,
    output logic [AW-1:0]     ba2_addr,
    output logic [AW-1:0]     ba3_addr,
    output logic [3:0]        ba_rd,
    output logic [3:0]        ba_wr,
    input  logic [3:0]        ba_ack,
    input  logic [3:0]        ba_dst,
    input  logic [3:0]        ba_rdy,
    input  logic [15:0]       data_read,
    output logic [15:0]       ba0_din,
    output logic [15:0]       ba1_din,
    output logic [15:0]       ba2_din,
    output logic [15:0]       ba3_din,
    output logic [1:0]        ba0_dsn,
    output logic [1:0]        ba1_dsn,
    output logic [1:0]        ba2_dsn,
    output logic [1:0]        ba3_dsn
);

wire fabric_reset = reset || invalidate;
logic main_sdram_rd;
logic gfx_sdram_rd;
logic oki_sdram_rd;
logic [21:0] tile_physical_addr;
logic [21:0] object_physical_addr;

fixeight_gfx_repack u_tile_repack (
    .logical_addr(tile_logical_addr),
    .physical_addr(tile_physical_addr)
);

fixeight_gfx_repack u_object_repack (
    .logical_addr(object_logical_addr),
    .physical_addr(object_physical_addr)
);

jtframe_rom_1slot #(
    .SDRAMW(AW),
    .SLOT0_DW(16),
    .SLOT0_AW(19),
    .SLOT0_LATCH(0),
    .SLOT0_OKLATCH(0)
) u_main (
    .rst(fabric_reset),
    .clk,
    .slot0_addr(main_word_addr),
    .slot0_dout(main_data),
    .slot0_cs(main_req),
    .slot0_ok(main_ack),
    .sdram_ack(ba_ack[0]),
    .sdram_rd(main_sdram_rd),
    .sdram_addr(ba0_addr),
    .data_dst(ba_dst[0]),
    .data_rdy(ba_rdy[0]),
    .data_read
);

jtframe_rom_2slots #(
    .SDRAMW(AW),
    .SLOT0_DW(16),
    .SLOT1_DW(16),
    .SLOT0_AW(22),
    .SLOT1_AW(22),
    .SLOT0_LATCH(0),
    .SLOT1_LATCH(0),
    .SLOT0_OKLATCH(0),
    .SLOT1_OKLATCH(0)
) u_graphics (
    .rst(fabric_reset),
    .clk,
    .slot0_addr(tile_physical_addr),
    .slot1_addr(object_physical_addr),
    .slot0_dout(tile_data),
    .slot1_dout(object_data),
    .slot0_cs(tile_req),
    .slot1_cs(object_req),
    .slot0_ok(tile_ack),
    .slot1_ok(object_ack),
    .sdram_ack(ba_ack[1]),
    .sdram_rd(gfx_sdram_rd),
    .sdram_addr(ba1_addr),
    .data_dst(ba_dst[1]),
    .data_rdy(ba_rdy[1]),
    .data_read
);

jtframe_rom_1slot #(
    .SDRAMW(AW),
    .SLOT0_DW(8),
    .SLOT0_AW(18),
    .SLOT0_LATCH(0),
    .SLOT0_OKLATCH(0)
) u_oki (
    .rst(fabric_reset),
    .clk,
    .slot0_addr(oki_addr),
    .slot0_dout(oki_data),
    .slot0_cs(1'b1),
    .slot0_ok(oki_ack),
    .sdram_ack(ba_ack[2]),
    .sdram_rd(oki_sdram_rd),
    .sdram_addr(ba2_addr),
    .data_dst(ba_dst[2]),
    .data_rdy(ba_rdy[2]),
    .data_read
);

assign ba3_addr = {AW{1'b0}};
assign ba_rd = {1'b0, oki_sdram_rd, gfx_sdram_rd, main_sdram_rd};
assign ba_wr = 4'b0000;
assign ba0_din = 16'd0;
assign ba1_din = 16'd0;
assign ba2_din = 16'd0;
assign ba3_din = 16'd0;
assign ba0_dsn = 2'b11;
assign ba1_dsn = 2'b11;
assign ba2_dsn = 2'b11;
assign ba3_dsn = 2'b11;

endmodule
