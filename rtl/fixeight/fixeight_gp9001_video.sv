// TP-026 one-GP9001 line renderer. Tile and buffered-object engines build in
// parallel and publish only fully tagged lines.
module fixeight_gp9001_video (
    input              clk,
    input              rst,
    input              line_start,
    input              line_commit,
    input      [8:0]   target_y,
    input              target_epoch,
    input      [8:0]   display_x,
    input      [8:0]   display_y,
    input              display_epoch,

    input      [127:0] scrolls,
    input      [7:0]   scroll_flip,

    output     [12:0]  vram_addr,
    input      [15:0]  vram_data,
    output     [9:0]   object_addr,
    input      [15:0]  object_data,

    output             tile_gfx_req,
    output     [21:0]  tile_gfx_addr,
    input      [15:0]  tile_gfx_data,
    input              tile_gfx_ok,
    output             object_gfx_req,
    output     [21:0]  object_gfx_addr,
    input      [15:0]  object_gfx_data,
    input              object_gfx_ok,

    output             line_ready,
    output     [10:0]  final_color,
    output     [1:0]   engine_busy,
    output     [1:0]   deadline_miss,
    output     [15:0]  tile_cycles,
    output     [15:0]  object_cycles,
    output     [8:0]   debug_tile_y,
    output     [8:0]   debug_object_y,
    output     [1:0]   debug_line_epoch,
    output     [1:0]   debug_line_valid
);

wire [14:0] tile_pixel;
wire [14:0] object_pixel;
wire [8:0] tile_y;
wire [8:0] object_y;
wire tile_epoch;
wire object_epoch;
wire tile_valid;
wire object_valid;
wire [1:0] engine_done;

assign debug_tile_y = tile_y;
assign debug_object_y = object_y;
assign debug_line_epoch = {object_epoch, tile_epoch};
assign debug_line_valid = {object_valid, tile_valid};

fixeight_gp9001_tile_line u_tile (
    .clk               (clk),
    .rst               (rst),
    .start             (line_start),
    .commit            (line_commit),
    .target_y          (target_y),
    .target_epoch      (target_epoch),
    .scrolls           (scrolls),
    .scroll_flip       (scroll_flip),
    .busy              (engine_busy[0]),
    .done              (engine_done[0]),
    .deadline_miss     (deadline_miss[0]),
    .last_build_cycles (tile_cycles),
    .vram_addr         (vram_addr),
    .vram_data         (vram_data),
    .gfx_req           (tile_gfx_req),
    .gfx_addr          (tile_gfx_addr),
    .gfx_data          (tile_gfx_data),
    .gfx_ok            (tile_gfx_ok),
    .scan_x            (display_x),
    .scan_pixel        (tile_pixel),
    .scan_y            (tile_y),
    .scan_epoch        (tile_epoch),
    .scan_valid        (tile_valid)
);

fixeight_gp9001_object_line u_object (
    .clk               (clk),
    .rst               (rst),
    .start             (line_start),
    .commit            (line_commit),
    .target_y          (target_y),
    .target_epoch      (target_epoch),
    .scrolls           (scrolls),
    .scroll_flip       (scroll_flip),
    .busy              (engine_busy[1]),
    .done              (engine_done[1]),
    .deadline_miss     (deadline_miss[1]),
    .last_build_cycles (object_cycles),
    .object_addr       (object_addr),
    .object_data       (object_data),
    .gfx_req           (object_gfx_req),
    .gfx_addr          (object_gfx_addr),
    .gfx_data          (object_gfx_data),
    .gfx_ok            (object_gfx_ok),
    .scan_x            (display_x),
    .scan_pixel        (object_pixel),
    .scan_y            (object_y),
    .scan_epoch        (object_epoch),
    .scan_valid        (object_valid)
);

fixeight_gp9001_line_mixer u_mixer (
    .display_y    (display_y),
    .display_epoch(display_epoch),
    .tile_pixel   (tile_pixel),
    .tile_y       (tile_y),
    .tile_epoch   (tile_epoch),
    .tile_valid   (tile_valid),
    .object_pixel (object_pixel),
    .object_y     (object_y),
    .object_epoch (object_epoch),
    .object_valid (object_valid),
    .line_ready   (line_ready),
    .final_color  (final_color)
);

endmodule
