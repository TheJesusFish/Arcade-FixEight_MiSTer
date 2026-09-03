// Resolve TP-026's one GP9001 tile/object line with MAME priority semantics.
module fixeight_gp9001_line_mixer (
    input      [8:0]  display_y,
    input             display_epoch,
    input      [14:0] tile_pixel,
    input      [8:0]  tile_y,
    input             tile_epoch,
    input             tile_valid,
    input      [14:0] object_pixel,
    input      [8:0]  object_y,
    input             object_epoch,
    input             object_valid,
    output            line_ready,
    output     [10:0] final_color
);

wire sources_ready = tile_valid && object_valid &&
                     (tile_y == display_y) &&
                     (object_y == display_y) &&
                     (tile_epoch == display_epoch) &&
                     (object_epoch == display_epoch);

wire object_wins = (object_pixel[3:0] != 4'h0) &&
                   (object_pixel[14:11] >= tile_pixel[14:11]);
wire [14:0] mixed = object_wins ? object_pixel : tile_pixel;

assign line_ready = sources_ready;
assign final_color = sources_ready ? mixed[10:0] : 11'd0;

endmodule
