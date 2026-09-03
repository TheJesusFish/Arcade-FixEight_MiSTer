// FixEight-local framebuffer window; equivalent to the Batsugun menu adapter.
// Cropping changes only the scaler's read window, never screen_rotate's writes.
module fixeight_rotated_crop #(
    parameter [11:0] CROP_HEIGHT = 12'd270
) (
    input  logic        enable,
    input  logic signed [4:0] offset,
    input  logic [11:0] width_in,
    input  logic [11:0] height_in,
    input  logic [31:0] base_in,
    input  logic [13:0] stride_in,
    output logic        active,
    output logic [11:0] width_out,
    output logic [11:0] height_out,
    output logic [31:0] base_out
);
wire [11:0] margin = height_in >= CROP_HEIGHT ? height_in - CROP_HEIGHT : 12'd0;
wire [11:0] center = margin >> 1;
wire signed [13:0] requested = $signed({2'b00, center}) + {{9{offset[4]}}, offset};
wire [11:0] first_line = requested < 0 ? 12'd0 :
                          requested > $signed({2'b00, margin}) ? margin : requested[11:0];
wire [25:0] base_offset = first_line * stride_in;
assign active = enable && height_in >= CROP_HEIGHT && stride_in != 0;
assign width_out = width_in;
assign height_out = active ? CROP_HEIGHT : height_in;
assign base_out = active ? base_in + {6'd0, base_offset} : base_in;
endmodule
