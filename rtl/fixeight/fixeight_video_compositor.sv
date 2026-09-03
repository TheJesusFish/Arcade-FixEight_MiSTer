// TP-026 final composition: dynamic text overlays one GP9001 result.
// Palette RAM is xBGR555; the synchronous palette lookup is external.
module fixeight_video_compositor (
    input  logic        display_active,
    input  logic        hide_output,
    input  logic        gp_line_ready,
    input  logic [10:0] gp_color,
    input  logic        text_line_ready,
    input  logic        text_opaque,
    input  logic [10:0] text_color,
    input  logic [15:0] palette_word,

    output logic        line_ready,
    output logic [10:0] palette_addr,
    output logic [7:0]  red,
    output logic [7:0]  green,
    output logic [7:0]  blue
);

always_comb begin
    line_ready = gp_line_ready && text_line_ready;
    palette_addr = text_opaque ? text_color : gp_color;

    red = 8'h00;
    green = 8'h00;
    blue = 8'h00;
    if (display_active && !hide_output && line_ready) begin
        red = {palette_word[4:0], palette_word[4:2]};
        green = {palette_word[9:5], palette_word[9:7]};
        blue = {palette_word[14:10], palette_word[14:12]};
    end
end

endmodule
