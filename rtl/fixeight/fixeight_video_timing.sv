// TP-026 native 432x262 raster in the authoritative 94.5 MHz domain.
// All outputs are synchronous-domain events; ce_pixel is a clock enable.
module fixeight_video_timing (
    input  logic       clk,
    input  logic       reset,
    input  logic       ce_pixel,
    input  logic       hold,
    input  logic       restore_commit,
    input  logic [9:0] restore_h_count,
    input  logic [8:0] restore_v_count,
    input  logic       restore_frame_epoch,

    output logic [9:0] h_count,
    output logic [8:0] v_count,
    output logic       frame_epoch,
    output logic       pixel_tick,
    output logic       hblank,
    output logic       vblank,
    output logic       active,
    output logic       hsync_n,
    output logic       vsync_n,

    output logic       irq4_start,
    output logic       snapshot_start,
    output logic       frame_tick,
    output logic       line_start,
    output logic       line_commit,
    output logic [8:0] target_y,
    output logic       target_epoch
);

localparam logic [9:0] H_TOTAL = 10'd432;
localparam logic [8:0] V_TOTAL = 9'd262;

wire line_end = ce_pixel && !hold && (h_count == H_TOTAL - 10'd1);

assign pixel_tick = ce_pixel && !hold;
assign hblank = h_count >= 10'd320;
assign vblank = v_count >= 9'd240;
assign active = !hblank && !vblank;
assign hsync_n = !((h_count >= 10'd340) && (h_count < 10'd376));
assign vsync_n = !((v_count >= 9'd244) && (v_count < 9'd248));

assign irq4_start =
    pixel_tick && (h_count == 10'd0) && (v_count == 9'd230);
assign snapshot_start = line_end && (v_count == 9'd239);
assign frame_tick = line_end && (v_count == V_TOTAL - 9'd1);

// Build two lines ahead. Lines 0 and 1 are prepared during the final two
// vblank lines so line 0 is coherent when the epoch changes.
assign line_start =
    line_end && ((v_count < 9'd238) || (v_count >= 9'd260));
assign line_commit =
    line_end && ((v_count < 9'd239) || (v_count == V_TOTAL - 9'd1));
assign target_y =
    (v_count >= 9'd260) ? (v_count - 9'd260) : (v_count + 9'd2);
assign target_epoch =
    (v_count >= 9'd260) ? ~frame_epoch : frame_epoch;

always_ff @(posedge clk) begin
    if (reset) begin
        h_count <= 10'd0;
        v_count <= 9'd0;
        frame_epoch <= 1'b0;
    end else if (restore_commit) begin
        h_count <= restore_h_count;
        v_count <= restore_v_count;
        frame_epoch <= restore_frame_epoch;
    end else if (pixel_tick) begin
        if (h_count == H_TOTAL - 10'd1) begin
            h_count <= 10'd0;
            if (v_count == V_TOTAL - 9'd1) begin
                v_count <= 9'd0;
                frame_epoch <= ~frame_epoch;
            end else begin
                v_count <= v_count + 9'd1;
            end
        end else begin
            h_count <= h_count + 10'd1;
        end
    end
end

endmodule
