// MAME renders each frame with one coherent palette state. TP-026 updates
// palette RAM after the line-230 IRQ, so copy it during vblank and expose the
// completed snapshot to the following visible frame.
module fixeight_palette_snapshot (
    input              clk,
    input              rst,
    input              snapshot_start,
    output     [10:0]  source_addr,
    input      [15:0]  source_data,
    input      [10:0]  display_addr,
    output     [15:0]  display_data,
    output             copy_busy
);

reg        copy_active;
reg        copy_valid;
reg        copy_final_pending;
reg [10:0] copy_read_addr;
reg [10:0] copy_write_addr;

assign source_addr = copy_read_addr;
assign copy_busy = copy_active;

jtframe_dual_ram16 #(.AW(11)) u_display_palette (
    .clk0  (clk),
    .data0 (source_data),
    .addr0 (copy_write_addr),
    .we0   ({2{copy_active && copy_valid}}),
    .q0    (),
    .clk1  (clk),
    .data1 (16'h0000),
    .addr1 (display_addr),
    .we1   (2'b00),
    .q1    (display_data)
);

always @(posedge clk or posedge rst) begin
    if (rst) begin
        copy_active <= 1'b0;
        copy_valid <= 1'b0;
        copy_final_pending <= 1'b0;
        copy_read_addr <= 11'd0;
        copy_write_addr <= 11'd0;
    end else if (snapshot_start && !copy_active) begin
        copy_active <= 1'b1;
        copy_valid <= 1'b0;
        copy_final_pending <= 1'b0;
        copy_read_addr <= 11'd0;
        copy_write_addr <= 11'd0;
    end else if (copy_active) begin
        if (!copy_valid) begin
            copy_valid <= 1'b1;
            copy_write_addr <= copy_read_addr;
            copy_read_addr <= copy_read_addr + 1'b1;
        end else if (copy_final_pending) begin
            copy_active <= 1'b0;
            copy_valid <= 1'b0;
            copy_final_pending <= 1'b0;
        end else begin
            copy_write_addr <= copy_read_addr;
            if (copy_read_addr == 11'h7ff)
                copy_final_pending <= 1'b1;
            else
                copy_read_addr <= copy_read_addr + 1'b1;
        end
    end
end

endmodule
