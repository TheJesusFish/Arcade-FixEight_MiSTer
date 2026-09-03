// Freeze CPU-mutating GP9001 tile and scroll state during vblank so every
// visible frame is rendered from one coherent state, matching MAME's update.
module fixeight_gp9001_snapshot (
    input              clk,
    input              rst,
    input              snapshot_start,

    output     [12:0]  source_addr,
    input      [15:0]  source_data,
    input      [127:0] source_scrolls,
    input      [7:0]   source_scroll_flip,

    input      [12:0]  display_addr,
    output     [15:0]  display_data,
    input      [9:0]   object_addr,
    output     [15:0]  object_data,
    output reg [127:0] display_scrolls,
    output reg [7:0]   display_scroll_flip,

    output             copy_busy,
    output reg         copy_miss
);

reg        copy_active;
reg        copy_valid;
reg        copy_final_pending;
reg [12:0] copy_read_addr;
reg [12:0] copy_write_addr;
reg        display_snapshot_valid;
reg        object_copy_active;
reg        object_copy_valid;
reg        object_copy_final_pending;
reg        object_copy_source_valid;
reg [9:0]  object_copy_read_addr;
reg [9:0]  object_copy_write_addr;
wire [12:0] display_ram_addr = object_copy_active ?
                                {3'b110, object_copy_read_addr} :
                                display_addr;
wire [15:0] display_ram_data;
wire [15:0] object_copy_data = object_copy_source_valid ?
                               display_ram_data : 16'h0000;

assign source_addr = copy_read_addr;
assign display_data = display_ram_data;
assign copy_busy = copy_active || object_copy_active;

jtframe_dual_ram16 #(.AW(13)) u_display_vram (
    .clk0  (clk),
    .data0 (source_data),
    .addr0 (copy_write_addr),
    .we0   ({2{copy_active && copy_valid}}),
    .q0    (),
    .clk1  (clk),
    .data1 (16'h0000),
    .addr1 (display_ram_addr),
    .we1   (2'b00),
    .q1    (display_ram_data)
);

// MAME presents current tile/scroll state with object RAM from the preceding
// display snapshot. Preserve that 1K-word region while the current 8K-word
// source copy runs in parallel; the source writer cannot reach 0x1800 before
// this shorter copy has completed.
jtframe_dual_ram16 #(.AW(10)) u_object_history (
    .clk0  (clk),
    .data0 (object_copy_data),
    .addr0 (object_copy_write_addr),
    .we0   ({2{object_copy_active && object_copy_valid}}),
    .q0    (),
    .clk1  (clk),
    .data1 (16'h0000),
    .addr1 (object_addr),
    .we1   (2'b00),
    .q1    (object_data)
);

always @(posedge clk or posedge rst) begin
    if (rst) begin
        copy_active <= 1'b0;
        copy_valid <= 1'b0;
        copy_final_pending <= 1'b0;
        copy_read_addr <= 13'd0;
        copy_write_addr <= 13'd0;
        display_snapshot_valid <= 1'b0;
        object_copy_active <= 1'b0;
        object_copy_valid <= 1'b0;
        object_copy_final_pending <= 1'b0;
        object_copy_source_valid <= 1'b0;
        object_copy_read_addr <= 10'd0;
        object_copy_write_addr <= 10'd0;
        display_scrolls <= 128'd0;
        display_scroll_flip <= 8'd0;
        copy_miss <= 1'b0;
    end else begin
        copy_miss <= 1'b0;

        if (snapshot_start) begin
            if (!copy_active && !object_copy_active) begin
                copy_active <= 1'b1;
                copy_valid <= 1'b0;
                copy_final_pending <= 1'b0;
                copy_read_addr <= 13'd0;
                copy_write_addr <= 13'd0;
                object_copy_active <= 1'b1;
                object_copy_valid <= 1'b0;
                object_copy_final_pending <= 1'b0;
                object_copy_source_valid <= display_snapshot_valid;
                object_copy_read_addr <= 10'd0;
                object_copy_write_addr <= 10'd0;
                display_scrolls <= source_scrolls;
                display_scroll_flip <= source_scroll_flip;
            end else begin
                copy_miss <= 1'b1;
            end
        end else if (copy_active) begin
            if (!copy_valid) begin
                copy_valid <= 1'b1;
                copy_write_addr <= copy_read_addr;
                copy_read_addr <= copy_read_addr + 1'b1;
            end else if (copy_final_pending) begin
                copy_active <= 1'b0;
                copy_valid <= 1'b0;
                copy_final_pending <= 1'b0;
                display_snapshot_valid <= 1'b1;
            end else begin
                copy_write_addr <= copy_read_addr;
                if (copy_read_addr == 13'h1fff)
                    copy_final_pending <= 1'b1;
                else
                    copy_read_addr <= copy_read_addr + 1'b1;
            end
        end

        if (!snapshot_start && object_copy_active) begin
            if (!object_copy_valid) begin
                object_copy_valid <= 1'b1;
                object_copy_write_addr <= object_copy_read_addr;
                object_copy_read_addr <= object_copy_read_addr + 1'b1;
            end else if (object_copy_final_pending) begin
                object_copy_active <= 1'b0;
                object_copy_valid <= 1'b0;
                object_copy_final_pending <= 1'b0;
            end else begin
                object_copy_write_addr <= object_copy_read_addr;
                if (object_copy_read_addr == 10'h3ff)
                    object_copy_final_pending <= 1'b1;
                else
                    object_copy_read_addr <= object_copy_read_addr + 1'b1;
            end
        end
    end
end

endmodule
