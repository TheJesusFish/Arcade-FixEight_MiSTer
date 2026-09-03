// SPDX-License-Identifier: BSD-3-Clause
// TP-026 dynamic 8x8 text layer with MAME 0.288 line select/scroll semantics.

module fixeight_text #(
    parameter [7:0] SS_VRAM_IDX       = 8'd6,
    parameter [7:0] SS_LINE_SELECT_IDX = 8'd7,
    parameter [7:0] SS_LINE_SCROLL_IDX = 8'd8,
    parameter [7:0] SS_CHAR_IDX        = 8'd9
) (
    input  logic        clk,
    input  logic        reset,
    input  logic        invalidate,

    input  logic        cpu_req,
    input  logic        cpu_rw,
    input  logic [1:0]  cpu_region,
    input  logic [14:0] cpu_word_addr,
    input  logic [15:0] cpu_wdata,
    input  logic [1:0]  cpu_byte_enable,
    output logic        cpu_ack,
    output logic [15:0] cpu_rdata,

    input  logic        line_start,
    input  logic        line_commit,
    input  logic [8:0]  target_y,
    input  logic        target_epoch,
    input  logic [8:0]  display_x,
    input  logic [8:0]  display_y,
    input  logic        display_epoch,
    output logic        line_ready,
    output logic        text_opaque,
    output logic [10:0] text_color,
    output logic        renderer_busy,
    output logic        deadline_miss,
    output logic [15:0] build_cycles,

    input  logic        state_hold,
    input  logic        ss_restore_enable,
    input  logic [63:0] ss_data,
    input  logic [31:0] ss_addr,
    input  logic [7:0]  ss_select,
    input  logic        ss_write,
    input  logic        ss_read,
    input  logic        ss_query,
    output logic [63:0] ss_data_out,
    output logic        ss_ack,
    output logic        state_idle,
    output logic        state_held
);

localparam logic [1:0] REGION_VRAM        = 2'd0;
localparam logic [1:0] REGION_LINE_SELECT = 2'd1;
localparam logic [1:0] REGION_LINE_SCROLL = 2'd2;
localparam logic [1:0] REGION_CHAR        = 2'd3;

localparam logic CPU_IDLE = 1'b0;
localparam logic CPU_WAIT = 1'b1;

logic cpu_state;
logic cpu_req_seen;
logic [1:0] cpu_region_latched;

wire cpu_accept =
    (cpu_state == CPU_IDLE) && cpu_req && !cpu_req_seen && !state_hold;
wire [1:0] cpu_vram_we =
    {2{cpu_accept && !cpu_rw && (cpu_region == REGION_VRAM)}} &
    cpu_byte_enable;
wire [1:0] cpu_line_select_we =
    {2{cpu_accept && !cpu_rw && (cpu_region == REGION_LINE_SELECT)}} &
    cpu_byte_enable;
wire [1:0] cpu_line_scroll_we =
    {2{cpu_accept && !cpu_rw && (cpu_region == REGION_LINE_SCROLL)}} &
    cpu_byte_enable;
wire [1:0] cpu_char_we =
    {2{cpu_accept && !cpu_rw && (cpu_region == REGION_CHAR)}} &
    cpu_byte_enable;

logic [15:0] cpu_vram_q;
logic [15:0] cpu_line_select_q;
logic [15:0] cpu_line_scroll_q;
logic [15:0] cpu_char_q;

always_ff @(posedge clk) begin
    cpu_ack <= 1'b0;
    if (reset || invalidate) begin
        cpu_state <= CPU_IDLE;
        cpu_req_seen <= 1'b0;
        cpu_region_latched <= REGION_VRAM;
        cpu_rdata <= 16'h0000;
    end else begin
        if (!cpu_req)
            cpu_req_seen <= 1'b0;

        case (cpu_state)
            CPU_IDLE: begin
                if (cpu_accept) begin
                    cpu_req_seen <= 1'b1;
                    cpu_region_latched <= cpu_region;
                    cpu_state <= CPU_WAIT;
                end
            end

            default: begin
                case (cpu_region_latched)
                    REGION_VRAM: cpu_rdata <= cpu_vram_q;
                    REGION_LINE_SELECT: cpu_rdata <= cpu_line_select_q;
                    REGION_LINE_SCROLL: cpu_rdata <= cpu_line_scroll_q;
                    default: cpu_rdata <= cpu_char_q;
                endcase
                cpu_ack <= 1'b1;
                cpu_state <= CPU_IDLE;
            end
        endcase
    end
end

localparam logic [3:0] R_IDLE              = 4'd0;
localparam logic [3:0] R_LINE_ZERO_WAIT    = 4'd1;
localparam logic [3:0] R_LINE_ZERO_CAPTURE = 4'd2;
localparam logic [3:0] R_LINE_WAIT         = 4'd3;
localparam logic [3:0] R_LINE_CAPTURE      = 4'd4;
localparam logic [3:0] R_SCROLL_WAIT       = 4'd5;
localparam logic [3:0] R_SCROLL_CAPTURE    = 4'd6;
localparam logic [3:0] R_ATTR_WAIT         = 4'd7;
localparam logic [3:0] R_ATTR_CAPTURE      = 4'd8;
localparam logic [3:0] R_CHAR_WAIT         = 4'd9;
localparam logic [3:0] R_CHAR_CAPTURE      = 4'd10;
localparam logic [3:0] R_STORE             = 4'd11;
localparam logic [3:0] R_DONE              = 4'd12;
localparam logic [3:0] R_PIXEL_SETUP       = 4'd13;

logic [3:0] render_state;
logic [8:0] build_x;
logic [8:0] target_y_latched;
logic target_epoch_latched;
logic [15:0] line_zero_latched;
logic [15:0] line_select_latched;
logic [15:0] line_scroll_latched;
logic [15:0] attr_latched;
logic [15:0] char_latched;
logic [15:0] cycle_counter;
logic active_bank;
logic build_bank;
logic build_pending;
logic [8:0] scan_y;
logic scan_epoch;
logic scan_valid;

logic [11:0] vram_scan_addr;
logic [7:0] line_select_scan_addr;
logic [7:0] line_scroll_scan_addr;
logic [14:0] char_scan_addr;
logic [15:0] vram_scan_q;
logic [15:0] line_select_scan_q;
logic [15:0] line_scroll_scan_q;
logic [15:0] char_scan_q;

wire text_normal_x = line_zero_latched[15];
wire [8:0] source_x_normal =
    build_x + line_scroll_latched[8:0] + 9'd43;
wire [8:0] source_x_flipped =
    9'd319 + line_scroll_latched[8:0] - build_x;
wire [8:0] source_x =
    text_normal_x ? source_x_normal : source_x_flipped;
// MAME programs per-destination-line scrolly as lineselect[y] - y.
// Its FixEight tilemap Y offset is already absorbed by the visible-area
// transform, so the low byte of lineselect is the effective character row.
wire [7:0] source_y = line_select_latched[7:0];
wire [10:0] tile_index = {source_y[7:3], source_x[8:3]};
wire [9:0] character_code = attr_latched[9:0];
wire [14:0] character_word_addr =
    {character_code, 5'b00000} +
    {10'd0, source_y[2:0], 2'b00} +
    {13'd0, source_x[2:1]};
wire [3:0] character_pen =
    source_x[0] ? char_latched[3:0] : char_latched[7:4];
wire [10:0] character_color = {
    1'b1, attr_latched[15:10], character_pen
};
wire [11:0] line_write_data = {
    character_pen != 4'h0, character_color
};

logic [11:0] line_bank0_build_q;
logic [11:0] line_bank1_build_q;
logic [11:0] line_bank0_scan_q;
logic [11:0] line_bank1_scan_q;
wire [11:0] scan_word =
    active_bank ? line_bank1_scan_q : line_bank0_scan_q;
wire line_write = render_state == R_STORE;

jtframe_dual_ram #(.DW(12), .AW(9)) u_line_bank0 (
    .clk0  (clk),
    .data0 (line_write_data),
    .addr0 (build_x),
    .we0   (line_write && !build_bank),
    .q0    (line_bank0_build_q),
    .clk1  (clk),
    .data1 (12'd0),
    .addr1 (display_x),
    .we1   (1'b0),
    .q1    (line_bank0_scan_q)
);

jtframe_dual_ram #(.DW(12), .AW(9)) u_line_bank1 (
    .clk0  (clk),
    .data0 (line_write_data),
    .addr0 (build_x),
    .we0   (line_write && build_bank),
    .q0    (line_bank1_build_q),
    .clk1  (clk),
    .data1 (12'd0),
    .addr1 (display_x),
    .we1   (1'b0),
    .q1    (line_bank1_scan_q)
);

always_ff @(posedge clk) begin
    deadline_miss <= 1'b0;
    if (reset || invalidate) begin
        render_state <= R_IDLE;
        build_x <= 9'd0;
        target_y_latched <= 9'd0;
        target_epoch_latched <= 1'b0;
        line_zero_latched <= 16'h8000;
        line_select_latched <= 16'h0000;
        line_scroll_latched <= 16'h0000;
        attr_latched <= 16'h0000;
        char_latched <= 16'h0000;
        cycle_counter <= 16'd0;
        build_cycles <= 16'd0;
        active_bank <= 1'b0;
        build_bank <= 1'b1;
        build_pending <= 1'b0;
        scan_y <= 9'd0;
        scan_epoch <= 1'b0;
        scan_valid <= 1'b0;
        vram_scan_addr <= 12'd0;
        line_select_scan_addr <= 8'd0;
        line_scroll_scan_addr <= 8'd0;
        char_scan_addr <= 15'd0;
    end else begin
        if (line_commit && build_pending) begin
            active_bank <= build_bank;
            scan_y <= target_y_latched;
            scan_epoch <= target_epoch_latched;
            scan_valid <= 1'b1;
            build_pending <= 1'b0;
        end else if (line_commit && scan_valid) begin
            deadline_miss <= 1'b1;
        end

        if (line_start && render_state != R_IDLE)
            deadline_miss <= 1'b1;

        if (render_state != R_IDLE)
            cycle_counter <= cycle_counter + 16'd1;

        // A hold blocks a new scanline but must let an in-flight renderer
        // reach R_IDLE; otherwise the state owner can never acknowledge.
        if (!state_hold || (render_state != R_IDLE)) begin
            case (render_state)
                R_IDLE: begin
                    if (line_start && !state_hold) begin
                        target_y_latched <= target_y;
                        target_epoch_latched <= target_epoch;
                        build_bank <=
                            (line_commit && build_pending) ?
                            active_bank : ~active_bank;
                        build_x <= 9'd0;
                        cycle_counter <= 16'd0;
                        line_select_scan_addr <= 8'd0;
                        render_state <= R_LINE_ZERO_WAIT;
                    end
                end

                R_LINE_ZERO_WAIT:
                    render_state <= R_LINE_ZERO_CAPTURE;

                R_LINE_ZERO_CAPTURE: begin
                    line_zero_latched <= line_select_scan_q;
                    line_select_scan_addr <= target_y_latched[7:0];
                    render_state <= R_LINE_WAIT;
                end

                R_LINE_WAIT:
                    render_state <= R_LINE_CAPTURE;

                R_LINE_CAPTURE: begin
                    line_select_latched <= line_select_scan_q;
                    line_scroll_scan_addr <= target_y_latched[7:0];
                    render_state <= R_SCROLL_WAIT;
                end

                R_SCROLL_WAIT:
                    render_state <= R_SCROLL_CAPTURE;

                R_SCROLL_CAPTURE: begin
                    line_scroll_latched <= line_scroll_scan_q;
                    render_state <= R_PIXEL_SETUP;
                end

                R_PIXEL_SETUP: begin
                    vram_scan_addr <= {1'b0, tile_index};
                    render_state <= R_ATTR_WAIT;
                end

                R_ATTR_WAIT:
                    render_state <= R_ATTR_CAPTURE;

                R_ATTR_CAPTURE: begin
                    attr_latched <= vram_scan_q;
                    char_scan_addr <=
                        {vram_scan_q[9:0], 5'b00000} +
                        {10'd0, source_y[2:0], 2'b00} +
                        {13'd0, source_x[2:1]};
                    render_state <= R_CHAR_WAIT;
                end

                R_CHAR_WAIT:
                    render_state <= R_CHAR_CAPTURE;

                R_CHAR_CAPTURE: begin
                    char_latched <= char_scan_q;
                    render_state <= R_STORE;
                end

                R_STORE: begin
                    if (build_x == 9'd319) begin
                        render_state <= R_DONE;
                    end else begin
                        build_x <= build_x + 9'd1;
                        render_state <= R_PIXEL_SETUP;
                    end
                end

                default: begin
                    build_pending <= 1'b1;
                    build_cycles <= cycle_counter;
                    render_state <= R_IDLE;
                end
            endcase
        end
    end
end

assign renderer_busy = render_state != R_IDLE;
assign line_ready =
    scan_valid && (scan_y == display_y) && (scan_epoch == display_epoch);
assign text_opaque = line_ready && scan_word[11];
assign text_color = line_ready ? scan_word[10:0] : 11'd0;

wire [11:0] ss_vram_addr;
wire [15:0] ss_vram_data;
wire [1:0] ss_vram_we;
wire [7:0] ss_line_select_addr;
wire [15:0] ss_line_select_data;
wire [1:0] ss_line_select_we;
wire [7:0] ss_line_scroll_addr;
wire [15:0] ss_line_scroll_data;
wire [1:0] ss_line_scroll_we;
wire [14:0] ss_char_addr;
wire [15:0] ss_char_data;
wire [1:0] ss_char_we;
wire [63:0] ss_vram_data_out;
wire [63:0] ss_line_select_data_out;
wire [63:0] ss_line_scroll_data_out;
wire [63:0] ss_char_data_out;
wire ss_vram_ack;
wire ss_line_select_ack;
wire ss_line_scroll_ack;
wire ss_char_ack;

fixeight_ss_ram_port #(
    .WIDTH(16), .ADDR_WIDTH(12), .WE_WIDTH(2),
    .SS_IDX(SS_VRAM_IDX), .STREAM_WIDTH(2'd1)
) u_vram_state (
    .clk, .restore_enable(ss_restore_enable),
    .normal_we(2'b00), .normal_addr(vram_scan_addr),
    .normal_data(16'd0), .ram_we(ss_vram_we), .ram_addr(ss_vram_addr),
    .ram_data(ss_vram_data), .ram_q(vram_scan_q),
    .ss_data, .ss_addr, .ss_select, .ss_write, .ss_read, .ss_query,
    .ss_data_out(ss_vram_data_out), .ss_ack(ss_vram_ack)
);

fixeight_ss_ram_port #(
    .WIDTH(16), .ADDR_WIDTH(8), .WE_WIDTH(2),
    .SS_IDX(SS_LINE_SELECT_IDX), .STREAM_WIDTH(2'd1)
) u_line_select_state (
    .clk, .restore_enable(ss_restore_enable),
    .normal_we(2'b00), .normal_addr(line_select_scan_addr),
    .normal_data(16'd0), .ram_we(ss_line_select_we),
    .ram_addr(ss_line_select_addr), .ram_data(ss_line_select_data),
    .ram_q(line_select_scan_q),
    .ss_data, .ss_addr, .ss_select, .ss_write, .ss_read, .ss_query,
    .ss_data_out(ss_line_select_data_out), .ss_ack(ss_line_select_ack)
);

fixeight_ss_ram_port #(
    .WIDTH(16), .ADDR_WIDTH(8), .WE_WIDTH(2),
    .SS_IDX(SS_LINE_SCROLL_IDX), .STREAM_WIDTH(2'd1)
) u_line_scroll_state (
    .clk, .restore_enable(ss_restore_enable),
    .normal_we(2'b00), .normal_addr(line_scroll_scan_addr),
    .normal_data(16'd0), .ram_we(ss_line_scroll_we),
    .ram_addr(ss_line_scroll_addr), .ram_data(ss_line_scroll_data),
    .ram_q(line_scroll_scan_q),
    .ss_data, .ss_addr, .ss_select, .ss_write, .ss_read, .ss_query,
    .ss_data_out(ss_line_scroll_data_out), .ss_ack(ss_line_scroll_ack)
);

fixeight_ss_ram_port #(
    .WIDTH(16), .ADDR_WIDTH(15), .WE_WIDTH(2),
    .SS_IDX(SS_CHAR_IDX), .STREAM_WIDTH(2'd1)
) u_char_state (
    .clk, .restore_enable(ss_restore_enable),
    .normal_we(2'b00), .normal_addr(char_scan_addr),
    .normal_data(16'd0), .ram_we(ss_char_we), .ram_addr(ss_char_addr),
    .ram_data(ss_char_data), .ram_q(char_scan_q),
    .ss_data, .ss_addr, .ss_select, .ss_write, .ss_read, .ss_query,
    .ss_data_out(ss_char_data_out), .ss_ack(ss_char_ack)
);

jtframe_dual_ram16 #(.AW(12)) u_vram (
    .clk0(clk), .data0(cpu_wdata), .addr0(cpu_word_addr[11:0]),
    .we0(cpu_vram_we), .q0(cpu_vram_q),
    .clk1(clk), .data1(ss_vram_data), .addr1(ss_vram_addr),
    .we1(ss_vram_we), .q1(vram_scan_q)
);

jtframe_dual_ram16 #(.AW(8)) u_line_select (
    .clk0(clk), .data0(cpu_wdata), .addr0(cpu_word_addr[7:0]),
    .we0(cpu_line_select_we), .q0(cpu_line_select_q),
    .clk1(clk), .data1(ss_line_select_data), .addr1(ss_line_select_addr),
    .we1(ss_line_select_we), .q1(line_select_scan_q)
);

jtframe_dual_ram16 #(.AW(8)) u_line_scroll (
    .clk0(clk), .data0(cpu_wdata), .addr0(cpu_word_addr[7:0]),
    .we0(cpu_line_scroll_we), .q0(cpu_line_scroll_q),
    .clk1(clk), .data1(ss_line_scroll_data), .addr1(ss_line_scroll_addr),
    .we1(ss_line_scroll_we), .q1(line_scroll_scan_q)
);

jtframe_dual_ram16 #(.AW(15)) u_char (
    .clk0(clk), .data0(cpu_wdata), .addr0(cpu_word_addr),
    .we0(cpu_char_we), .q0(cpu_char_q),
    .clk1(clk), .data1(ss_char_data), .addr1(ss_char_addr),
    .we1(ss_char_we), .q1(char_scan_q)
);

assign ss_ack =
    ss_vram_ack || ss_line_select_ack || ss_line_scroll_ack || ss_char_ack;
assign ss_data_out =
    ss_vram_ack ? ss_vram_data_out :
    ss_line_select_ack ? ss_line_select_data_out :
    ss_line_scroll_ack ? ss_line_scroll_data_out :
    ss_char_ack ? ss_char_data_out : 64'd0;
assign state_idle = (cpu_state == CPU_IDLE) && (render_state == R_IDLE);
assign state_held = state_hold && state_idle;

endmodule
