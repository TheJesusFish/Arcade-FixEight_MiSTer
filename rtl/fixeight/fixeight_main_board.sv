// TP-026 MC68000-facing board memory and peripheral ownership.
// Address inputs are byte addresses; RAM state streams use native words.
module fixeight_main_board (
    input  logic         clk,
    input  logic         reset,

    input  logic         cold_reset,
    input  logic         hs_identity_valid,
    input  logic [3:0]   hs_set_id,
    input  logic [31:0]  hs_seed_crc32,
    input  logic         hs_config_download,
    input  logic         hs_nvram_download,
    input  logic         hs_nvram_upload,
    input  logic         hs_ioctl_wr,
    input  logic         hs_ioctl_rd,
    input  logic [26:0]  hs_ioctl_addr,
    input  logic [7:0]   hs_ioctl_data,
    output logic [7:0]   hs_nvram_q,
    output logic         hs_nvram_wait,
    output logic         hs_dirty,
    output logic         hs_ready,
    output logic         hs_active,
    input  logic         hs_ss_active,
    input  logic         hs_restore_ready_known,
    input  logic         hs_restore_ready,
    output logic         hs_hold_request,
    input  logic         hs_hold_ack,

    input  logic         board_req,
    input  logic [23:0]  board_addr,
    input  logic         board_rw,
    input  logic [1:0]   board_byte_en,
    input  logic [15:0]  board_wdata,
    output logic [15:0]  board_rdata,
    output logic         board_ack,

    input  logic [15:0]  p1,
    input  logic [15:0]  p2,
    input  logic [15:0]  p3,
    input  logic [15:0]  system,
    input  logic [9:0]   h_count,
    input  logic [8:0]   v_count,
    output logic [7:0]   coin_control,
    output logic         v25_release,
    output logic         irq4_clear,
    input  logic         state_restore_commit,
    input  logic [7:0]   state_restore_coin_control,
    input  logic         state_restore_v25_release,

    input  logic [14:0]  sound_shared_addr,
    input  logic [7:0]   sound_shared_dout,
    input  logic         sound_shared_we,
    output logic [7:0]   sound_shared_din,

    input  logic         object_snapshot_start,
    input  logic [12:0]  gp_snapshot_addr,
    output logic [15:0]  gp_snapshot_data,
    input  logic [9:0]   gp_object_addr,
    output logic [15:0]  gp_object_data,
    output logic         gp_object_busy,
    output logic [127:0] gp_scrolls,
    output logic [7:0]   gp_scroll_flip,

    input  logic [10:0]  palette_snapshot_addr,
    output logic [15:0]  palette_snapshot_data,

    input  logic         line_start,
    input  logic         line_commit,
    input  logic         video_invalidate,
    input  logic [8:0]   target_y,
    input  logic         target_epoch,
    input  logic [8:0]   display_x,
    input  logic [8:0]   display_y,
    input  logic         display_epoch,
    output logic         text_line_ready,
    output logic         text_opaque,
    output logic [10:0]  text_color,
    output logic         text_renderer_busy,
    output logic         text_deadline_miss,
    output logic [15:0]  text_build_cycles,

    input  logic         state_hold,
    input  logic         ss_restore_enable,
    input  logic [63:0]  ss_data,
    input  logic [31:0]  ss_addr,
    input  logic [7:0]   ss_select,
    input  logic         ss_write,
    input  logic         ss_read,
    input  logic         ss_query,
    output logic [63:0]  ss_data_out,
    output logic         ss_ack,
    output logic         state_idle,
    output logic         state_held
);

logic rom_cs;
logic rom_read_cs;
logic wram_cs;
logic p1_cs;
logic p2_cs;
logic p3_cs;
logic system_cs;
logic coin_cs;
logic shared_cs;
logic gp_cs;
logic palette_cs;
logic text_vram_cs;
logic text_line_select_cs;
logic text_line_scroll_cs;
logic text_char_cs;
logic sound_reset_cs;
logic vcount_cs;
logic mapped_cs;
logic unmapped_cs;

fixeight_main_decode u_decode (
    .addr(board_addr),
    .bus_active(board_req),
    .rw(board_rw),
    .rom_cs,
    .rom_read_cs,
    .wram_cs,
    .p1_cs,
    .p2_cs,
    .p3_cs,
    .system_cs,
    .coin_cs,
    .shared_cs,
    .gp_cs,
    .palette_cs,
    .text_vram_cs,
    .text_line_select_cs,
    .text_line_scroll_cs,
    .text_char_cs,
    .sound_reset_cs,
    .vcount_cs,
    .mapped_cs,
    .unmapped_cs
);

logic board_seen;
wire board_accept = board_req && !board_seen && !state_hold;
wire text_cs =
    text_vram_cs || text_line_select_cs ||
    text_line_scroll_cs || text_char_cs;
wire local_cs =
    wram_cs || p1_cs || p2_cs || p3_cs || system_cs || coin_cs ||
    shared_cs || palette_cs || sound_reset_cs || vcount_cs;
logic local_ack;

always_ff @(posedge clk) begin
    local_ack <= 1'b0;
    if (reset) begin
        board_seen <= 1'b0;
        coin_control <= 8'h00;
        v25_release <= 1'b0;
    end else begin
        if (!board_req)
            board_seen <= 1'b0;
        else if (board_accept)
            board_seen <= 1'b1;

        if (board_accept && local_cs)
            local_ack <= 1'b1;

        if (state_restore_commit) begin
            coin_control <= state_restore_coin_control;
            v25_release <= state_restore_v25_release;
        end else begin
            if (board_accept && coin_cs && board_byte_en[0])
                coin_control <= board_wdata[7:0];
            if (board_accept && sound_reset_cs && board_byte_en[0])
                v25_release <= board_wdata[3];
        end
    end
end

wire [1:0] wram_cpu_we =
    {2{board_accept && wram_cs && !board_rw}} & board_byte_en;
wire [15:0] wram_cpu_q;
wire [12:0] wram_state_addr;
wire [15:0] wram_state_data;
wire [1:0] wram_state_we;
wire [15:0] wram_state_q;
wire [63:0] wram_ss_data_out;
wire wram_ss_ack;
wire [12:0] hs_ram_addr;
wire [1:0] hs_ram_we;
wire [15:0] hs_ram_data;

fixeight_highscore u_highscore (
    .clk, .reset(cold_reset), .cpu_reset(reset),
    .identity_valid(hs_identity_valid), .set_id(hs_set_id), .seed_crc32(hs_seed_crc32),
    .config_download(hs_config_download), .nvram_download(hs_nvram_download),
    .nvram_upload(hs_nvram_upload), .ioctl_wr(hs_ioctl_wr), .ioctl_rd(hs_ioctl_rd),
    .ioctl_addr(hs_ioctl_addr), .ioctl_data(hs_ioctl_data),
    .nvram_q(hs_nvram_q), .nvram_wait(hs_nvram_wait),
    .ss_active(hs_ss_active), .ss_restore_commit(state_restore_commit),
    .ss_ready_known(hs_restore_ready_known), .ss_ready_value(hs_restore_ready),
    .normal_addr(board_addr[13:1]), .normal_we(wram_cpu_we), .normal_data(board_wdata),
    .hold_request(hs_hold_request), .hold_ack(hs_hold_ack),
    .ram_addr(hs_ram_addr), .ram_we(hs_ram_we), .ram_data(hs_ram_data), .ram_q(wram_state_q),
    .dirty(hs_dirty), .ready(hs_ready), .active(hs_active), .config_valid()
);

fixeight_ss_ram_port #(
    .WIDTH(16), .ADDR_WIDTH(13), .WE_WIDTH(2),
    .SS_IDX(8'd2), .STREAM_WIDTH(2'd1)
) u_wram_state (
    .clk,
    .restore_enable(ss_restore_enable),
    .normal_we(hs_ram_we),
    .normal_addr(hs_ram_addr),
    .normal_data(hs_ram_data),
    .ram_we(wram_state_we),
    .ram_addr(wram_state_addr),
    .ram_data(wram_state_data),
    .ram_q(wram_state_q),
    .ss_data,
    .ss_addr,
    .ss_select,
    .ss_write,
    .ss_read,
    .ss_query,
    .ss_data_out(wram_ss_data_out),
    .ss_ack(wram_ss_ack)
);

jtframe_dual_ram16 #(.AW(13)) u_wram (
    .clk0(clk),
    .data0(board_wdata),
    .addr0(board_addr[13:1]),
    .we0(wram_cpu_we),
    .q0(wram_cpu_q),
    .clk1(clk),
    .data1(wram_state_data),
    .addr1(wram_state_addr),
    .we1(wram_state_we),
    .q1(wram_state_q)
);

wire shared_cpu_we =
    board_accept && shared_cs && !board_rw && board_byte_en[0];
wire [7:0] shared_cpu_q;
wire [14:0] shared_state_addr;
wire [7:0] shared_state_data;
wire shared_state_we;
wire [7:0] shared_state_q;
wire [63:0] shared_ss_data_out;
wire shared_ss_ack;

fixeight_shared_state_port #(
    .SS_IDX(8'd3)
) u_shared_state (
    .clk,
    .reset,
    .restore_enable(ss_restore_enable),
    .normal_we(sound_shared_we),
    .normal_addr(sound_shared_addr),
    .normal_data(sound_shared_dout),
    .ram_we(shared_state_we),
    .ram_addr(shared_state_addr),
    .ram_data(shared_state_data),
    .ram_q(shared_state_q),
    .ss_data,
    .ss_addr,
    .ss_select,
    .ss_write,
    .ss_read,
    .ss_query,
    .ss_data_out(shared_ss_data_out),
    .ss_ack(shared_ss_ack)
);

jtframe_dual_ram #(.DW(8), .AW(15)) u_shared (
    .clk0(clk),
    .data0(board_wdata[7:0]),
    .addr0(board_addr[15:1]),
    .we0(shared_cpu_we),
    .q0(shared_cpu_q),
    .clk1(clk),
    .data1(shared_state_data),
    .addr1(shared_state_addr),
    .we1(shared_state_we),
    .q1(shared_state_q)
);
assign sound_shared_din = shared_state_q;

wire [1:0] palette_cpu_we =
    {2{board_accept && palette_cs && !board_rw}} & board_byte_en;
wire [15:0] palette_cpu_q;
wire [10:0] palette_state_addr;
wire [15:0] palette_state_data;
wire [1:0] palette_state_we;
wire [15:0] palette_state_q;
wire [63:0] palette_ss_data_out;
wire palette_ss_ack;

fixeight_ss_ram_port #(
    .WIDTH(16), .ADDR_WIDTH(11), .WE_WIDTH(2),
    .SS_IDX(8'd4), .STREAM_WIDTH(2'd1)
) u_palette_state (
    .clk,
    .restore_enable(ss_restore_enable),
    .normal_we(2'b00),
    .normal_addr(palette_snapshot_addr),
    .normal_data(16'd0),
    .ram_we(palette_state_we),
    .ram_addr(palette_state_addr),
    .ram_data(palette_state_data),
    .ram_q(palette_state_q),
    .ss_data,
    .ss_addr,
    .ss_select,
    .ss_write,
    .ss_read,
    .ss_query,
    .ss_data_out(palette_ss_data_out),
    .ss_ack(palette_ss_ack)
);

jtframe_dual_ram16 #(.AW(11)) u_palette (
    .clk0(clk),
    .data0(board_wdata),
    .addr0(board_addr[11:1]),
    .we0(palette_cpu_we),
    .q0(palette_cpu_q),
    .clk1(clk),
    .data1(palette_state_data),
    .addr1(palette_state_addr),
    .we1(palette_state_we),
    .q1(palette_state_q)
);
assign palette_snapshot_data = palette_state_q;

wire gp_start = board_accept && gp_cs;
wire gp_busy;
wire gp_done;
wire [15:0] gp_dout;
wire [12:0] gp_ptr;
wire [7:0] gp_scroll_select;
wire gp_vram_write;
wire gp_scroll_write;
wire gp_obj_miss;
wire [63:0] gp_ss_data_out;
wire gp_ss_ack;

wire [9:0] vdp_status_sum = {1'b0, v_count} + 10'd15;
wire [9:0] vdp_status_v =
    (vdp_status_sum >= 10'd262) ?
    (vdp_status_sum - 10'd262) : vdp_status_sum;
wire vdp_hsync_n = ~((h_count > 10'd325) && (h_count < 10'd380));
wire vdp_vsync_n = ~((v_count >= 9'd232) && (v_count <= 9'd245));
wire vdp_fblank_n = vdp_hsync_n && vdp_vsync_n;
wire [15:0] vdp_count_flags =
    16'hff00 &
    (vdp_hsync_n ? 16'hffff : 16'h7fff) &
    (vdp_vsync_n ? 16'hffff : 16'hbfff) &
    (vdp_fblank_n ? 16'hffff : 16'hfeff);
wire [15:0] vcount_data =
    vdp_count_flags |
    ((vdp_status_v < 10'd256) ?
        {8'h00, vdp_status_v[7:0]} : 16'h00ff);
wire gp_status = vdp_status_v >= 10'd245;

fixeight_gp9001_cpu u_gp (
    .clk,
    .rst(reset),
    .start(gp_start),
    .rw(board_rw),
    .addr(board_addr[3:0]),
    .din(board_wdata),
    .we_mask(board_byte_en),
    .status_bit(gp_status),
    .busy(gp_busy),
    .done(gp_done),
    .dout(gp_dout),
    .irq_clear(irq4_clear),
    .scan_addr(gp_snapshot_addr),
    .scan_dout(gp_snapshot_data),
    .obj_buf_start(object_snapshot_start),
    .obj_scan_addr(gp_object_addr),
    .obj_scan_dout(gp_object_data),
    .obj_buf_busy(gp_object_busy),
    .obj_buf_miss(gp_obj_miss),
    .dbg_ptr(gp_ptr),
    .dbg_scroll_select(gp_scroll_select),
    .scrolls(gp_scrolls),
    .scroll_flip(gp_scroll_flip),
    .vram_write(gp_vram_write),
    .scroll_write(gp_scroll_write),
    .ss_hold(state_hold),
    .ss_restore_enable,
    .ss_data,
    .ss_addr,
    .ss_select,
    .ss_write,
    .ss_read,
    .ss_query,
    .ss_data_out(gp_ss_data_out),
    .ss_ack(gp_ss_ack)
);

logic [1:0] text_region;
always_comb begin
    text_region = 2'd0;
    if (text_line_select_cs)
        text_region = 2'd1;
    else if (text_line_scroll_cs)
        text_region = 2'd2;
    else if (text_char_cs)
        text_region = 2'd3;
end

wire text_req = board_accept && text_cs;
wire text_ack;
wire [15:0] text_rdata;
wire [63:0] text_ss_data_out;
wire text_ss_ack;
wire text_state_idle;
wire text_state_held;

fixeight_text u_text (
    .clk,
    .reset,
    .invalidate(video_invalidate),
    .cpu_req(text_req),
    .cpu_rw(board_rw),
    .cpu_region(text_region),
    .cpu_word_addr(board_addr[15:1]),
    .cpu_wdata(board_wdata),
    .cpu_byte_enable(board_byte_en),
    .cpu_ack(text_ack),
    .cpu_rdata(text_rdata),
    .line_start,
    .line_commit,
    .target_y,
    .target_epoch,
    .display_x,
    .display_y,
    .display_epoch,
    .line_ready(text_line_ready),
    .text_opaque,
    .text_color,
    .renderer_busy(text_renderer_busy),
    .deadline_miss(text_deadline_miss),
    .build_cycles(text_build_cycles),
    .state_hold,
    .ss_restore_enable,
    .ss_data,
    .ss_addr,
    .ss_select,
    .ss_write,
    .ss_read,
    .ss_query,
    .ss_data_out(text_ss_data_out),
    .ss_ack(text_ss_ack),
    .state_idle(text_state_idle),
    .state_held(text_state_held)
);

assign board_ack = local_ack || gp_done || text_ack;

always_comb begin
    board_rdata = 16'hffff;
    if (wram_cs)
        board_rdata = wram_cpu_q;
    else if (p1_cs)
        board_rdata = p1;
    else if (p2_cs)
        board_rdata = p2;
    else if (p3_cs)
        board_rdata = p3;
    else if (system_cs)
        board_rdata = system;
    else if (shared_cs)
        board_rdata = {8'hff, shared_cpu_q};
    else if (gp_cs)
        board_rdata = gp_dout;
    else if (palette_cs)
        board_rdata = palette_cpu_q;
    else if (text_cs)
        board_rdata = text_rdata;
    else if (vcount_cs)
        board_rdata = vcount_data;
end

assign ss_ack =
    wram_ss_ack || shared_ss_ack || palette_ss_ack ||
    gp_ss_ack || text_ss_ack;
assign ss_data_out =
    wram_ss_ack ? wram_ss_data_out :
    shared_ss_ack ? shared_ss_data_out :
    palette_ss_ack ? palette_ss_data_out :
    gp_ss_ack ? gp_ss_data_out :
    text_ss_ack ? text_ss_data_out : 64'd0;
assign state_idle =
    (!board_req || state_hold) &&
    !gp_busy && text_state_idle && !gp_object_busy;
assign state_held = state_hold && state_idle && text_state_held;

endmodule
