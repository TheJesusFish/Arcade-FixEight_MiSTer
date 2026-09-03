// TP-026 functional composition in one 94.5 MHz clock domain.
// MiSTer shell, loader, SDRAM arbitration, and state stream transport remain
// outside this board core.
module fixeight_core (
    input  logic         clk,
    input  logic         clock_reset,
    input  logic         reset,
    input  logic         halt_n,
    input  logic         cpu_run,

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

    input  logic [15:0]  p1,
    input  logic [15:0]  p2,
    input  logic [15:0]  p3,
    input  logic [15:0]  system,
    input  logic         ym_enable,
    input  logic         oki_enable,
    input  logic [1:0]   fx_level,
    input  logic         hide_output,

    output logic         main_rom_req,
    output logic [18:0]  main_rom_word_addr,
    input  logic [15:0]  main_rom_data,
    input  logic         main_rom_ack,

    output logic         tile_gfx_req,
    output logic [21:0]  tile_gfx_addr,
    input  logic [15:0]  tile_gfx_data,
    input  logic         tile_gfx_ack,
    output logic         object_gfx_req,
    output logic [21:0]  object_gfx_addr,
    input  logic [15:0]  object_gfx_data,
    input  logic         object_gfx_ack,

    output logic [17:0]  oki_rom_addr,
    input  logic [7:0]   oki_rom_data,
    input  logic         oki_rom_ack,

    input  logic         eeprom_seed_we,
    input  logic [5:0]   eeprom_seed_addr,
    input  logic [15:0]  eeprom_seed_data,

    input  logic         state_hold,
    input  logic         state_irq7,
    input  logic         state_override,
    input  logic         state_reset,
    input  logic [63:0]  state_reset_vector,
    input  logic         ss_restore_enable,
    input  logic         ss_restore_commit,
    input  logic         ss_restore_irq4,
    input  logic [7:0]   ss_restore_coin_control,
    input  logic         ss_restore_v25_release,
    input  logic [9:0]   ss_restore_h_count,
    input  logic [8:0]   ss_restore_v_count,
    input  logic         ss_restore_frame_epoch,
    input  logic [63:0]  ss_data,
    input  logic [31:0]  ss_addr,
    input  logic [7:0]   ss_select,
    input  logic         ss_write,
    input  logic         ss_read,
    input  logic         ss_query,
    output logic [63:0]  ss_data_out,
    output logic         ss_ack,
    output logic         state_idle,
    output logic         state_held,

    output logic [7:0]   red,
    output logic [7:0]   green,
    output logic [7:0]   blue,
    output logic         hsync_n,
    output logic         vsync_n,
    output logic         hblank,
    output logic         vblank,
    output logic         pixel_tick,
    output logic signed [15:0] audio_mono,
    output logic         audio_sample,

    output logic [9:0]   debug_h_count,
    output logic [8:0]   debug_v_count,
    output logic         debug_irq4,
    output logic [23:0]  debug_main_pc,
    output logic         debug_cpu_bus_active,
    output logic         debug_cpu_rw,
    output logic         debug_cpu_ack,
    output logic         debug_cpu_iack,
    output logic         debug_cpu_lds_n,
    output logic [2:0]   debug_cpu_fc,
    output logic [15:0]  debug_cpu_dout,
    output logic [7:0]   debug_coin_control,
    output logic         debug_v25_release,
    output logic         debug_frame_epoch,
    output logic         debug_frame_tick,
    output logic [19:0]  debug_v25_pc,
    output logic [1:0]   debug_video_busy,
    output logic [2:0]   debug_video_miss
);

logic ce_vdp;
logic ce_cpu;
logic ce_pixel;
logic ce_opm;
logic ce_oki;

fixeight_clock_enables u_enables (
    .clk,
    // JTFrame releases game reset by counting pxl_cen pulses. Keep the
    // enable divider alive during game reset so that handshake cannot
    // deadlock; only the independent cold reset may stop it.
    .reset(clock_reset),
    .ce_vdp_27m(ce_vdp),
    .ce_cpu_16m(ce_cpu),
    .ce_pixel_6m75(ce_pixel),
    .ce_ikaopm_3m375(ce_opm),
    .ce_oki_1m(ce_oki)
);

logic [9:0] h_count;
logic [8:0] v_count;
logic frame_epoch;
logic timing_active;
logic timing_hsync_n;
logic timing_vsync_n;
logic timing_hblank;
logic timing_vblank;
logic irq4_start;
logic snapshot_start;
logic frame_tick;
logic line_start;
logic line_commit;
logic [8:0] target_y;
logic target_epoch;

fixeight_video_timing u_timing (
    .clk,
    .reset,
    .ce_pixel,
    .hold(state_hold),
    .restore_commit(ss_restore_commit),
    .restore_h_count(ss_restore_h_count),
    .restore_v_count(ss_restore_v_count),
    .restore_frame_epoch(ss_restore_frame_epoch),
    .h_count,
    .v_count,
    .frame_epoch,
    .pixel_tick,
    .hblank(timing_hblank),
    .vblank(timing_vblank),
    .active(timing_active),
    .hsync_n(timing_hsync_n),
    .vsync_n(timing_vsync_n),
    .irq4_start,
    .snapshot_start,
    .frame_tick,
    .line_start,
    .line_commit,
    .target_y,
    .target_epoch
);

assign debug_h_count = h_count;
assign debug_v_count = v_count;

logic board_req;
logic [23:0] board_addr;
logic board_rw;
logic [1:0] board_byte_en;
logic [15:0] board_wdata;
logic [15:0] board_rdata;
logic board_ack;
logic irq4_clear;
logic irq4_pending;
logic cpu_bus_active;
logic cpu_ack;
logic cpu_iack;
logic [2:0] cpu_fc;
logic [15:0] cpu_din_debug;
logic [15:0] cpu_dout_debug;
logic [31:0] main_bus_count;
logic [31:0] main_rom_count;
logic [31:0] main_board_count;
logic [31:0] main_unmapped_count;
logic [23:0] main_last_fetch;

logic hs_hold_request;
logic hs_hold_ack;
wire hs_cpu_run = !hs_hold_request || (cpu_bus_active && !hs_hold_ack);
always_ff @(posedge clk) begin
    if (reset || !hs_hold_request || hs_ss_active)
        hs_hold_ack <= 1'b0;
    else if (!cpu_bus_active)
        hs_hold_ack <= 1'b1;
end

fixeight_main_cpu u_main_cpu (
    .clk,
    .reset,
    .halt_n,
    .cpu_run(cpu_run && !state_hold && hs_cpu_run),
    .rom_req(main_rom_req),
    .rom_word_addr(main_rom_word_addr),
    .rom_rdata(main_rom_data),
    .rom_ack(main_rom_ack),
    .board_req,
    .board_addr,
    .board_rw,
    .board_byte_en,
    .board_wdata,
    .board_rdata,
    .board_ack,
    .irq4_start,
    .irq4_clear,
    .state_irq7,
    .state_override,
    .state_reset,
    .state_reset_vector,
    .state_restore_commit(ss_restore_commit),
    .state_restore_irq4(ss_restore_irq4),
    .irq4_pending,
    .cpu_bus_active,
    .cpu_ack,
    .cpu_iack,
    .cpu_lds_n_debug(debug_cpu_lds_n),
    .cpu_fc,
    .cpu_addr_debug(debug_main_pc),
    .cpu_din_debug,
    .cpu_dout_debug,
    .bus_count(main_bus_count),
    .rom_read_count(main_rom_count),
    .board_access_count(main_board_count),
    .unmapped_count(main_unmapped_count),
    .last_program_fetch(main_last_fetch)
);
assign debug_irq4 = irq4_pending;
assign debug_cpu_bus_active = cpu_bus_active;
assign debug_cpu_rw = board_rw;
assign debug_cpu_ack = cpu_ack;
assign debug_cpu_iack = cpu_iack;
assign debug_cpu_fc = cpu_fc;
assign debug_cpu_dout = cpu_dout_debug;
assign debug_frame_epoch = frame_epoch;
assign debug_frame_tick = frame_tick;

logic [14:0] sound_shared_addr;
logic [7:0] sound_shared_dout;
logic sound_shared_we;
logic [7:0] sound_shared_din;
logic [7:0] coin_control;
logic v25_release;
logic [12:0] gp_snapshot_addr;
logic [15:0] gp_snapshot_data;
logic [9:0] gp_object_addr;
logic [15:0] gp_object_data;
logic gp_object_busy;
logic [127:0] gp_source_scrolls;
logic [7:0] gp_source_scroll_flip;
logic [10:0] palette_snapshot_addr;
logic [15:0] palette_snapshot_data;
logic text_line_ready;
logic text_opaque;
logic [10:0] text_color;
logic text_renderer_busy;
logic text_deadline_miss;
logic [15:0] text_build_cycles;
logic [63:0] board_ss_data_out;
logic board_ss_ack;
logic board_state_idle;
logic board_state_held;

// The coherent renderer consumes the snapshot module's object-history RAM.
// Keep the live GP object scan port parked while retaining its busy signal as
// part of the board's state-quiescence contract.
assign gp_object_addr = 10'd0;

fixeight_main_board u_board (
    .clk,
    .reset,
    .cold_reset(clock_reset),
    .hs_identity_valid, .hs_set_id, .hs_seed_crc32,
    .hs_config_download, .hs_nvram_download, .hs_nvram_upload,
    .hs_ioctl_wr, .hs_ioctl_rd, .hs_ioctl_addr, .hs_ioctl_data,
    .hs_nvram_q, .hs_nvram_wait, .hs_dirty, .hs_ready, .hs_active,
    .hs_ss_active, .hs_hold_request, .hs_hold_ack,
    .hs_restore_ready_known, .hs_restore_ready,
    .board_req,
    .board_addr,
    .board_rw,
    .board_byte_en,
    .board_wdata,
    .board_rdata,
    .board_ack,
    .p1,
    .p2,
    .p3,
    .system,
    .h_count,
    .v_count,
    .coin_control,
    .v25_release,
    .irq4_clear,
    .state_restore_commit(ss_restore_commit),
    .state_restore_coin_control(ss_restore_coin_control),
    .state_restore_v25_release(ss_restore_v25_release),
    .sound_shared_addr,
    .sound_shared_dout,
    .sound_shared_we,
    .sound_shared_din,
    .object_snapshot_start(snapshot_start),
    .gp_snapshot_addr,
    .gp_snapshot_data,
    .gp_object_addr,
    .gp_object_data,
    .gp_object_busy,
    .gp_scrolls(gp_source_scrolls),
    .gp_scroll_flip(gp_source_scroll_flip),
    .palette_snapshot_addr,
    .palette_snapshot_data,
    .line_start,
    .line_commit,
    .video_invalidate(ss_restore_commit),
    .target_y,
    .target_epoch,
    .display_x(h_count[8:0]),
    .display_y(v_count),
    .display_epoch(frame_epoch),
    .text_line_ready,
    .text_opaque,
    .text_color,
    .text_renderer_busy,
    .text_deadline_miss,
    .text_build_cycles,
    .state_hold,
    .ss_restore_enable,
    .ss_data,
    .ss_addr,
    .ss_select,
    .ss_write,
    .ss_read,
    .ss_query,
    .ss_data_out(board_ss_data_out),
    .ss_ack(board_ss_ack),
    .state_idle(board_state_idle),
    .state_held(board_state_held)
);

logic [12:0] gp_display_vram_addr;
logic [15:0] gp_display_vram_data;
logic [9:0] gp_display_object_addr;
logic [15:0] gp_display_object_data;
logic [127:0] gp_display_scrolls;
logic [7:0] gp_display_scroll_flip;
logic gp_snapshot_busy;
logic gp_snapshot_miss;

wire derived_video_reset = reset || ss_restore_commit;

fixeight_gp9001_snapshot u_gp_snapshot (
    .clk,
    .rst(derived_video_reset),
    .snapshot_start,
    .source_addr(gp_snapshot_addr),
    .source_data(gp_snapshot_data),
    .source_scrolls(gp_source_scrolls),
    .source_scroll_flip(gp_source_scroll_flip),
    .display_addr(gp_display_vram_addr),
    .display_data(gp_display_vram_data),
    .object_addr(gp_display_object_addr),
    .object_data(gp_display_object_data),
    .display_scrolls(gp_display_scrolls),
    .display_scroll_flip(gp_display_scroll_flip),
    .copy_busy(gp_snapshot_busy),
    .copy_miss(gp_snapshot_miss)
);

logic gp_line_ready;
logic [10:0] gp_color;
logic [1:0] gp_engine_busy;
logic [1:0] gp_deadline_miss;
logic [15:0] gp_tile_cycles;
logic [15:0] gp_object_cycles;

fixeight_gp9001_video u_gp_video (
    .clk,
    .rst(derived_video_reset),
    .line_start,
    .line_commit,
    .target_y,
    .target_epoch,
    .display_x(h_count[8:0]),
    .display_y(v_count),
    .display_epoch(frame_epoch),
    .scrolls(gp_display_scrolls),
    .scroll_flip(gp_display_scroll_flip),
    .vram_addr(gp_display_vram_addr),
    .vram_data(gp_display_vram_data),
    .object_addr(gp_display_object_addr),
    .object_data(gp_display_object_data),
    .tile_gfx_req,
    .tile_gfx_addr,
    .tile_gfx_data,
    .tile_gfx_ok(tile_gfx_ack),
    .object_gfx_req,
    .object_gfx_addr,
    .object_gfx_data,
    .object_gfx_ok(object_gfx_ack),
    .line_ready(gp_line_ready),
    .final_color(gp_color),
    .engine_busy(gp_engine_busy),
    .deadline_miss(gp_deadline_miss),
    .tile_cycles(gp_tile_cycles),
    .object_cycles(gp_object_cycles),
    .debug_tile_y(),
    .debug_object_y(),
    .debug_line_epoch(),
    .debug_line_valid()
);

logic [10:0] palette_display_addr;
logic [15:0] palette_display_data;
logic palette_snapshot_busy;

fixeight_palette_snapshot u_palette_snapshot (
    .clk,
    .rst(derived_video_reset),
    .snapshot_start,
    .source_addr(palette_snapshot_addr),
    .source_data(palette_snapshot_data),
    .display_addr(palette_display_addr),
    .display_data(palette_display_data),
    .copy_busy(palette_snapshot_busy)
);

logic composed_ready;
logic [7:0] composed_red;
logic [7:0] composed_green;
logic [7:0] composed_blue;

fixeight_video_compositor u_compositor (
    .display_active(timing_active),
    .hide_output(hide_output || state_hold),
    .gp_line_ready,
    .gp_color,
    .text_line_ready,
    .text_opaque,
    .text_color,
    .palette_word(palette_display_data),
    .line_ready(composed_ready),
    .palette_addr(palette_display_addr),
    .red(composed_red),
    .green(composed_green),
    .blue(composed_blue)
);

always_ff @(posedge clk) begin
    if (reset) begin
        red <= 8'h00;
        green <= 8'h00;
        blue <= 8'h00;
        hsync_n <= 1'b1;
        vsync_n <= 1'b1;
        hblank <= 1'b0;
        vblank <= 1'b0;
    end else if (pixel_tick) begin
        red <= composed_red;
        green <= composed_green;
        blue <= composed_blue;
        hsync_n <= timing_hsync_n;
        vsync_n <= timing_vsync_n;
        hblank <= timing_hblank;
        vblank <= timing_vblank;
    end
end

logic [63:0] sound_ss_data_out;
logic sound_ss_ack;
logic sound_state_idle;
logic sound_state_held;
logic sound_fault;
logic sound_halted;

fixeight_sound u_sound (
    .clk,
    .reset,
    .v25_release,
    .ce_v25(ce_cpu),
    .ce_opm,
    .ce_oki,
    .p1(p1[7:0]),
    .p2(p2[7:0]),
    .p3(p3[7:0]),
    .ym_enable,
    .oki_enable,
    .fx_level,
    .shared_addr(sound_shared_addr),
    .shared_dout(sound_shared_dout),
    .shared_we(sound_shared_we),
    .shared_din(sound_shared_din),
    .eeprom_seed_we,
    .eeprom_seed_addr,
    .eeprom_seed_data,
    .eeprom_state_hold(1'b0),
    .eeprom_state_we(1'b0),
    .eeprom_state_addr(7'd0),
    .eeprom_state_wdata(16'd0),
    .eeprom_state_rdata(),
    .oki_rom_addr,
    .oki_rom_data,
    .oki_rom_ok(oki_rom_ack),
    .state_hold,
    .ss_restore_enable,
    .ss_restore_commit,
    .ss_data,
    .ss_addr,
    .ss_select,
    .ss_write,
    .ss_read,
    .ss_query,
    .ss_data_out(sound_ss_data_out),
    .ss_ack(sound_ss_ack),
    .snd_mono(audio_mono),
    .sample(audio_sample),
    .state_idle(sound_state_idle),
    .state_held(sound_state_held),
    .debug_fault(sound_fault),
    .debug_halted(sound_halted),
    .debug_pc(debug_v25_pc),
    .debug_v25_reset_n(),
    .debug_ym_write(),
    .debug_ym_a0(),
    .debug_ym_data(),
    .debug_oki_write(),
    .debug_oki_data()
);

assign ss_ack = board_ss_ack || sound_ss_ack;
assign ss_data_out =
    board_ss_ack ? board_ss_data_out :
    sound_ss_ack ? sound_ss_data_out : 64'd0;
assign state_idle =
    board_state_idle && sound_state_idle &&
    !(|gp_engine_busy) && !text_renderer_busy &&
    !gp_snapshot_busy && !palette_snapshot_busy;
assign state_held =
    state_hold && state_idle && board_state_held && sound_state_held;
assign debug_video_busy = gp_engine_busy;
assign debug_video_miss = {
    text_deadline_miss, gp_deadline_miss
};
assign debug_coin_control = coin_control;
assign debug_v25_release = v25_release;

endmodule
