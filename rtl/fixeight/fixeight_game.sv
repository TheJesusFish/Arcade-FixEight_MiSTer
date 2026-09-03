// Framework-facing TP-026 game wrapper. The MiSTer root shell owns HPS I/O,
// video post-processing, SDRAM control, and save-state DDR transport.
module fixeight_game #(
    parameter integer AW = 22
) (
    input  logic              rst,
    input  logic              cold_rst,
    input  logic              clk,
    input  logic              clk96,
    input  logic              rst96,
    input  logic              clk48,
    input  logic              rst48,
    input  logic              clk24,
    input  logic              rst24,

    output logic              pxl2_cen,
    output logic              pxl_cen,
    output logic [7:0]        red,
    output logic [7:0]        green,
    output logic [7:0]        blue,
    output logic              LHBL,
    output logic              LVBL,
    output logic              HS,
    output logic              VS,

    input  logic [3:0]        cab_1p,
    input  logic [3:0]        coin,
    input  logic [`JTFRAME_BUTTONS+3:0] joystick1,
    input  logic [`JTFRAME_BUTTONS+3:0] joystick2,
    input  logic [`JTFRAME_BUTTONS+3:0] joystick3,
    input  logic [`JTFRAME_BUTTONS+3:0] joystick4,
    input  logic [15:0]       joyana_l1,
    input  logic [15:0]       joyana_l2,
    input  logic [15:0]       joyana_l3,
    input  logic [15:0]       joyana_l4,
    input  logic [15:0]       joyana_r1,
    input  logic [15:0]       joyana_r2,
    input  logic [15:0]       joyana_r3,
    input  logic [15:0]       joyana_r4,
    input  logic [1:0]        dial_x,
    input  logic [1:0]        dial_y,

    input  logic [26:0]       ioctl_addr,
    input  logic [7:0]        ioctl_dout,
    input  logic              ioctl_cart,
    input  logic              ioctl_wr,
    input  logic              ioctl_ram,
    output logic [7:0]        ioctl_din,
    input  logic              ioctl_rom,
    output logic              dwnld_busy,
    input  logic [15:0]       data_read,

    input  logic              hs_config_download,
    input  logic              hs_nvram_download,
    input  logic              hs_nvram_upload,
    input  logic              hs_ioctl_wr,
    input  logic              hs_ioctl_rd,
    input  logic [26:0]       hs_ioctl_addr,
    input  logic [7:0]        hs_ioctl_data,
    output logic [7:0]        hs_nvram_q,
    output logic              hs_nvram_wait,
    output logic              hs_dirty,
    output logic              hs_ready,
    output logic              hs_active,

    output logic [AW-1:0]     ba0_addr,
    output logic [AW-1:0]     ba1_addr,
    output logic [AW-1:0]     ba2_addr,
    output logic [AW-1:0]     ba3_addr,
    output logic [3:0]        ba_rd,
    output logic [3:0]        ba_wr,
    input  logic [3:0]        ba_dst,
    input  logic [3:0]        ba_dok,
    input  logic [3:0]        ba_rdy,
    input  logic [3:0]        ba_ack,
    output logic [15:0]       ba0_din,
    output logic [1:0]        ba0_dsn,
    output logic [15:0]       ba1_din,
    output logic [1:0]        ba1_dsn,
    output logic [15:0]       ba2_din,
    output logic [1:0]        ba2_dsn,
    output logic [15:0]       ba3_din,
    output logic [1:0]        ba3_dsn,

    output logic [1:0]        prog_ba,
    input  logic              prog_rdy,
    input  logic              prog_ack,
    input  logic              prog_dok,
    input  logic              prog_dst,
    output logic [15:0]       prog_data,
    output logic [AW-1:0]     prog_addr,
    output logic              prog_rd,
    output logic              prog_we,
    output logic [1:0]        prog_mask,

    input  logic [31:0]       status,
    input  logic              dip_pause,
    inout  wire               dip_flip,
    input  logic              dip_test,
    input  logic [1:0]        dip_fxlevel,
    input  logic              service,
    input  logic              tilt,
    input  logic [31:0]       dipsw,

    output logic signed [15:0] snd_left,
    output logic signed [15:0] snd_right,
    output logic              sample,
    input  logic [5:0]        snd_en,
    input  logic [7:0]        snd_vol,
    output logic [5:0]        snd_vu,
    output logic              snd_peak,

    input  logic [3:0]        gfx_en,
    output logic [7:0]        debug_bus,
    output logic [7:0]        debug_view,

    input  logic              ss_do_save,
    input  logic              ss_do_restore,
    input  logic              ss_busy,
    input  logic              ss_format_valid,
    output logic              ss_write_start,
    output logic              ss_read_start,
    output logic              ss_active,
    output logic [3:0]        ss_state_out,
    input  logic [63:0]       ss_data,
    input  logic [31:0]       ss_addr,
    input  logic [7:0]        ss_select,
    input  logic              ss_write,
    input  logic              ss_read,
    input  logic              ss_query,
    output logic [63:0]       ss_data_out,
    output logic              ss_ack
);

logic image_valid;
logic loader_accepted;
logic loader_range_error;
logic loader_overflow_error;
logic loader_sequence_error;
logic loader_length_error;
logic [26:0] loader_last_addr;
logic [26:0] loader_byte_count;
logic eeprom_seed_we;
logic [5:0] eeprom_seed_addr;
logic [15:0] eeprom_seed_data;
logic image_identity_valid;
logic [3:0] image_set_id;
logic [31:0] image_eeprom_crc32;
logic [26:0] image_aggregate_length;
logic [255:0] image_aggregate_sha256;

fixeight_rom_loader #(.AW(AW)) u_loader (
    .clk,
    .reset(cold_rst),
    .ioctl_rom,
    .ioctl_addr,
    .ioctl_dout,
    .ioctl_wr,
    .dwnld_busy,
    .image_valid,
    .image_identity_valid,
    .image_set_id,
    .image_eeprom_crc32,
    .image_aggregate_length,
    .image_aggregate_sha256,
    .prog_ba,
    .prog_addr,
    .prog_data,
    .prog_mask,
    .prog_rd,
    .prog_we,
    .prog_rdy,
    .prog_ack,
    .eeprom_seed_we,
    .eeprom_seed_addr,
    .eeprom_seed_data,
    .accepted(loader_accepted),
    .range_error(loader_range_error),
    .overflow_error(loader_overflow_error),
    .sequence_error(loader_sequence_error),
    .length_error(loader_length_error),
    .last_addr(loader_last_addr),
    .byte_count(loader_byte_count)
);

// JTFrame changes game reset on the opposite master-clock edge. Collapse that
// external, high-fanout reset into one local async-assert/sync-release chain
// before it reaches the board. Download and image-valid are synchronous here,
// so delaying their assertion by at most one edge is harmless while giving
// every board register a full-cycle reset path.
logic [1:0] board_reset_pipe;
always_ff @(posedge clk or posedge rst) begin
    if (rst)
        board_reset_pipe <= 2'b11;
    else if (dwnld_busy || !image_valid)
        board_reset_pipe <= 2'b11;
    else
        board_reset_pipe <= {board_reset_pipe[0], 1'b0};
end
wire board_reset = board_reset_pipe[1];

// Root metadata is written well before game clients are restored. Register
// its validity once at the game boundary to avoid a format-comparator path
// fanning directly into every restorable RAM bit.
logic ss_format_valid_q;
always_ff @(posedge clk) begin
    if (board_reset)
        ss_format_valid_q <= 1'b0;
    else
        ss_format_valid_q <= ss_format_valid;
end
logic [15:0] p1;
logic [15:0] p2;
logic [15:0] p3;
logic [15:0] system;

fixeight_inputs u_inputs (
    .joy1_n(joystick1[6:0]),
    .joy2_n(joystick2[6:0]),
    .joy3_n(joystick3[6:0]),
    .start_n(cab_1p),
    .coin_n(coin),
    .tilt_n(tilt),
    .test_n(dip_test),
    .region_reset_n(service),
    .p1,
    .p2,
    .p3,
    .system
);

logic main_rom_req;
logic [18:0] main_rom_word_addr;
logic [15:0] main_rom_data;
logic main_rom_ack;
logic tile_gfx_req;
logic [21:0] tile_gfx_addr;
logic [15:0] tile_gfx_data;
logic tile_gfx_ack;
logic object_gfx_req;
logic [21:0] object_gfx_addr;
logic [15:0] object_gfx_data;
logic object_gfx_ack;
logic [17:0] oki_rom_addr;
logic [7:0] oki_rom_data;
logic oki_rom_ack;
logic restore_commit;

fixeight_rom_fabric #(.AW(AW)) u_rom_fabric (
    .clk,
    .reset(board_reset),
    .invalidate(restore_commit),
    .main_req(main_rom_req),
    .main_word_addr(main_rom_word_addr),
    .main_data(main_rom_data),
    .main_ack(main_rom_ack),
    .tile_req(tile_gfx_req),
    .tile_logical_addr(tile_gfx_addr),
    .tile_data(tile_gfx_data),
    .tile_ack(tile_gfx_ack),
    .object_req(object_gfx_req),
    .object_logical_addr(object_gfx_addr),
    .object_data(object_gfx_data),
    .object_ack(object_gfx_ack),
    .oki_addr(oki_rom_addr),
    .oki_data(oki_rom_data),
    .oki_ack(oki_rom_ack),
    .ba0_addr,
    .ba1_addr,
    .ba2_addr,
    .ba3_addr,
    .ba_rd,
    .ba_wr,
    .ba_ack,
    .ba_dst,
    .ba_rdy,
    .data_read,
    .ba0_din,
    .ba1_din,
    .ba2_din,
    .ba3_din,
    .ba0_dsn,
    .ba1_dsn,
    .ba2_dsn,
    .ba3_dsn
);

logic hblank;
logic vblank;
logic pixel_tick;
logic [7:0] core_red;
logic [7:0] core_green;
logic [7:0] core_blue;
logic core_hsync_n;
logic core_vsync_n;
logic signed [15:0] audio_mono;
logic core_sample;
logic core_state_idle;
logic core_state_held;
logic [9:0] debug_h_count;
logic [8:0] debug_v_count;
logic debug_irq4;
logic [23:0] debug_main_pc;
logic [19:0] debug_v25_pc;
logic [1:0] debug_video_busy;
logic [2:0] debug_video_miss;
logic debug_cpu_bus_active;
logic debug_cpu_rw;
logic debug_cpu_ack;
logic debug_cpu_iack;
logic debug_cpu_lds_n;
logic [2:0] debug_cpu_fc;
logic [15:0] debug_cpu_dout;
logic [7:0] debug_coin_control;
logic debug_v25_release;
logic debug_frame_epoch;
logic debug_frame_tick;

logic state_hold;
logic state_irq7;
logic state_override;
logic state_reset;
logic state_cpu_run;
logic state_hide_output;
logic restore_begin;
logic restore_capture;
logic restore_enable;
logic restore_identity_valid;
logic [31:0] saved_ssp;
logic [31:0] restore_ssp;
logic restore_irq4;
logic [7:0] restore_coin_control;
logic restore_v25_release;
logic restore_hs_ready_known;
logic restore_hs_ready;
logic [15:0] restore_prev_p3;
logic [15:0] restore_prev_system;
logic [9:0] restore_h_count;
logic [8:0] restore_v_count;
logic restore_frame_epoch;
logic [63:0] scalar_ss_data_out;
logic scalar_ss_ack;
logic [63:0] core_ss_data_out;
logic core_ss_ack;

// The outer transport reserves selector 0 for immutable stream metadata.
// Translate selectors 1..15 onto the frozen zero-based game registry.
wire game_ss_selected =
    (ss_select >= 8'd1) && (ss_select <= 8'd15);
wire [7:0] game_ss_select = ss_select - 8'd1;
wire game_ss_write = ss_write && game_ss_selected;
wire game_ss_read = ss_read && game_ss_selected;
wire game_ss_query = ss_query && game_ss_selected;
wire state_safe_point =
    !board_reset &&
    (debug_h_count == 10'd0) &&
    (debug_v_count == 9'd240) &&
    !debug_cpu_bus_active;
wire [63:0] state_reset_vector = {
    restore_ssp, 16'h00ff, 16'h0008
};

// Preserve a UI request arriving on the same edge as an NVRAM handoff.
// NVRAM completes its bounded atomic RAM copy before the state owner starts.
logic pending_save, pending_restore;
wire start_save = (ss_do_save || pending_save) && !hs_active && !ss_active && !ss_busy;
wire start_restore = (ss_do_restore || pending_restore) && !hs_active && !ss_active && !ss_busy;
always_ff @(posedge clk) begin
    if (board_reset || start_save || start_restore) begin
        pending_save <= 0;
        pending_restore <= 0;
    end else if (!ss_active && !ss_busy) begin
        pending_save <= pending_save || ss_do_save;
        pending_restore <= pending_restore || ss_do_restore;
    end
end

fixeight_state_controller u_state_controller (
    .clk,
    .reset(board_reset),
    .do_save(start_save),
    .do_restore(start_restore),
    .stream_busy(ss_busy),
    .format_valid(ss_format_valid_q),
    .identity_valid(restore_identity_valid),
    .safe_point(state_safe_point),
    .core_held(core_state_held),
    .release_ready(core_state_idle),
    .frame_tick(debug_frame_tick),
    .h_count(debug_h_count),
    .v_count(debug_v_count),
    .cpu_bus_active(debug_cpu_bus_active),
    .cpu_rw(debug_cpu_rw),
    .cpu_ack(debug_cpu_ack),
    .cpu_iack(debug_cpu_iack),
    .cpu_lds_n(debug_cpu_lds_n),
    .cpu_fc(debug_cpu_fc),
    .cpu_addr(debug_main_pc),
    .cpu_dout(debug_cpu_dout),
    .write_start(ss_write_start),
    .read_start(ss_read_start),
    .active(ss_active),
    .state_out(ss_state_out),
    .state_hold,
    .state_irq7,
    .state_override,
    .state_reset,
    .cpu_run(state_cpu_run),
    .hide_output(state_hide_output),
    .restore_begin,
    .restore_capture,
    .restore_enable,
    .restore_commit,
    .saved_ssp
);

fixeight_state_clients u_state_clients (
    .clk,
    .reset(board_reset),
    .restore_begin,
    .restore_capture,
    .restore_commit,
    .state_hold,
    .image_identity_valid,
    .image_set_id,
    .image_eeprom_crc32,
    .image_aggregate_length,
    .image_aggregate_sha256,
    .current_irq4(debug_irq4),
    .current_coin_control(debug_coin_control),
    .current_v25_release(debug_v25_release),
    .current_hs_ready(hs_ready),
    .current_controller_state(ss_state_out),
    .saved_ssp,
    .current_p3(p3),
    .current_system(system),
    .current_h_count(debug_h_count),
    .current_v_count(debug_v_count),
    .current_frame_epoch(debug_frame_epoch),
    .ss_data,
    .ss_addr,
    .ss_select(game_ss_select),
    .ss_write(game_ss_write),
    .ss_read(game_ss_read),
    .ss_query(game_ss_query),
    .ss_data_out(scalar_ss_data_out),
    .ss_ack(scalar_ss_ack),
    .restore_identity_valid,
    .restore_ssp,
    .restore_irq4,
    .restore_coin_control,
    .restore_v25_release,
    .restore_hs_ready_known,
    .restore_hs_ready,
    .restore_prev_p3,
    .restore_prev_system,
    .restore_h_count,
    .restore_v_count,
    .restore_frame_epoch
);

fixeight_core u_core (
    .clk,
    .clock_reset(cold_rst),
    .hs_identity_valid(image_identity_valid), .hs_set_id(image_set_id),
    .hs_seed_crc32(image_eeprom_crc32),
    .hs_config_download, .hs_nvram_download, .hs_nvram_upload,
    .hs_ioctl_wr, .hs_ioctl_rd, .hs_ioctl_addr, .hs_ioctl_data,
    .hs_nvram_q, .hs_nvram_wait, .hs_dirty, .hs_ready, .hs_active,
    .hs_ss_active(ss_active || ss_busy || start_save || start_restore),
    .hs_restore_ready_known(restore_hs_ready_known),
    .hs_restore_ready(restore_hs_ready),
    .reset(board_reset),
    .halt_n(dip_pause),
    .cpu_run(state_cpu_run),
    .p1,
    .p2,
    .p3,
    .system,
    .ym_enable(!status[9]),
    .oki_enable(!status[8]),
    .fx_level(dip_fxlevel),
    .hide_output(board_reset || state_hide_output),
    .main_rom_req,
    .main_rom_word_addr,
    .main_rom_data,
    .main_rom_ack,
    .tile_gfx_req,
    .tile_gfx_addr,
    .tile_gfx_data,
    .tile_gfx_ack,
    .object_gfx_req,
    .object_gfx_addr,
    .object_gfx_data,
    .object_gfx_ack,
    .oki_rom_addr,
    .oki_rom_data,
    .oki_rom_ack,
    .eeprom_seed_we,
    .eeprom_seed_addr,
    .eeprom_seed_data,
    .state_hold,
    .state_irq7,
    .state_override,
    .state_reset,
    .state_reset_vector,
    .ss_restore_enable(restore_enable),
    .ss_restore_commit(restore_commit),
    .ss_restore_irq4(restore_irq4),
    .ss_restore_coin_control(restore_coin_control),
    .ss_restore_v25_release(restore_v25_release),
    .ss_restore_h_count(restore_h_count),
    .ss_restore_v_count(restore_v_count),
    .ss_restore_frame_epoch(restore_frame_epoch),
    .ss_data,
    .ss_addr,
    .ss_select(game_ss_select),
    .ss_write(game_ss_write),
    .ss_read(game_ss_read),
    .ss_query(game_ss_query),
    .ss_data_out(core_ss_data_out),
    .ss_ack(core_ss_ack),
    .state_idle(core_state_idle),
    .state_held(core_state_held),
    .red(core_red),
    .green(core_green),
    .blue(core_blue),
    .hsync_n(core_hsync_n),
    .vsync_n(core_vsync_n),
    .hblank,
    .vblank,
    .pixel_tick,
    .audio_mono,
    .audio_sample(core_sample),
    .debug_h_count,
    .debug_v_count,
    .debug_irq4,
    .debug_main_pc,
    .debug_cpu_bus_active,
    .debug_cpu_rw,
    .debug_cpu_ack,
    .debug_cpu_iack,
    .debug_cpu_lds_n,
    .debug_cpu_fc,
    .debug_cpu_dout,
    .debug_coin_control,
    .debug_v25_release,
    .debug_frame_epoch,
    .debug_frame_tick,
    .debug_v25_pc,
    .debug_video_busy,
    .debug_video_miss
);

assign ss_ack = game_ss_selected && (scalar_ss_ack || core_ss_ack);
wire [63:0] game_ss_data_out_local =
    scalar_ss_ack ? scalar_ss_data_out :
    core_ss_ack ? core_ss_data_out : 64'd0;
assign ss_data_out =
    game_ss_query && (scalar_ss_ack || core_ss_ack) ?
    {ss_select, game_ss_data_out_local[55:0]} :
    game_ss_data_out_local;

assign snd_left = audio_mono;
assign snd_right = audio_mono;
assign sample = core_sample;
wire [15:0] audio_magnitude =
    audio_mono[15] ? (~audio_mono + 16'd1) : audio_mono;
assign snd_vu = audio_magnitude[14:9];
assign snd_peak = |audio_magnitude[15:14];
assign ioctl_din = 8'd0;
assign dip_flip = 1'b0;

assign pxl_cen = pixel_tick;
assign red = core_red;
assign green = core_green;
assign blue = core_blue;
assign HS = core_hsync_n;
assign VS = core_vsync_n;
assign LHBL = !hblank;
assign LVBL = !vblank;

logic [3:0] pixel2_div;
always_ff @(posedge clk) begin
    // Keep both /14 pixel phases aligned and running while JTFrame holds
    // game reset. The reset sequencer itself advances on pxl_cen.
    if (cold_rst) begin
        pixel2_div <= 4'd0;
        pxl2_cen <= 1'b0;
    end else begin
        pxl2_cen <= (pixel2_div == 4'd6) || (pixel2_div == 4'd13);
        if (pixel2_div == 4'd13)
            pixel2_div <= 4'd0;
        else
            pixel2_div <= pixel2_div + 4'd1;
    end
end

assign debug_bus = {
    loader_range_error,
    loader_overflow_error,
    loader_sequence_error,
    loader_length_error,
    image_valid,
    dwnld_busy,
    debug_irq4,
    |debug_video_miss
};
assign debug_view = dwnld_busy ?
    loader_last_addr[7:0] : debug_main_pc[15:8];

endmodule
