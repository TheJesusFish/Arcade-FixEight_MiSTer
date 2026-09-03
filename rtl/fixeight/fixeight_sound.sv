// SPDX-License-Identifier: BSD-3-Clause
// TP-026 V25, serial EEPROM, YM2151, MSM6295, and mono-mix subsystem.

module fixeight_sound (
    input  logic               clk,
    input  logic               reset,
    input  logic               v25_release,
    input  logic               ce_v25,
    input  logic               ce_opm,
    input  logic               ce_oki,
    input  logic [7:0]         p1,
    input  logic [7:0]         p2,
    input  logic [7:0]         p3,
    input  logic               ym_enable,
    input  logic               oki_enable,
    input  logic [1:0]         fx_level,

    output logic [14:0]        shared_addr,
    output logic [7:0]         shared_dout,
    output logic               shared_we,
    input  logic [7:0]         shared_din,

    input  logic               eeprom_seed_we,
    input  logic [5:0]         eeprom_seed_addr,
    input  logic [15:0]        eeprom_seed_data,
    input  logic               eeprom_state_hold,
    input  logic               eeprom_state_we,
    input  logic [6:0]         eeprom_state_addr,
    input  logic [15:0]        eeprom_state_wdata,
    output logic [15:0]        eeprom_state_rdata,

    output logic [17:0]        oki_rom_addr,
    input  logic [7:0]         oki_rom_data,
    input  logic               oki_rom_ok,

    input  logic               state_hold,
    input  logic               ss_restore_enable,
    input  logic               ss_restore_commit,
    input  logic [63:0]        ss_data,
    input  logic [31:0]        ss_addr,
    input  logic [7:0]         ss_select,
    input  logic               ss_write,
    input  logic               ss_read,
    input  logic               ss_query,
    output logic [63:0]        ss_data_out,
    output logic               ss_ack,

    output logic signed [15:0] snd_mono,
    output logic               sample,
    output logic               state_idle,
    output logic               state_held,
    output logic               debug_fault,
    output logic               debug_halted,
    output logic [19:0]        debug_pc,
    output logic               debug_v25_reset_n,
    output logic               debug_ym_write,
    output logic               debug_ym_a0,
    output logic [7:0]         debug_ym_data,
    output logic               debug_oki_write,
    output logic [7:0]         debug_oki_data
);

localparam logic [19:0] YM_ADDR  = 20'h0000a;
localparam logic [19:0] YM_DATA  = 20'h0000b;
localparam logic [19:0] OKI_DATA = 20'h0000c;
wire sound_domain_reset = reset || ss_restore_commit;

logic [5:0] ym_reset_count;
logic ym_reset;
logic [1:0] oki_reset_state;
logic [13:0] oki_reset_timer;
logic oki_reset;
logic oki_ready;
logic sound_ready;
logic v25_started;
logic [1:0] v25_reset_pipe;
logic v25_reset_n;

always_ff @(posedge clk or posedge sound_domain_reset) begin
    if (sound_domain_reset) begin
        ym_reset_count <= 6'd0;
    end else if (!ym_reset_count[5] && ce_opm) begin
        ym_reset_count <= ym_reset_count + 6'd1;
    end
end
assign ym_reset = sound_domain_reset || !ym_reset_count[5];

// JT6295's acknowledge ring has no reset. Prime one rotation, pulse reset,
// then release from a deterministic state as in the audited donor wrapper.
always_ff @(posedge clk or posedge sound_domain_reset) begin
    if (sound_domain_reset) begin
        oki_reset_state <= 2'd0;
        oki_reset_timer <= 14'd0;
        oki_reset <= 1'b1;
        oki_ready <= 1'b0;
    end else begin
        case (oki_reset_state)
            2'd0: begin
                oki_reset <= 1'b0;
                if (&oki_reset_timer) begin
                    oki_reset_state <= 2'd1;
                    oki_reset_timer <= 14'd0;
                    oki_reset <= 1'b1;
                end else begin
                    oki_reset_timer <= oki_reset_timer + 14'd1;
                end
            end
            2'd1: begin
                oki_reset <= 1'b1;
                if (oki_reset_timer == 14'd15) begin
                    oki_reset_state <= 2'd2;
                    oki_reset_timer <= 14'd0;
                    oki_reset <= 1'b0;
                    oki_ready <= 1'b1;
                end else begin
                    oki_reset_timer <= oki_reset_timer + 14'd1;
                end
            end
            default: begin
                oki_reset <= 1'b0;
                oki_ready <= 1'b1;
            end
        endcase
    end
end

assign sound_ready = !ym_reset && oki_ready;
always_ff @(posedge clk or posedge reset) begin
    if (reset)
        v25_started <= 1'b0;
    else if (!v25_release)
        v25_started <= 1'b0;
    else if (sound_ready)
        v25_started <= 1'b1;
end

wire v25_reset_async = reset || !v25_release || !v25_started;
always_ff @(posedge clk or posedge v25_reset_async) begin
    if (v25_reset_async)
        v25_reset_pipe <= 2'b11;
    else
        v25_reset_pipe <= {v25_reset_pipe[0], 1'b0};
end
assign v25_reset_n = !v25_reset_pipe[1];
assign debug_v25_reset_n = v25_reset_n;

logic [19:0] v25_bus_addr;
logic [7:0] v25_bus_dout;
logic [7:0] v25_bus_din;
logic v25_bus_doe;
logic v25_bus_r_w;
logic v25_bus_mreq_n;
logic v25_bus_mstb_n;
logic v25_bus_iostb_n;
logic v25_state_idle;
logic [7:0] v25_port0_out;
logic eeprom_do;
logic [63:0] v25_ss_data_out;
logic v25_ss_ack;
logic [63:0] intent_ss_data_out;
logic intent_ss_ack;
wire bgm_replay_holds_v25;
wire v25_hold_boundary;

// The TP-026 upload/ack sequence runs the V25 while the 68000 is still
// populating shared RAM. Use the audited TS001 service timing from the
// Dogyuun subsystem explicitly; the wrapper's untuned defaults can outrun
// that producer and execute an incomplete upload before writing the 0xff
// readiness acknowledgement.
fixeight_v25_cpu #(
    .BUS_PHASES_USE_CEN(1),
    .BUS_BYTE_PHASES(4),
    .SOUND_WRITE_EXTRA_PHASES(17),
    .INSTRUCTION_GAP_PHASES(1),
    .REGISTER_INC_GAP_PHASES(4),
    .OPCODE_TIMING_PROFILE(1),
    .TEST1_IMMEDIATE_DELAY(26),
    .TEST1_MEMORY_DELAY(4)
) u_v25 (
    .clk,
    .reset,
    .reset_n(v25_reset_n),
    .clock_enable(
        ce_v25 && !v25_hold_boundary && !bgm_replay_holds_v25
    ),
    .port0_in({eeprom_do, 7'h00}),
    .port1_in(8'h00),
    .portt_in(8'h00),
    .port0_out(v25_port0_out),
    .bus_addr(v25_bus_addr),
    .bus_dout(v25_bus_dout),
    .bus_din(v25_bus_din),
    .bus_doe(v25_bus_doe),
    .bus_r_w(v25_bus_r_w),
    .bus_mreq_n(v25_bus_mreq_n),
    .bus_mstb_n(v25_bus_mstb_n),
    .bus_iostb_n(v25_bus_iostb_n),
    .halted(debug_halted),
    .fault(debug_fault),
    .debug_pc(debug_pc),
    .state_idle(v25_state_idle),
    .ss_restore_enable,
    .ss_restore_commit,
    .ss_data,
    .ss_addr,
    .ss_select,
    .ss_write,
    .ss_read,
    .ss_query,
    .ss_data_out(v25_ss_data_out),
    .ss_ack(v25_ss_ack)
);

wire eeprom_ss_selected = ss_select == 8'd10;
wire eeprom_ss_word_valid = ss_addr < 32'd72;
wire eeprom_ss_write =
    eeprom_ss_selected && eeprom_ss_word_valid && ss_write &&
    ss_restore_enable && state_hold;
wire eeprom_hold = eeprom_state_hold || state_hold;
wire eeprom_write = eeprom_state_we || eeprom_ss_write;
wire [6:0] eeprom_addr =
    eeprom_ss_selected ? ss_addr[6:0] : eeprom_state_addr;
wire [15:0] eeprom_wdata =
    eeprom_ss_selected ? ss_data[15:0] : eeprom_state_wdata;
logic [15:0] eeprom_rdata;
logic [63:0] eeprom_ss_data_out;
logic eeprom_ss_ack;

fixeight_eeprom u_eeprom (
    .clk,
    .reset,
    .serial_cs(v25_port0_out[4]),
    .serial_clk(v25_port0_out[5]),
    .serial_di(v25_port0_out[6]),
    .serial_do(eeprom_do),
    .seed_we(eeprom_seed_we),
    .seed_addr(eeprom_seed_addr),
    .seed_data(eeprom_seed_data),
    .state_hold(eeprom_hold),
    .state_we(eeprom_write),
    .state_addr(eeprom_addr),
    .state_wdata(eeprom_wdata),
    .state_rdata(eeprom_rdata)
);
assign eeprom_state_rdata = eeprom_rdata;

always_ff @(posedge clk) begin
    if (reset) begin
        eeprom_ss_data_out <= 64'd0;
        eeprom_ss_ack <= 1'b0;
    end else begin
        eeprom_ss_ack <= 1'b0;
        if (eeprom_ss_selected && ss_query) begin
            eeprom_ss_data_out <= {8'd10, 22'd0, 2'd1, 32'd72};
            eeprom_ss_ack <= 1'b1;
        end else if (
            eeprom_ss_selected && eeprom_ss_word_valid &&
            (ss_read || ss_write)
        ) begin
            eeprom_ss_data_out <= {48'd0, eeprom_rdata};
            eeprom_ss_ack <= 1'b1;
        end
    end
end

wire v25_mem_active = !v25_bus_mreq_n && !v25_bus_mstb_n;
wire v25_write_active =
    v25_mem_active && !v25_bus_r_w && v25_bus_doe;
logic v25_write_active_q;
wire v25_write_start = v25_write_active && !v25_write_active_q;
wire v25_shared_cs = v25_bus_addr[19];
wire v25_mailbox_ready_write =
    v25_write_start && v25_shared_cs &&
    (v25_bus_addr[14:0] == 15'h7800) &&
    (v25_bus_dout == 8'hff);
wire v25_shared_read_active =
    v25_mem_active && v25_bus_r_w && v25_shared_cs;
logic v25_shared_read_active_q;
logic [14:0] v25_shared_read_addr_q;
wire v25_shared_read_stable =
    v25_shared_read_active && v25_shared_read_active_q &&
    (v25_bus_addr[14:0] == v25_shared_read_addr_q);

logic [7:0] bgm_pending_command;
logic [7:0] bgm_pending_argument;
logic bgm_pending_valid;
logic bgm_pending_argument_valid;
logic [7:0] sound_bgm_command;
logic [7:0] sound_bgm_argument;
logic sound_bgm_valid;
logic [7:0] restored_bgm_command;
logic [7:0] restored_bgm_argument;
logic restored_bgm_valid;

wire intent_ss_selected = ss_select == 8'd12;
always_ff @(posedge clk) begin
    if (reset) begin
        intent_ss_data_out <= 64'd0;
        intent_ss_ack <= 1'b0;
        restored_bgm_command <= 8'd0;
        restored_bgm_argument <= 8'd0;
        restored_bgm_valid <= 1'b0;
    end else begin
        intent_ss_ack <= 1'b0;
        if (intent_ss_selected && ss_query) begin
            intent_ss_data_out <= {8'd12, 22'd0, 2'd3, 32'd1};
            intent_ss_ack <= 1'b1;
        end else if (
            intent_ss_selected && (ss_addr == 32'd0) && ss_read
        ) begin
            intent_ss_data_out <= {
                23'd0,
                sound_bgm_valid,
                sound_bgm_argument,
                sound_bgm_command,
                24'd0
            };
            intent_ss_ack <= 1'b1;
        end else if (
            intent_ss_selected && (ss_addr == 32'd0) && ss_write &&
            ss_restore_enable && state_hold
        ) begin
            restored_bgm_command <= ss_data[31:24];
            restored_bgm_argument <= ss_data[39:32];
            restored_bgm_valid <= ss_data[40];
            intent_ss_ack <= 1'b1;
        end
    end
end

always_ff @(posedge clk) begin
    if (reset) begin
        v25_shared_read_active_q <= 1'b0;
        v25_shared_read_addr_q <= 15'd0;
        bgm_pending_command <= 8'd0;
        bgm_pending_argument <= 8'd0;
        bgm_pending_valid <= 1'b0;
        bgm_pending_argument_valid <= 1'b0;
        sound_bgm_command <= 8'd0;
        sound_bgm_argument <= 8'd0;
        sound_bgm_valid <= 1'b0;
    end else begin
        v25_shared_read_active_q <= v25_shared_read_active;
        v25_shared_read_addr_q <= v25_bus_addr[14:0];

        if (ss_restore_commit && ss_restore_enable) begin
            bgm_pending_command <= 8'd0;
            bgm_pending_argument <= 8'd0;
            bgm_pending_valid <= 1'b0;
            bgm_pending_argument_valid <= 1'b0;
            sound_bgm_command <= restored_bgm_command;
            sound_bgm_argument <= restored_bgm_argument;
            sound_bgm_valid <= restored_bgm_valid;
        end else begin
            if (
                v25_shared_read_stable &&
                (v25_bus_addr[14:0] == 15'h7800) &&
                !bgm_pending_valid &&
                (shared_din != 8'hff) && (shared_din != 8'haa)
            ) begin
                bgm_pending_command <= shared_din;
                bgm_pending_argument <= 8'd0;
                bgm_pending_valid <= 1'b1;
                bgm_pending_argument_valid <= 1'b0;
            end else if (
                v25_shared_read_stable &&
                (v25_bus_addr[14:0] == 15'h7801) &&
                bgm_pending_valid
            ) begin
                bgm_pending_argument <= shared_din;
                bgm_pending_argument_valid <= 1'b1;
            end

            if (v25_mailbox_ready_write) begin
                if (
                    bgm_pending_valid &&
                    bgm_pending_argument_valid &&
                    (bgm_pending_command == 8'h00) &&
                    (bgm_pending_argument == 8'h01)
                ) begin
                    sound_bgm_valid <= 1'b0;
                end else if (
                    bgm_pending_valid &&
                    bgm_pending_argument_valid &&
                    (bgm_pending_command == 8'h01) &&
                    (bgm_pending_argument == 8'h00)
                ) begin
                    sound_bgm_command <= bgm_pending_command;
                    sound_bgm_argument <= bgm_pending_argument;
                    sound_bgm_valid <= 1'b1;
                end
                bgm_pending_valid <= 1'b0;
                bgm_pending_argument_valid <= 1'b0;
            end
        end
    end
end

localparam logic [3:0] BGM_IDLE          = 4'd0;
localparam logic [3:0] BGM_WAIT_READY    = 4'd1;
localparam logic [3:0] BGM_STOP_ARG      = 4'd2;
localparam logic [3:0] BGM_STOP_COMMAND  = 4'd3;
localparam logic [3:0] BGM_RELEASE_STOP  = 4'd4;
localparam logic [3:0] BGM_WAIT_STOP_ACK = 4'd5;
localparam logic [3:0] BGM_WRITE_ARG     = 4'd6;
localparam logic [3:0] BGM_WRITE_COMMAND = 4'd7;
localparam logic [3:0] BGM_RELEASE       = 4'd8;
localparam logic [3:0] BGM_WAIT_ACK      = 4'd9;

logic [3:0] bgm_replay_state;
logic [7:0] bgm_replay_command;
logic [7:0] bgm_replay_argument;
logic [3:0] bgm_replay_wait_count;
wire bgm_stop_arg = bgm_replay_state == BGM_STOP_ARG;
wire bgm_stop_command = bgm_replay_state == BGM_STOP_COMMAND;
wire bgm_write_arg = bgm_replay_state == BGM_WRITE_ARG;
wire bgm_write_command = bgm_replay_state == BGM_WRITE_COMMAND;
assign bgm_replay_holds_v25 =
    (bgm_replay_state == BGM_WAIT_READY) ||
    (bgm_replay_state == BGM_STOP_ARG) ||
    (bgm_replay_state == BGM_STOP_COMMAND) ||
    (bgm_replay_state == BGM_RELEASE_STOP) ||
    (bgm_replay_state == BGM_WRITE_ARG) ||
    (bgm_replay_state == BGM_WRITE_COMMAND) ||
    (bgm_replay_state == BGM_RELEASE);
wire bgm_replay_active = bgm_replay_state != BGM_IDLE;
assign v25_hold_boundary =
    state_hold && v25_state_idle && !bgm_replay_active;

always_ff @(posedge clk) begin
    if (reset) begin
        bgm_replay_state <= BGM_IDLE;
        bgm_replay_command <= 8'd0;
        bgm_replay_argument <= 8'd0;
        bgm_replay_wait_count <= 4'd0;
    end else if (ss_restore_commit && ss_restore_enable) begin
        bgm_replay_state <=
            restored_bgm_valid ? BGM_WAIT_READY : BGM_IDLE;
        bgm_replay_command <= restored_bgm_command;
        bgm_replay_argument <= restored_bgm_argument;
        bgm_replay_wait_count <= 4'd0;
    end else begin
        case (bgm_replay_state)
            BGM_WAIT_READY: begin
                if (!sound_ready)
                    bgm_replay_wait_count <= 4'd0;
                else if (bgm_replay_wait_count == 4'd8) begin
                    bgm_replay_state <= BGM_STOP_ARG;
                    bgm_replay_wait_count <= 4'd0;
                end else
                    bgm_replay_wait_count <=
                        bgm_replay_wait_count + 4'd1;
            end
            BGM_STOP_ARG:
                bgm_replay_state <= BGM_STOP_COMMAND;
            BGM_STOP_COMMAND:
                bgm_replay_state <= BGM_RELEASE_STOP;
            BGM_RELEASE_STOP:
                bgm_replay_state <= BGM_WAIT_STOP_ACK;
            BGM_WAIT_STOP_ACK:
                if (v25_mailbox_ready_write)
                    bgm_replay_state <= BGM_WRITE_ARG;
            BGM_WRITE_ARG:
                bgm_replay_state <= BGM_WRITE_COMMAND;
            BGM_WRITE_COMMAND:
                bgm_replay_state <= BGM_RELEASE;
            BGM_RELEASE:
                bgm_replay_state <= BGM_WAIT_ACK;
            BGM_WAIT_ACK:
                if (v25_mailbox_ready_write)
                    bgm_replay_state <= BGM_IDLE;
            default:
                bgm_replay_state <= BGM_IDLE;
        endcase
    end
end

assign shared_addr =
    (bgm_stop_arg || bgm_write_arg) ? 15'h7801 :
    (bgm_stop_command || bgm_write_command) ? 15'h7800 :
    v25_bus_addr[14:0];
assign shared_dout =
    bgm_stop_arg ? 8'h01 :
    bgm_stop_command ? 8'h00 :
    bgm_write_arg ? bgm_replay_argument :
    bgm_write_command ? bgm_replay_command :
    v25_bus_dout;
assign shared_we =
    bgm_stop_arg || bgm_stop_command ||
    bgm_write_arg || bgm_write_command ||
    (v25_write_start && v25_shared_cs);

logic [7:0] ym_dout;
logic [7:0] oki_dout;
always_comb begin
    v25_bus_din = 8'h00;
    if (v25_mem_active && v25_bus_r_w) begin
        if (v25_shared_cs)
            v25_bus_din = shared_din;
        else begin
            case (v25_bus_addr)
                20'h00000: v25_bus_din = p1;
                20'h00002: v25_bus_din = p2;
                20'h00004: v25_bus_din = p3;
                YM_ADDR,
                YM_DATA: v25_bus_din = ym_dout;
                OKI_DATA: v25_bus_din = oki_dout;
                default: v25_bus_din = 8'h00;
            endcase
        end
    end
end

logic ym_cs_n;
logic ym_wr_n;
logic ym_host_a0;
logic [7:0] ym_host_data;
logic ym_write_pending;
logic oki_wr_n;
logic [7:0] oki_host_data;

always_ff @(posedge clk) begin
    if (!v25_reset_n || sound_domain_reset) begin
        v25_write_active_q <= 1'b0;
        ym_cs_n <= 1'b1;
        ym_wr_n <= 1'b1;
        ym_host_a0 <= 1'b0;
        ym_host_data <= 8'h00;
        ym_write_pending <= 1'b0;
        oki_wr_n <= 1'b1;
        oki_host_data <= 8'h00;
        debug_ym_write <= 1'b0;
        debug_ym_a0 <= 1'b0;
        debug_ym_data <= 8'h00;
        debug_oki_write <= 1'b0;
        debug_oki_data <= 8'h00;
    end else begin
        v25_write_active_q <= v25_write_active;
        oki_wr_n <= 1'b1;
        debug_ym_write <= 1'b0;
        debug_oki_write <= 1'b0;

        if (ym_write_pending) begin
            ym_cs_n <= 1'b0;
            ym_wr_n <= 1'b0;
            if (ce_opm) begin
                ym_cs_n <= 1'b1;
                ym_wr_n <= 1'b1;
                ym_write_pending <= 1'b0;
            end
        end else begin
            ym_cs_n <= 1'b1;
            ym_wr_n <= 1'b1;
        end

        if (v25_write_start && !v25_shared_cs) begin
            if (v25_bus_addr == YM_ADDR || v25_bus_addr == YM_DATA) begin
                ym_cs_n <= 1'b0;
                ym_wr_n <= 1'b0;
                ym_host_a0 <= v25_bus_addr[0];
                ym_host_data <= v25_bus_dout;
                ym_write_pending <= 1'b1;
                debug_ym_write <= 1'b1;
                debug_ym_a0 <= v25_bus_addr[0];
                debug_ym_data <= v25_bus_dout;
            end else if (v25_bus_addr == OKI_DATA) begin
                oki_wr_n <= 1'b0;
                oki_host_data <= v25_bus_dout;
                debug_oki_write <= 1'b1;
                debug_oki_data <= v25_bus_dout;
            end
        end
    end
end

logic ym_sample;
logic signed [15:0] ym_left;
logic signed [15:0] ym_right;
fixeight_opm u_ym2151 (
    .reset(ym_reset),
    .clk,
    .cen(ce_opm),
    .cs_n(ym_cs_n),
    .wr_n(ym_wr_n),
    .a0(ym_host_a0),
    .din(ym_host_data),
    .dout(ym_dout),
    .sample(ym_sample),
    .left(ym_left),
    .right(ym_right)
);

logic signed [13:0] oki_sound;
logic oki_sample;
jt6295 #(.INTERPOL(0)) u_oki6295 (
    .rst(oki_reset),
    .clk,
    .cen(ce_oki),
    .ss(1'b1),
    .wrn(oki_wr_n),
    .din(oki_host_data),
    .dout(oki_dout),
    .rom_addr(oki_rom_addr),
    .rom_data(oki_rom_data),
    .rom_ok(oki_rom_ok),
    .sound(oki_sound),
    .sample(oki_sample)
);

fixeight_sound_mixer u_mixer (
    .clk,
    .reset(sound_domain_reset || ym_reset),
    .sample(ym_sample),
    .ym_left,
    .ym_right,
    .oki(oki_sound),
    .ym_enable,
    .oki_enable,
    .fx_level,
    .oki_ready,
    .mono(snd_mono)
);

assign sample = ym_sample;
assign state_idle =
    v25_state_idle && !v25_write_active && !ym_write_pending &&
    ym_cs_n && ym_wr_n && oki_wr_n &&
    sound_ready && !bgm_replay_active;
assign state_held = state_hold && state_idle;
assign ss_ack = eeprom_ss_ack || v25_ss_ack || intent_ss_ack;
assign ss_data_out =
    eeprom_ss_ack ? eeprom_ss_data_out :
    v25_ss_ack ? v25_ss_data_out :
    intent_ss_ack ? intent_ss_data_out : 64'd0;

endmodule
