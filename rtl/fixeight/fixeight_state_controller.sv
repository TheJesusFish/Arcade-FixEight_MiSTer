// SPDX-License-Identifier: BSD-3-Clause
// Persistent save/restore sequencer for the TP-026 single-clock board.
module fixeight_state_controller #(
    parameter logic [19:0] IRQ_TIMEOUT = 20'd999999,
    parameter logic [19:0] HOLD_TIMEOUT = 20'd999999
) (
    input  logic        clk,
    input  logic        reset,
    input  logic        do_save,
    input  logic        do_restore,
    input  logic        stream_busy,
    input  logic        format_valid,
    input  logic        identity_valid,
    input  logic        safe_point,
    input  logic        core_held,
    input  logic        release_ready,
    input  logic        frame_tick,
    input  logic [9:0]  h_count,
    input  logic [8:0]  v_count,
    input  logic        cpu_bus_active,
    input  logic        cpu_rw,
    input  logic        cpu_ack,
    input  logic        cpu_iack,
    input  logic        cpu_lds_n,
    input  logic [2:0]  cpu_fc,
    input  logic [23:0] cpu_addr,
    input  logic [15:0] cpu_dout,

    output logic        write_start,
    output logic        read_start,
    output logic        active,
    output logic [3:0]  state_out,
    output logic        state_hold,
    output logic        state_irq7,
    output logic        state_override,
    output logic        state_reset,
    output logic        cpu_run,
    output logic        hide_output,
    output logic        restore_begin,
    output logic        restore_capture,
    output logic        restore_enable,
    output logic        restore_commit,
    output logic [31:0] saved_ssp
);

localparam logic [3:0] IDLE                = 4'd0;
localparam logic [3:0] SAVE_WAIT_SAFE      = 4'd1;
localparam logic [3:0] SAVE_WAIT_IRQ       = 4'd2;
localparam logic [3:0] SAVE_WAIT_SSP       = 4'd3;
localparam logic [3:0] SAVE_WAIT_STREAM    = 4'd4;
localparam logic [3:0] SAVE_WAIT_EXIT      = 4'd5;
localparam logic [3:0] RESTORE_WAIT_SAFE   = 4'd6;
localparam logic [3:0] RESTORE_WAIT_STREAM = 4'd7;
localparam logic [3:0] RESTORE_HOLD_RESET  = 4'd8;
localparam logic [3:0] RESTORE_WAIT_RESET  = 4'd9;
localparam logic [3:0] SAVE_WAIT_HOLD      = 4'd10;
localparam logic [3:0] RESTORE_WAIT_HOLD   = 4'd11;
localparam logic [3:0] RESTORE_WAIT_VIDEO  = 4'd12;
localparam logic [3:0] SAVE_WAIT_VIDEO     = 4'd13;
localparam logic [3:0] SAVE_RETRY          = 4'd14;

logic [3:0] state;
logic [19:0] wait_counter;
logic [2:0] irq_retry_count;
logic [7:0] reset_counter;
logic save_frame_seen;
logic [1:0] restore_frame_count;
logic release_ready_seen;

assign active = state != IDLE;
assign state_out = state;
assign hide_output = active;
assign state_irq7 = state == SAVE_WAIT_IRQ;
assign state_override =
    (state == SAVE_WAIT_SSP) ||
    (state == SAVE_WAIT_EXIT) ||
    (state == RESTORE_HOLD_RESET) ||
    (state == RESTORE_WAIT_RESET);
assign state_reset = state == RESTORE_HOLD_RESET;
assign state_hold =
    (state == SAVE_WAIT_HOLD) ||
    (state == SAVE_WAIT_STREAM) ||
    (state == RESTORE_WAIT_HOLD) ||
    (state == RESTORE_WAIT_STREAM) ||
    (state == RESTORE_HOLD_RESET);
assign cpu_run =
    (state != SAVE_WAIT_HOLD) &&
    (state != SAVE_WAIT_STREAM) &&
    (state != RESTORE_WAIT_HOLD) &&
    (state != RESTORE_WAIT_STREAM) &&
    (state != RESTORE_HOLD_RESET);
assign restore_capture = state == RESTORE_WAIT_STREAM;
assign restore_enable =
    (restore_capture || restore_commit ||
     (state == RESTORE_HOLD_RESET)) &&
    format_valid && identity_valid;

wire video_release_point =
    (h_count == 10'd0) && (v_count == 9'd240);

always_ff @(posedge clk) begin
    if (reset) begin
        state <= IDLE;
        write_start <= 1'b0;
        read_start <= 1'b0;
        restore_begin <= 1'b0;
        restore_commit <= 1'b0;
        wait_counter <= 20'd0;
        irq_retry_count <= 3'd0;
        reset_counter <= 8'd0;
        save_frame_seen <= 1'b0;
        restore_frame_count <= 2'd0;
        release_ready_seen <= 1'b0;
        saved_ssp <= 32'd0;
    end else begin
        restore_begin <= 1'b0;
        restore_commit <= 1'b0;

        if ((state == SAVE_WAIT_VIDEO) && frame_tick)
            save_frame_seen <= 1'b1;
        if (
            (state == RESTORE_WAIT_VIDEO) && frame_tick &&
            (restore_frame_count != 2'd3)
        )
            restore_frame_count <= restore_frame_count + 2'd1;
        if ((state == RESTORE_WAIT_VIDEO) && release_ready)
            release_ready_seen <= 1'b1;

        case (state)
            IDLE: begin
                write_start <= 1'b0;
                read_start <= 1'b0;
                wait_counter <= 20'd0;
                irq_retry_count <= 3'd0;
                release_ready_seen <= 1'b0;
                if (do_save) begin
                    save_frame_seen <= 1'b0;
                    state <= SAVE_WAIT_SAFE;
                end else if (do_restore) begin
                    restore_frame_count <= 2'd0;
                    restore_begin <= 1'b1;
                    state <= RESTORE_WAIT_SAFE;
                end
            end

            SAVE_WAIT_SAFE: begin
                if (safe_point) begin
                    wait_counter <= 20'd0;
                    state <= SAVE_WAIT_IRQ;
                end
            end

            SAVE_WAIT_IRQ: begin
                if (
                    cpu_iack && (cpu_addr[3:1] == 3'b111) &&
                    !cpu_lds_n
                ) begin
                    wait_counter <= 20'd0;
                    state <= SAVE_WAIT_SSP;
                end else if (wait_counter >= IRQ_TIMEOUT) begin
                    wait_counter <= 20'd0;
                    if (&irq_retry_count)
                        state <= IDLE;
                    else begin
                        irq_retry_count <= irq_retry_count + 3'd1;
                        state <= SAVE_RETRY;
                    end
                end else begin
                    wait_counter <= wait_counter + 20'd1;
                end
            end

            SAVE_RETRY: begin
                if (v_count != 9'd240)
                    state <= SAVE_WAIT_SAFE;
            end

            SAVE_WAIT_SSP: begin
                if (
                    cpu_ack && !cpu_rw &&
                    (cpu_addr == 24'hff0000)
                )
                    saved_ssp[31:16] <= cpu_dout;
                if (
                    cpu_ack && !cpu_rw &&
                    (cpu_addr == 24'hff0002)
                ) begin
                    saved_ssp[15:0] <= cpu_dout;
                    wait_counter <= 20'd0;
                    state <= SAVE_WAIT_HOLD;
                end
            end

            SAVE_WAIT_HOLD: begin
                if (core_held) begin
                    write_start <= 1'b1;
                    state <= SAVE_WAIT_STREAM;
                end else if (wait_counter >= HOLD_TIMEOUT) begin
                    state <= SAVE_WAIT_EXIT;
                end else begin
                    wait_counter <= wait_counter + 20'd1;
                end
            end

            SAVE_WAIT_STREAM: begin
                if (stream_busy && write_start)
                    write_start <= 1'b0;
                else if (!stream_busy && !write_start) begin
                    save_frame_seen <= 1'b0;
                    state <= SAVE_WAIT_EXIT;
                end
            end

            SAVE_WAIT_EXIT: begin
                if (
                    cpu_ack && cpu_rw && (cpu_fc == 3'b110) &&
                    (cpu_addr[23:8] != 16'hff00)
                )
                    state <= SAVE_WAIT_VIDEO;
            end

            SAVE_WAIT_VIDEO: begin
                if (video_release_point && save_frame_seen)
                    state <= IDLE;
            end

            RESTORE_WAIT_SAFE: begin
                if (safe_point) begin
                    wait_counter <= 20'd0;
                    state <= RESTORE_WAIT_HOLD;
                end
            end

            RESTORE_WAIT_HOLD: begin
                if (core_held) begin
                    read_start <= 1'b1;
                    state <= RESTORE_WAIT_STREAM;
                end else if (wait_counter >= HOLD_TIMEOUT) begin
                    state <= RESTORE_WAIT_VIDEO;
                end else begin
                    wait_counter <= wait_counter + 20'd1;
                end
            end

            RESTORE_WAIT_STREAM: begin
                if (stream_busy && read_start)
                    read_start <= 1'b0;
                else if (!stream_busy && !read_start) begin
                    restore_frame_count <= 2'd0;
                    if (format_valid && identity_valid) begin
                        reset_counter <= 8'd0;
                        release_ready_seen <= 1'b0;
                        restore_commit <= 1'b1;
                        state <= RESTORE_HOLD_RESET;
                    end else begin
                        state <= RESTORE_WAIT_VIDEO;
                    end
                end
            end

            RESTORE_HOLD_RESET: begin
                reset_counter <= reset_counter + 8'd1;
                if (&reset_counter)
                    state <= RESTORE_WAIT_RESET;
            end

            RESTORE_WAIT_RESET: begin
                if (
                    cpu_ack && cpu_rw && (cpu_fc == 3'b110) &&
                    (cpu_addr[23:8] != 16'hff00) &&
                    (cpu_addr >= 24'h000008)
                ) begin
                    restore_frame_count <= 2'd0;
                    state <= RESTORE_WAIT_VIDEO;
                end
            end

            RESTORE_WAIT_VIDEO: begin
                if (
                    video_release_point &&
                    (restore_frame_count >= 2'd2) &&
                    (release_ready || release_ready_seen)
                )
                    state <= IDLE;
            end

            default: begin
                state <= IDLE;
                write_start <= 1'b0;
                read_start <= 1'b0;
            end
        endcase
    end
end

endmodule
