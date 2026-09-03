// SPDX-License-Identifier: BSD-3-Clause
// Schema-1 scalar owners. Large RAM/device owners remain beside their state.
module fixeight_state_clients (
    input  logic         clk,
    input  logic         reset,
    input  logic         restore_begin,
    input  logic         restore_capture,
    input  logic         restore_commit,
    input  logic         state_hold,

    input  logic         image_identity_valid,
    input  logic [3:0]   image_set_id,
    input  logic [31:0]  image_eeprom_crc32,
    input  logic [26:0]  image_aggregate_length,
    input  logic [255:0] image_aggregate_sha256,

    input  logic         current_irq4,
    input  logic [7:0]   current_coin_control,
    input  logic         current_v25_release,
    input  logic         current_hs_ready,
    input  logic [3:0]   current_controller_state,
    input  logic [31:0]  saved_ssp,
    input  logic [15:0]  current_p3,
    input  logic [15:0]  current_system,
    input  logic [9:0]   current_h_count,
    input  logic [8:0]   current_v_count,
    input  logic         current_frame_epoch,

    input  logic [63:0]  ss_data,
    input  logic [31:0]  ss_addr,
    input  logic [7:0]   ss_select,
    input  logic         ss_write,
    input  logic         ss_read,
    input  logic         ss_query,
    output logic [63:0]  ss_data_out,
    output logic         ss_ack,

    output logic         restore_identity_valid,
    output logic [31:0]  restore_ssp,
    output logic         restore_irq4,
    output logic [7:0]   restore_coin_control,
    output logic         restore_v25_release,
    output logic         restore_hs_ready_known,
    output logic         restore_hs_ready,
    output logic [15:0]  restore_prev_p3,
    output logic [15:0]  restore_prev_system,
    output logic [9:0]   restore_h_count,
    output logic [8:0]   restore_v_count,
    output logic         restore_frame_epoch
);

localparam logic [63:0] BOARD_TAG = 64'h5450_3032_365f_3031; // "TP026_01"
localparam logic [15:0] SOUND_POLICY = 16'd1;

logic [6:0] identity_seen;
logic [63:0] restored_identity [0:6];
logic [15:0] prev_p3;
logic [15:0] prev_system;

wire global_selected = ss_select == 8'd0;
wire m68k_selected = ss_select == 8'd1;
wire input_selected = ss_select == 8'd13;
wire video_selected = ss_select == 8'd14;
wire selected =
    global_selected || m68k_selected ||
    input_selected || video_selected;

function automatic logic [63:0] global_word(input logic [2:0] index);
    begin
        case (index)
            3'd0: global_word = BOARD_TAG;
            3'd1: global_word = {
                28'd0, image_set_id, image_eeprom_crc32
            };
            3'd2: global_word = {
                21'd0, image_aggregate_length, SOUND_POLICY
            };
            3'd3: global_word = image_aggregate_sha256[255:192];
            3'd4: global_word = image_aggregate_sha256[191:128];
            3'd5: global_word = image_aggregate_sha256[127:64];
            3'd6: global_word = image_aggregate_sha256[63:0];
            default: global_word = {
                // Optional HS readiness extension; existing word/chunk sizes
                // and the seven immutable identity words remain unchanged.
                16'h4853, 5'd0, current_hs_ready,
                current_v25_release,
                current_coin_control,
                current_irq4,
                28'd0,
                current_controller_state
            };
        endcase
    end
endfunction

wire identity_words_match =
    image_identity_valid &&
    (identity_seen == 7'h7f) &&
    (restored_identity[0] == BOARD_TAG) &&
    (restored_identity[1] ==
        {28'd0, image_set_id, image_eeprom_crc32}) &&
    (restored_identity[2] ==
        {21'd0, image_aggregate_length, SOUND_POLICY}) &&
    (restored_identity[3] == image_aggregate_sha256[255:192]) &&
    (restored_identity[4] == image_aggregate_sha256[191:128]) &&
    (restored_identity[5] == image_aggregate_sha256[127:64]) &&
    (restored_identity[6] == image_aggregate_sha256[63:0]);

always_ff @(posedge clk) begin
    if (reset) begin
        ss_data_out <= 64'd0;
        ss_ack <= 1'b0;
        identity_seen <= 7'd0;
        restore_identity_valid <= 1'b0;
        restore_ssp <= 32'd0;
        restore_irq4 <= 1'b0;
        restore_coin_control <= 8'd0;
        restore_v25_release <= 1'b0;
        restore_hs_ready_known <= 1'b0;
        restore_hs_ready <= 1'b0;
        restore_prev_p3 <= 16'd0;
        restore_prev_system <= 16'd0;
        restore_h_count <= 10'd0;
        restore_v_count <= 9'd240;
        restore_frame_epoch <= 1'b0;
        prev_p3 <= 16'd0;
        prev_system <= 16'd0;
    end else begin
        ss_ack <= 1'b0;

        if (!state_hold) begin
            prev_p3 <= current_p3;
            prev_system <= current_system;
        end

        if (restore_begin) begin
            identity_seen <= 7'd0;
            restore_identity_valid <= 1'b0;
            restore_ssp <= 32'd0;
            restore_irq4 <= 1'b0;
            restore_coin_control <= 8'd0;
            restore_v25_release <= 1'b0;
            restore_hs_ready_known <= 1'b0;
            restore_hs_ready <= 1'b0;
            restore_prev_p3 <= 16'd0;
            restore_prev_system <= 16'd0;
            restore_h_count <= 10'd0;
            restore_v_count <= 9'd240;
            restore_frame_epoch <= 1'b0;
        end

        if (restore_commit && restore_identity_valid) begin
            prev_p3 <= restore_prev_p3;
            prev_system <= restore_prev_system;
        end

        if (selected && ss_query) begin
            if (global_selected)
                ss_data_out <= {8'd0, 22'd0, 2'd3, 32'd8};
            else
                ss_data_out <= {
                    ss_select, 22'd0, 2'd3, 32'd1
                };
            ss_ack <= 1'b1;
        end else if (selected && ss_read) begin
            if (global_selected && (ss_addr < 32'd8))
                ss_data_out <= global_word(ss_addr[2:0]);
            else if (m68k_selected && (ss_addr == 32'd0))
                ss_data_out <= {32'd0, saved_ssp};
            else if (input_selected && (ss_addr == 32'd0))
                ss_data_out <= {32'd0, prev_system, prev_p3};
            else if (video_selected && (ss_addr == 32'd0))
                ss_data_out <= {
                    43'd0, current_frame_epoch,
                    current_v_count, current_h_count, 1'b1
                };
            else
                ss_data_out <= 64'd0;
            ss_ack <=
                (global_selected && (ss_addr < 32'd8)) ||
                (!global_selected && (ss_addr == 32'd0));
        end else if (
            selected && ss_write && restore_capture && state_hold
        ) begin
            if (global_selected && (ss_addr < 32'd8)) begin
                if (ss_addr < 32'd7) begin
                    restored_identity[ss_addr[2:0]] <= ss_data;
                    identity_seen[ss_addr[2:0]] <= 1'b1;
                end else begin
                    restore_v25_release <= ss_data[41];
                    restore_hs_ready_known <=
                        identity_words_match &&
                        ss_data[63:48] == 16'h4853 && ss_data[47:43] == 5'd0;
                    restore_hs_ready <= identity_words_match && ss_data[42];
                    restore_coin_control <= ss_data[40:33];
                    restore_irq4 <= ss_data[32];
                    restore_identity_valid <= identity_words_match;
                end
                ss_ack <= 1'b1;
            end else if (m68k_selected && (ss_addr == 32'd0)) begin
                if (restore_identity_valid)
                    restore_ssp <= ss_data[31:0];
                ss_ack <= 1'b1;
            end else if (input_selected && (ss_addr == 32'd0)) begin
                if (restore_identity_valid) begin
                    restore_prev_p3 <= ss_data[15:0];
                    restore_prev_system <= ss_data[31:16];
                end
                ss_ack <= 1'b1;
            end else if (video_selected && (ss_addr == 32'd0)) begin
                if (restore_identity_valid) begin
                    restore_h_count <= ss_data[10:1];
                    restore_v_count <= ss_data[19:11];
                    restore_frame_epoch <= ss_data[20];
                end
                ss_ack <= 1'b1;
            end
        end
    end
end

endmodule
