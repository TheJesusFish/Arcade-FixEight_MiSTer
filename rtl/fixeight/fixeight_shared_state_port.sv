// SPDX-License-Identifier: BSD-3-Clause
// Pack the TP-026 32 KiB byte-wide shared RAM into 16,384 state words.
module fixeight_shared_state_port #(
    parameter logic [7:0] SS_IDX = 8'd3
) (
    input  logic        clk,
    input  logic        reset,
    input  logic        restore_enable,

    input  logic        normal_we,
    input  logic [14:0] normal_addr,
    input  logic [7:0]  normal_data,

    output logic        ram_we,
    output logic [14:0] ram_addr,
    output logic [7:0]  ram_data,
    input  logic [7:0]  ram_q,

    input  logic [63:0] ss_data,
    input  logic [31:0] ss_addr,
    input  logic [7:0]  ss_select,
    input  logic        ss_write,
    input  logic        ss_read,
    input  logic        ss_query,
    output logic [63:0] ss_data_out,
    output logic        ss_ack
);

localparam logic [2:0] IDLE             = 3'd0;
localparam logic [2:0] READ_EVEN        = 3'd1;
localparam logic [2:0] READ_ODD_WAIT    = 3'd2;
localparam logic [2:0] READ_ODD_CAPTURE = 3'd3;
localparam logic [2:0] WRITE_ODD        = 3'd4;

logic [2:0] state;
logic [13:0] word_addr;
logic [15:0] write_word;
logic [7:0] even_byte;

wire selected = ss_select == SS_IDX;
wire in_range = ss_addr < 32'd16384;
wire access =
    selected && in_range && !ss_query && (ss_read || ss_write);

always_comb begin
    ram_we = normal_we;
    ram_addr = normal_addr;
    ram_data = normal_data;

    case (state)
        IDLE: begin
            if (access) begin
                ram_addr = {ss_addr[13:0], 1'b0};
                ram_data = ss_data[15:8];
                ram_we = ss_write && restore_enable;
            end
        end

        READ_EVEN:
            ram_addr = {word_addr, 1'b0};

        READ_ODD_WAIT,
        READ_ODD_CAPTURE:
            ram_addr = {word_addr, 1'b1};

        default: begin
            ram_addr = {word_addr, 1'b1};
            ram_data = write_word[7:0];
            ram_we = restore_enable;
        end
    endcase
end

always_ff @(posedge clk) begin
    if (reset) begin
        state <= IDLE;
        word_addr <= 14'd0;
        write_word <= 16'd0;
        even_byte <= 8'd0;
        ss_data_out <= 64'd0;
        ss_ack <= 1'b0;
    end else begin
        ss_ack <= 1'b0;

        case (state)
            IDLE: begin
                if (selected && ss_query) begin
                    ss_data_out <= {SS_IDX, 22'd0, 2'd1, 32'd16384};
                    ss_ack <= 1'b1;
                end else if (access) begin
                    word_addr <= ss_addr[13:0];
                    write_word <= ss_data[15:0];
                    if (ss_write) begin
                        if (restore_enable)
                            state <= WRITE_ODD;
                        else
                            ss_ack <= 1'b1;
                    end else begin
                        state <= READ_EVEN;
                    end
                end
            end

            READ_EVEN: begin
                even_byte <= ram_q;
                state <= READ_ODD_WAIT;
            end

            READ_ODD_WAIT:
                state <= READ_ODD_CAPTURE;

            READ_ODD_CAPTURE: begin
                ss_data_out <= {48'd0, even_byte, ram_q};
                ss_ack <= 1'b1;
                state <= IDLE;
            end

            default: begin
                ss_ack <= 1'b1;
                state <= IDLE;
            end
        endcase
    end
end

endmodule
