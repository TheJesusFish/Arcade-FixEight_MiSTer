// 93C46 in 64x16 organization with a sequential save-state aperture.
module fixeight_eeprom #(
    parameter integer WRITE_BUSY_CYCLES = 4
) (
    input  logic        clk,
    input  logic        reset,

    input  logic        serial_cs,
    input  logic        serial_clk,
    input  logic        serial_di,
    output logic        serial_do,

    input  logic        seed_we,
    input  logic [5:0]  seed_addr,
    input  logic [15:0] seed_data,

    input  logic        state_hold,
    input  logic        state_we,
    input  logic [6:0]  state_addr,
    input  logic [15:0] state_wdata,
    output logic [15:0] state_rdata
);

localparam logic [2:0] MODE_IDLE    = 3'd0;
localparam logic [2:0] MODE_COMMAND = 3'd1;
localparam logic [2:0] MODE_READ    = 3'd2;
localparam logic [2:0] MODE_WRITE   = 3'd3;
localparam logic [2:0] MODE_WRAL    = 3'd4;
localparam logic [2:0] MODE_BUSY    = 3'd5;
localparam logic [2:0] MODE_DONE    = 3'd6;
localparam logic [2:0] MODE_BULK    = 3'd7;

logic [15:0] memory [0:63];
logic        write_enabled;
logic        cs_q;
logic        serial_clk_q;
logic        di_q;
logic        do_q;
logic [2:0]  mode;
logic [5:0]  command_address;
logic [8:0]  command_shift;
logic [3:0]  command_count;
logic [15:0] data_shift;
logic [4:0]  data_count;
logic [16:0] output_shift;
logic [4:0]  output_count;
logic [7:0]  busy_count;
logic [5:0]  bulk_index;
logic        bulk_write_all;
logic [15:0] bulk_word;

logic cs_rise;
logic cs_fall;
logic serial_rise;
logic [8:0] next_command;
logic [1:0] next_opcode;
logic [5:0] next_address;
logic [15:0] next_data;

assign serial_do = do_q;
assign cs_rise = serial_cs && !cs_q;
assign cs_fall = !serial_cs && cs_q;
assign serial_rise = serial_cs && serial_clk && !serial_clk_q;
assign next_command = {command_shift[7:0], serial_di};
assign next_opcode = next_command[7:6];
assign next_address = next_command[5:0];
assign next_data = {data_shift[14:0], serial_di};

always_comb begin
    state_rdata = 16'h0000;
    if (state_addr < 7'd64) begin
        state_rdata = memory[state_addr[5:0]];
    end else begin
        case (state_addr)
            7'd64: state_rdata = {
                2'b00, mode, write_enabled, cs_q, serial_clk_q, di_q, do_q,
                command_address
            };
            7'd65: state_rdata = {
                busy_count, command_count, data_count[3:0]
            };
            7'd66: state_rdata = {
                1'b0, data_count[4], output_count, command_shift
            };
            7'd67: state_rdata = data_shift;
            7'd68: state_rdata = output_shift[15:0];
            7'd69: state_rdata = {15'd0, output_shift[16]};
            7'd70: state_rdata = bulk_word;
            7'd71: state_rdata = {
                9'd0, bulk_write_all, bulk_index
            };
            default: state_rdata = 16'h0000;
        endcase
    end
end

always_ff @(posedge clk) begin
    if (reset) begin
        // MRA seed bytes arrive while the rest of the board is intentionally
        // held in reset. Protocol state resets, but explicit seed writes must
        // still initialize the nonvolatile array.
        if (seed_we)
            memory[seed_addr] <= seed_data;
        write_enabled <= 1'b0;
        cs_q <= 1'b0;
        serial_clk_q <= 1'b0;
        di_q <= 1'b0;
        do_q <= 1'b1;
        mode <= MODE_IDLE;
        command_address <= 6'd0;
        command_shift <= 9'd0;
        command_count <= 4'd0;
        data_shift <= 16'd0;
        data_count <= 5'd0;
        output_shift <= 17'd0;
        output_count <= 5'd0;
        busy_count <= 8'd0;
        bulk_index <= 6'd0;
        bulk_write_all <= 1'b0;
        bulk_word <= 16'hffff;
    end else if (state_hold) begin
        if (state_we) begin
            if (state_addr < 7'd64) begin
                memory[state_addr[5:0]] <= state_wdata;
            end else begin
                case (state_addr)
                    7'd64: begin
                        mode <= state_wdata[13:11];
                        write_enabled <= state_wdata[10];
                        cs_q <= state_wdata[9];
                        serial_clk_q <= state_wdata[8];
                        di_q <= state_wdata[7];
                        do_q <= state_wdata[6];
                        command_address <= state_wdata[5:0];
                    end
                    7'd65: begin
                        busy_count <= state_wdata[15:8];
                        command_count <= state_wdata[7:4];
                        data_count[3:0] <= state_wdata[3:0];
                    end
                    7'd66: begin
                        data_count[4] <= state_wdata[14];
                        output_count <= state_wdata[13:9];
                        command_shift <= state_wdata[8:0];
                    end
                    7'd67: data_shift <= state_wdata;
                    7'd68: output_shift[15:0] <= state_wdata;
                    7'd69: output_shift[16] <= state_wdata[0];
                    7'd70: bulk_word <= state_wdata;
                    7'd71: begin
                        bulk_write_all <= state_wdata[6];
                        bulk_index <= state_wdata[5:0];
                    end
                    default: begin end
                endcase
            end
        end
    end else begin
        cs_q <= serial_cs;
        serial_clk_q <= serial_clk;
        di_q <= serial_di;

        if (seed_we)
            memory[seed_addr] <= seed_data;

        if (cs_rise) begin
            command_shift <= 9'd0;
            command_count <= 4'd0;
            data_shift <= 16'd0;
            data_count <= 5'd0;
            output_shift <= 17'd0;
            output_count <= 5'd0;
            mode <= MODE_COMMAND;
            do_q <= (busy_count == 0);
        end

        if (cs_fall) begin
            command_shift <= 9'd0;
            command_count <= 4'd0;
            data_shift <= 16'd0;
            data_count <= 5'd0;
            output_shift <= 17'd0;
            output_count <= 5'd0;
            mode <= MODE_IDLE;
            do_q <= 1'b1;
        end

        if (mode == MODE_BULK) begin
            memory[bulk_index] <= bulk_write_all ? bulk_word : 16'hffff;
            if (bulk_index == 6'd63) begin
                bulk_index <= 6'd0;
                busy_count <= WRITE_BUSY_CYCLES;
                do_q <= 1'b0;
                mode <= MODE_BUSY;
            end else begin
                bulk_index <= bulk_index + 6'd1;
            end
        end else if (serial_rise) begin
            if (busy_count != 0) begin
                if (busy_count == 1) begin
                    busy_count <= 8'd0;
                    do_q <= 1'b1;
                    mode <= MODE_COMMAND;
                end else begin
                    busy_count <= busy_count - 8'd1;
                    do_q <= 1'b0;
                    mode <= MODE_BUSY;
                end
            end else begin
                case (mode)
                    MODE_COMMAND: begin
                        if ((command_count != 0) || serial_di) begin
                            command_shift <= next_command;
                            if (command_count == 4'd8) begin
                                command_address <= next_address;
                                command_count <= 4'd9;
                                case (next_opcode)
                                    2'b10: begin
                                        // A 93C46 asserts the zero dummy bit as
                                        // soon as the READ command completes.
                                        // The first following rising clock
                                        // presents data bit 15.
                                        output_shift <= {
                                            memory[next_address], 1'b0
                                        };
                                        output_count <= 5'd16;
                                        mode <= MODE_READ;
                                        do_q <= 1'b0;
                                    end
                                    2'b01: begin
                                        data_shift <= 16'd0;
                                        data_count <= 5'd0;
                                        mode <= MODE_WRITE;
                                    end
                                    2'b11: begin
                                        if (write_enabled) begin
                                            memory[next_address] <= 16'hffff;
                                            busy_count <= WRITE_BUSY_CYCLES;
                                            do_q <= 1'b0;
                                            mode <= MODE_BUSY;
                                        end else begin
                                            mode <= MODE_DONE;
                                        end
                                    end
                                    default: begin
                                        case (next_address[5:4])
                                            2'b00: begin
                                                write_enabled <= 1'b0;
                                                mode <= MODE_DONE;
                                            end
                                            2'b11: begin
                                                write_enabled <= 1'b1;
                                                mode <= MODE_DONE;
                                            end
                                            2'b01: begin
                                                data_shift <= 16'd0;
                                                data_count <= 5'd0;
                                                mode <= MODE_WRAL;
                                            end
                                            default: begin
                                                if (write_enabled) begin
                                                    bulk_index <= 6'd0;
                                                    bulk_write_all <= 1'b0;
                                                    mode <= MODE_BULK;
                                                end else begin
                                                    mode <= MODE_DONE;
                                                end
                                            end
                                        endcase
                                    end
                                endcase
                            end else begin
                                command_count <= command_count + 4'd1;
                            end
                        end
                    end
                    MODE_READ: begin
                        if (output_count != 0) begin
                            do_q <= output_shift[16];
                            output_shift <= {output_shift[15:0], 1'b0};
                            output_count <= output_count - 5'd1;
                        end else begin
                            do_q <= 1'b1;
                        end
                    end
                    MODE_WRITE: begin
                        data_shift <= next_data;
                        if (data_count == 5'd15) begin
                            if (write_enabled) begin
                                memory[command_address] <= next_data;
                                busy_count <= WRITE_BUSY_CYCLES;
                                do_q <= 1'b0;
                                mode <= MODE_BUSY;
                            end else begin
                                mode <= MODE_DONE;
                            end
                            data_count <= 5'd16;
                        end else begin
                            data_count <= data_count + 5'd1;
                        end
                    end
                    MODE_WRAL: begin
                        data_shift <= next_data;
                        if (data_count == 5'd15) begin
                            data_count <= 5'd16;
                            if (write_enabled) begin
                                bulk_word <= next_data;
                                bulk_index <= 6'd0;
                                bulk_write_all <= 1'b1;
                                mode <= MODE_BULK;
                            end else begin
                                mode <= MODE_DONE;
                            end
                        end else begin
                            data_count <= data_count + 5'd1;
                        end
                    end
                    default: do_q <= 1'b1;
                endcase
            end
        end
    end
end

endmodule
