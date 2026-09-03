// FixEight-local save-state adaptor for an existing synchronous RAM port.
// The normal client must be quiescent while the state stream owns the port.
module fixeight_ss_ram_adaptor #(
    parameter integer WIDTH = 16,
    parameter integer ADDR_WIDTH = 10,
    parameter integer WE_WIDTH = (WIDTH + 7) / 8,
    parameter [7:0] SS_IDX = 8'd0,
    parameter [1:0] STREAM_WIDTH = 2'd1
) (
    input                       clk,
    input                       restore_enable,

    input      [WE_WIDTH-1:0]   normal_we,
    input      [ADDR_WIDTH-1:0] normal_addr,
    input      [WIDTH-1:0]      normal_data,

    output     [WE_WIDTH-1:0]   ram_we,
    output     [ADDR_WIDTH-1:0] ram_addr,
    output     [WIDTH-1:0]      ram_data,
    input      [WIDTH-1:0]      ram_q,

    input      [63:0]           ss_data,
    input      [31:0]           ss_addr,
    input      [7:0]            ss_select,
    input                       ss_write,
    input                       ss_read,
    input                       ss_query,
    output reg [63:0]           ss_data_out,
    output reg                  ss_ack
);

localparam [31:0] WORD_COUNT = 32'd1 << ADDR_WIDTH;

wire selected = ss_select == SS_IDX;
wire access = selected && !ss_query && (ss_read || ss_write);

assign ram_addr = access ? ss_addr[ADDR_WIDTH-1:0] : normal_addr;
assign ram_data = access ? ss_data[WIDTH-1:0] : normal_data;
assign ram_we = access ?
                {WE_WIDTH{ss_write && restore_enable}} :
                normal_we;

reg read_delay = 1'b0;

always @(posedge clk) begin
    ss_ack <= 1'b0;

    if (selected && ss_query) begin
        ss_data_out <= {SS_IDX, 22'd0, STREAM_WIDTH, WORD_COUNT};
        ss_ack <= 1'b1;
        read_delay <= 1'b0;
    end else if (access) begin
        if (ss_write) begin
            ss_ack <= 1'b1;
            read_delay <= 1'b0;
        end else if (read_delay) begin
            ss_data_out <= {{(64-WIDTH){1'b0}}, ram_q};
            ss_ack <= 1'b1;
        end else begin
            read_delay <= 1'b1;
        end
    end else begin
        read_delay <= 1'b0;
    end
end

endmodule
