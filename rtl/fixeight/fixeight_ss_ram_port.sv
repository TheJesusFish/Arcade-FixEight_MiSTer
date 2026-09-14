// FixEight save-state adaptor for an existing synchronous RAM port.
// The normal client must be quiescent while the state stream owns the port.
module fixeight_ss_ram_port #(
    parameter integer WIDTH = 16,
    parameter integer ADDR_WIDTH = 10,
    parameter integer WE_WIDTH = (WIDTH + 7) / 8,
    parameter [7:0] SS_IDX = 8'd0,
    parameter [1:0] STREAM_WIDTH = 2'd1,
    parameter [31:0] SS_BASE = 32'd0,
    parameter [31:0] SS_WORD_COUNT = 32'd1 << ADDR_WIDTH,
    parameter SS_QUERY_OWNER = 1'b1
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

localparam [31:0] LOCAL_WORD_COUNT = 32'd1 << ADDR_WIDTH;

wire selected = ss_select == SS_IDX;
wire in_range =
    (ss_addr >= SS_BASE) && (ss_addr < (SS_BASE + LOCAL_WORD_COUNT));
wire access = selected && in_range && !ss_query && (ss_read || ss_write);
wire [31:0] local_addr = ss_addr - SS_BASE;

localparam [2:0]
    SS_IDLE          = 3'd0,
    SS_WRITE_COMMIT  = 3'd1,
    SS_READ_WAIT     = 3'd2,
    SS_READ_CAPTURE  = 3'd3,
    SS_WAIT_RELEASE  = 3'd4;

reg [2:0] ss_state = SS_IDLE;
reg [ADDR_WIDTH-1:0] ss_addr_latched = {ADDR_WIDTH{1'b0}};
reg [WIDTH-1:0] ss_data_latched = {WIDTH{1'b0}};
reg ss_restore_latched = 1'b0;

wire ss_owns_ram =
    ss_state == SS_WRITE_COMMIT ||
    ss_state == SS_READ_WAIT ||
    ss_state == SS_READ_CAPTURE;

// The shared state bus spans the entire core.  Capture its command beside
// each RAM before driving the synchronous memory port so read_req/state/address
// never form a full-core combinational cone into the block-RAM write enables.
assign ram_addr = ss_owns_ram ? ss_addr_latched : normal_addr;
assign ram_data = ss_owns_ram ? ss_data_latched : normal_data;
assign ram_we = ss_state == SS_WRITE_COMMIT ?
                {WE_WIDTH{ss_restore_latched}} : normal_we;

always @(posedge clk) begin
    ss_ack <= 1'b0;

    case (ss_state)
        SS_IDLE: begin
            if (selected && ss_query && SS_QUERY_OWNER) begin
                ss_data_out <=
                    {SS_IDX, 22'd0, STREAM_WIDTH, SS_WORD_COUNT};
                ss_ack <= 1'b1;
                ss_state <= SS_WAIT_RELEASE;
            end else if (access) begin
                ss_addr_latched <= local_addr[ADDR_WIDTH-1:0];
                ss_data_latched <= ss_data[WIDTH-1:0];
                ss_restore_latched <= restore_enable;
                ss_state <= ss_write ? SS_WRITE_COMMIT : SS_READ_WAIT;
            end
        end

        SS_WRITE_COMMIT: begin
            // ram_we is high throughout the cycle ending at this edge.
            ss_ack <= 1'b1;
            ss_state <= SS_WAIT_RELEASE;
        end

        SS_READ_WAIT: begin
            // Present the registered address for one complete synchronous
            // RAM cycle before sampling its registered output.
            ss_state <= SS_READ_CAPTURE;
        end

        SS_READ_CAPTURE: begin
            ss_data_out <= {{(64-WIDTH){1'b0}}, ram_q};
            ss_ack <= 1'b1;
            ss_state <= SS_WAIT_RELEASE;
        end

        SS_WAIT_RELEASE: begin
            if (!(ss_read || ss_write || ss_query))
                ss_state <= SS_IDLE;
        end

        default: ss_state <= SS_IDLE;
    endcase
end

endmodule
