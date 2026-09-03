// One GP9001 tile-plane line builder. It renders all three 32x32, 16x16
// tilemaps into a double-buffered 320-pixel line using a held graphics-ROM
// request. The object layer is composed after all three tile layers.
module fixeight_gp9001_tile_line (
    input              clk,
    input              rst,
    input              start,
    input              commit,
    input      [8:0]   target_y,
    input              target_epoch,
    input      [127:0] scrolls,
    input      [7:0]   scroll_flip,

    output reg         busy,
    output reg         done,
    output reg         deadline_miss,
    output reg [15:0]  last_build_cycles,

    output reg [12:0]  vram_addr,
    input      [15:0]  vram_data,

    output             gfx_req,
    output     [21:0]  gfx_addr,
    input      [15:0]  gfx_data,
    input              gfx_ok,

    input      [8:0]   scan_x,
    output     [14:0]  scan_pixel,
    output reg [8:0]   scan_y,
    output reg         scan_epoch,
    output reg         scan_valid
);

localparam [4:0] ST_IDLE          = 5'd0;
localparam [4:0] ST_CLEAR         = 5'd1;
localparam [4:0] ST_LAYER_SETUP   = 5'd2;
localparam [4:0] ST_ATTR_SET      = 5'd3;
localparam [4:0] ST_ATTR_WAIT     = 5'd4;
localparam [4:0] ST_ATTR_CAPTURE  = 5'd5;
localparam [4:0] ST_CODE_WAIT     = 5'd6;
localparam [4:0] ST_CODE_CAPTURE  = 5'd7;
localparam [4:0] ST_GFX_LO        = 5'd8;
localparam [4:0] ST_GFX_GAP       = 5'd9;
localparam [4:0] ST_GFX_HI        = 5'd10;
localparam [4:0] ST_FETCH_STORE   = 5'd11;
localparam [4:0] ST_COMP_READ     = 5'd12;
localparam [4:0] ST_COMP_WAIT     = 5'd13;
localparam [4:0] ST_COMP_WRITE    = 5'd14;
localparam [4:0] ST_DONE          = 5'd15;

reg [4:0] state = ST_IDLE;
reg [1:0] layer = 2'd0;
reg [5:0] group = 6'd0;
reg [2:0] pixel = 3'd0;
reg [8:0] clear_x = 9'd0;
reg [8:0] target_y_latched = 9'd0;
reg target_epoch_latched = 1'b0;
reg [127:0] scrolls_latched = 128'd0;
reg [7:0] scroll_flip_latched = 8'd0;
reg [8:0] source_x0 = 9'd0;
reg [2:0] source_offset = 3'd0;
reg [8:0] fetch_source_x = 9'd0;
reg       fetch_chunk = 1'b0;
reg [15:0] fetch_attr = 16'd0;
reg [15:0] fetch_code = 16'd0;
reg [15:0] fetch_lo = 16'd0;
reg [31:0] fetch_pixels = 32'd0;
reg [15:0] chunk0_attr = 16'd0;
reg [15:0] chunk1_attr = 16'd0;
reg [31:0] chunk0_pixels = 32'd0;
reg [31:0] chunk1_pixels = 32'd0;
reg        active_bank = 1'b0;
reg        build_bank = 1'b1;
reg        build_pending = 1'b0;
reg [15:0] build_cycle_count = 16'd0;

function automatic [8:0] layer_x_origin;
    input [1:0] which;
    begin
        case (which)
            2'd0: layer_x_origin = 9'd42;
            2'd1: layer_x_origin = 9'd40;
            default: layer_x_origin = 9'd38;
        endcase
    end
endfunction

function automatic [12:0] layer_base;
    input [1:0] which;
    begin
        case (which)
            2'd0: layer_base = 13'h0000;
            2'd1: layer_base = 13'h0800;
            default: layer_base = 13'h1000;
        endcase
    end
endfunction

function automatic [31:0] decode_eight;
    input [15:0] lower_planes;
    input [15:0] upper_planes;
    integer i;
    reg [2:0] bit_index;
    begin
        decode_eight = 32'd0;
        for (i = 0; i < 8; i = i + 1) begin
            bit_index = 3'd7 - i[2:0];
            decode_eight[(i * 4) +: 4] = {
                upper_planes[bit_index],
                upper_planes[8 + bit_index],
                lower_planes[bit_index],
                lower_planes[8 + bit_index]
            };
        end
    end
endfunction

function automatic [3:0] packed_pixel;
    input [31:0] pixels;
    input [2:0] index;
    begin
        case (index)
            3'd0: packed_pixel = pixels[3:0];
            3'd1: packed_pixel = pixels[7:4];
            3'd2: packed_pixel = pixels[11:8];
            3'd3: packed_pixel = pixels[15:12];
            3'd4: packed_pixel = pixels[19:16];
            3'd5: packed_pixel = pixels[23:20];
            3'd6: packed_pixel = pixels[27:24];
            default: packed_pixel = pixels[31:28];
        endcase
    end
endfunction

function automatic [21:0] tile_gfx_addr;
    input [15:0] code;
    input [3:0] py;
    input       right_half;
    input       upper_planes;
    reg [4:0] word_in_tile;
    reg [20:0] tile_offset;
    begin
        word_in_tile = (py[3] ? (5'd16 + {2'b00, py[2:0]}) :
                                      {2'b00, py[2:0]}) +
                       (right_half ? 5'd8 : 5'd0);
        // TP-026 has 32K 16x16 tiles. Each 2 MiB graphics half supplies
        // two bitplanes and occupies 0x100000 words in SDRAM.
        tile_offset = {1'b0, code[14:0], 5'b00000} + word_in_tile;
        tile_gfx_addr = {1'b0, tile_offset} +
                        (upper_planes ? 22'h100000 : 22'h000000);
    end
endfunction

reg [15:0] layer_scroll_x;
reg [15:0] layer_scroll_y;
always @* begin
    case (layer)
        2'd0: begin
            layer_scroll_x = scrolls_latched[15:0];
            layer_scroll_y = scrolls_latched[31:16];
        end
        2'd1: begin
            layer_scroll_x = scrolls_latched[47:32];
            layer_scroll_y = scrolls_latched[63:48];
        end
        default: begin
            layer_scroll_x = scrolls_latched[79:64];
            layer_scroll_y = scrolls_latched[95:80];
        end
    endcase
end
wire layer_flip_x = scroll_flip_latched[{layer, 1'b0}];
wire layer_flip_y = scroll_flip_latched[{layer, 1'b1}];
wire [8:0] source_start_sum = layer_flip_x ?
    layer_scroll_x[8:0] - layer_x_origin(layer) :
    layer_scroll_x[8:0] + layer_x_origin(layer);
wire [8:0] source_y = layer_flip_y ?
    layer_scroll_y[8:0] - target_y_latched + 9'd495 :
    target_y_latched + layer_scroll_y[8:0] + 9'd17;
wire [12:0] fetch_pair_addr = layer_base(layer) +
    {2'b00, source_y[8:4], fetch_source_x[8:4], 1'b0};
wire [21:0] fetch_gfx_lo_addr = tile_gfx_addr(
    fetch_code, source_y[3:0], fetch_source_x[3], 1'b0);
wire [21:0] fetch_gfx_hi_addr = tile_gfx_addr(
    fetch_code, source_y[3:0], fetch_source_x[3], 1'b1);

assign gfx_req = (state == ST_GFX_LO) || (state == ST_GFX_HI);
assign gfx_addr = (state == ST_GFX_HI) ?
                  fetch_gfx_hi_addr : fetch_gfx_lo_addr;

wire [3:0] forward_source_position =
    {1'b0, source_offset} + {1'b0, pixel};
wire reverse_uses_chunk1 = pixel > source_offset;
wire [2:0] reverse_source_index = source_offset - pixel;
wire source_uses_chunk1 = layer_flip_x ?
    reverse_uses_chunk1 : forward_source_position[3];
wire [2:0] source_pixel_index = layer_flip_x ?
    reverse_source_index : forward_source_position[2:0];
wire [3:0] candidate_pen = source_uses_chunk1 ?
    packed_pixel(chunk1_pixels, source_pixel_index) :
    packed_pixel(chunk0_pixels, source_pixel_index);
wire [15:0] candidate_attr = source_uses_chunk1 ?
                             chunk1_attr : chunk0_attr;
wire [3:0] candidate_priority = candidate_attr[11:8] & 4'he;
wire [14:0] candidate_pixel = {
    candidate_priority, candidate_attr[6:0], candidate_pen
};

wire [8:0] compose_addr = {group, 3'b000} + pixel;
wire [8:0] line_addr = (state == ST_CLEAR) ? clear_x : compose_addr;
wire [14:0] line_bank0_build_q;
wire [14:0] line_bank1_build_q;
wire [14:0] line_bank0_scan_q;
wire [14:0] line_bank1_scan_q;
wire [14:0] line_build_q = build_bank ?
                              line_bank1_build_q : line_bank0_build_q;
wire candidate_wins = (candidate_pen != 4'h0) &&
                      ((layer == 2'd0) ||
                       (candidate_priority >= line_build_q[14:11]));
wire line_write = (state == ST_CLEAR) ||
                  ((state == ST_COMP_WRITE) && candidate_wins);
wire [14:0] line_write_data = (state == ST_CLEAR) ?
                              15'd0 : candidate_pixel;

assign scan_pixel = active_bank ? line_bank1_scan_q : line_bank0_scan_q;

jtframe_dual_ram #(.DW(15), .AW(9)) u_line_bank0 (
    .clk0  (clk),
    .data0 (line_write_data),
    .addr0 (line_addr),
    .we0   (line_write && !build_bank),
    .q0    (line_bank0_build_q),
    .clk1  (clk),
    .data1 (15'd0),
    .addr1 (scan_x),
    .we1   (1'b0),
    .q1    (line_bank0_scan_q)
);

jtframe_dual_ram #(.DW(15), .AW(9)) u_line_bank1 (
    .clk0  (clk),
    .data0 (line_write_data),
    .addr0 (line_addr),
    .we0   (line_write && build_bank),
    .q0    (line_bank1_build_q),
    .clk1  (clk),
    .data1 (15'd0),
    .addr1 (scan_x),
    .we1   (1'b0),
    .q1    (line_bank1_scan_q)
);

always @(posedge clk) begin
    done <= 1'b0;
    deadline_miss <= 1'b0;

    if (rst) begin
        state <= ST_IDLE;
        busy <= 1'b0;
        layer <= 2'd0;
        group <= 6'd0;
        pixel <= 3'd0;
        clear_x <= 9'd0;
        target_y_latched <= 9'd0;
        target_epoch_latched <= 1'b0;
        scrolls_latched <= 128'd0;
        scroll_flip_latched <= 8'd0;
        source_x0 <= 9'd0;
        source_offset <= 3'd0;
        fetch_source_x <= 9'd0;
        fetch_chunk <= 1'b0;
        fetch_attr <= 16'd0;
        fetch_code <= 16'd0;
        fetch_lo <= 16'd0;
        fetch_pixels <= 32'd0;
        chunk0_attr <= 16'd0;
        chunk1_attr <= 16'd0;
        chunk0_pixels <= 32'd0;
        chunk1_pixels <= 32'd0;
        active_bank <= 1'b0;
        build_bank <= 1'b1;
        build_pending <= 1'b0;
        build_cycle_count <= 16'd0;
        last_build_cycles <= 16'd0;
        scan_y <= 9'd0;
        scan_epoch <= 1'b0;
        scan_valid <= 1'b0;
        vram_addr <= 13'd0;
    end else begin
        if (commit && build_pending) begin
            active_bank <= build_bank;
            scan_y <= target_y_latched;
            scan_epoch <= target_epoch_latched;
            scan_valid <= 1'b1;
            build_pending <= 1'b0;
        end else if (commit && scan_valid) begin
            deadline_miss <= 1'b1;
        end

        if (start && (state != ST_IDLE))
            deadline_miss <= 1'b1;

        if (state != ST_IDLE)
            build_cycle_count <= build_cycle_count + 16'd1;

        case (state)
            ST_IDLE: begin
                busy <= 1'b0;
                if (start) begin
                    busy <= 1'b1;
                    build_bank <= (commit && build_pending) ?
                                  active_bank : ~active_bank;
                    target_y_latched <= target_y;
                    target_epoch_latched <= target_epoch;
                    scrolls_latched <= scrolls;
                    scroll_flip_latched <= scroll_flip;
                    clear_x <= 9'd0;
                    build_cycle_count <= 16'd0;
                    state <= ST_CLEAR;
                end
            end

            ST_CLEAR: begin
                if (clear_x == 9'd319) begin
                    layer <= 2'd0;
                    group <= 6'd0;
                    state <= ST_LAYER_SETUP;
                end else begin
                    clear_x <= clear_x + 9'd1;
                end
            end

            ST_LAYER_SETUP: begin
                source_x0 <= {source_start_sum[8:3], 3'b000};
                source_offset <= source_start_sum[2:0];
                fetch_source_x <= {source_start_sum[8:3], 3'b000};
                fetch_chunk <= 1'b0;
                state <= ST_ATTR_SET;
            end

            ST_ATTR_SET: begin
                vram_addr <= fetch_pair_addr;
                state <= ST_ATTR_WAIT;
            end

            ST_ATTR_WAIT: state <= ST_ATTR_CAPTURE;

            ST_ATTR_CAPTURE: begin
                fetch_attr <= vram_data;
                vram_addr <= fetch_pair_addr + 13'd1;
                state <= ST_CODE_WAIT;
            end

            ST_CODE_WAIT: state <= ST_CODE_CAPTURE;

            ST_CODE_CAPTURE: begin
                fetch_code <= vram_data;
                state <= ST_GFX_LO;
            end

            ST_GFX_LO: begin
                if (gfx_ok) begin
                    fetch_lo <= gfx_data;
                    state <= ST_GFX_GAP;
                end
            end

            ST_GFX_GAP: state <= ST_GFX_HI;

            ST_GFX_HI: begin
                if (gfx_ok) begin
                    fetch_pixels <= decode_eight(fetch_lo, gfx_data);
                    state <= ST_FETCH_STORE;
                end
            end

            ST_FETCH_STORE: begin
                if (!fetch_chunk) begin
                    chunk0_attr <= fetch_attr;
                    chunk0_pixels <= fetch_pixels;
                    fetch_source_x <= layer_flip_x ?
                        fetch_source_x - 9'd8 : fetch_source_x + 9'd8;
                    fetch_chunk <= 1'b1;
                    state <= ST_ATTR_SET;
                end else begin
                    chunk1_attr <= fetch_attr;
                    chunk1_pixels <= fetch_pixels;
                    pixel <= 3'd0;
                    state <= (layer == 2'd0) ?
                             ST_COMP_WRITE : ST_COMP_READ;
                end
            end

            ST_COMP_READ: state <= ST_COMP_WAIT;
            ST_COMP_WAIT: state <= ST_COMP_WRITE;

            ST_COMP_WRITE: begin
                if (pixel != 3'd7) begin
                    pixel <= pixel + 3'd1;
                    state <= (layer == 2'd0) ?
                             ST_COMP_WRITE : ST_COMP_READ;
                end else if (group != 6'd39) begin
                    group <= group + 6'd1;
                    source_x0 <= layer_flip_x ?
                        source_x0 - 9'd8 : source_x0 + 9'd8;
                    chunk0_attr <= chunk1_attr;
                    chunk0_pixels <= chunk1_pixels;
                    fetch_source_x <= layer_flip_x ?
                        source_x0 - 9'd16 : source_x0 + 9'd16;
                    fetch_chunk <= 1'b1;
                    state <= ST_ATTR_SET;
                end else if (layer != 2'd2) begin
                    layer <= layer + 2'd1;
                    group <= 6'd0;
                    state <= ST_LAYER_SETUP;
                end else begin
                    state <= ST_DONE;
                end
            end

            ST_DONE: begin
                build_pending <= 1'b1;
                last_build_cycles <= build_cycle_count;
                busy <= 1'b0;
                done <= 1'b1;
                state <= ST_IDLE;
            end

            default: state <= ST_IDLE;
        endcase
    end
end

endmodule
