// One GP9001 buffered-object line builder. The descriptor walk and pixel
// ordering follow MAME 0.288 draw_sprites(): later equal-priority objects win.
module fixeight_gp9001_object_line #(
    parameter integer MAX_VISIBLE_CHUNKS = 160
) (
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

    output reg [9:0]   object_addr,
    input      [15:0]  object_data,

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
localparam [4:0] ST_DESC_WAIT0    = 5'd2;
localparam [4:0] ST_DESC_CAPTURE0 = 5'd3;
localparam [4:0] ST_DESC_WAIT1    = 5'd4;
localparam [4:0] ST_DESC_CAPTURE1 = 5'd5;
localparam [4:0] ST_DESC_WAIT2    = 5'd6;
localparam [4:0] ST_DESC_CAPTURE2 = 5'd7;
localparam [4:0] ST_DESC_WAIT3    = 5'd8;
localparam [4:0] ST_DESC_CAPTURE3 = 5'd9;
localparam [4:0] ST_DESC_PROCESS  = 5'd10;
localparam [4:0] ST_GFX_LO        = 5'd11;
localparam [4:0] ST_GFX_GAP       = 5'd12;
localparam [4:0] ST_GFX_HI        = 5'd13;
localparam [4:0] ST_DRAW_READ      = 5'd14;
localparam [4:0] ST_DRAW_WRITE     = 5'd16;
localparam [4:0] ST_DONE           = 5'd18;
localparam [4:0] ST_DESC_DECIDE    = 5'd20;

reg [4:0] state = ST_IDLE;
reg [8:0] clear_x = 9'd0;
reg [7:0] descriptor = 8'd0;
reg [15:0] desc_attr = 16'd0;
reg [15:0] desc_code = 16'd0;
reg [15:0] desc_x = 16'd0;
reg [15:0] desc_y = 16'd0;
reg [8:0] old_x = 9'd0;
reg [8:0] old_y = 9'd0;
reg [8:0] target_y_latched = 9'd0;
reg target_epoch_latched = 1'b0;
reg [127:0] scrolls_latched = 128'd0;
reg [7:0] scroll_flip_latched = 8'd0;

reg signed [11:0] sprite_x_base = 12'sd0;
reg [4:0] sprite_x_chunks = 5'd1;
reg [16:0] sprite_code_row = 17'd0;
reg [3:0] sprite_priority = 4'd0;
reg [5:0] sprite_color = 6'd0;
reg sprite_flip_x = 1'b0;
reg [3:0] chunk = 4'd0;
reg [2:0] pixel = 3'd0;
reg [2:0] sprite_row = 3'd0;
reg [15:0] fetch_lo = 16'd0;
reg [31:0] fetch_pixels = 32'd0;

reg process_hits_line = 1'b0;
reg [4:0] process_x_chunks = 5'd1;
reg [16:0] process_code_base = 17'd0;
reg [3:0] process_priority = 4'd0;
reg [5:0] process_color = 6'd0;
reg process_effective_flip_x = 1'b0;
reg signed [11:0] process_x_base = 12'sd0;
reg signed [12:0] process_line_delta = 13'sd0;

reg active_bank = 1'b0;
reg build_bank = 1'b1;
reg build_pending = 1'b0;
reg [15:0] build_cycle_count = 16'd0;
reg [8:0] visible_chunk_count = 9'd0;
reg capacity_overflow = 1'b0;
reg [8:0] pending_draw_addr = 9'd0;
reg [14:0] pending_draw_pixel = 15'd0;
reg pending_draw_visible = 1'b0;
reg pending_draw_flush = 1'b0;

function automatic signed [11:0] wrapped_base;
    input [8:0] coordinate;
    input       local_flip;
    reg [8:0] shifted;
    begin
        shifted = local_flip ? coordinate - 9'd7 : coordinate;
        if ((!local_flip && (shifted >= 9'h180)) ||
            (local_flip && (shifted >= 9'h1c0)))
            wrapped_base = $signed({3'b000, shifted}) - 12'sd512;
        else
            wrapped_base = $signed({3'b000, shifted});
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

function automatic [21:0] sprite_gfx_addr;
    input [16:0] code;
    input [2:0]  py;
    input        upper_planes;
    reg [20:0] local_address;
    begin
        // TP-026 exposes 128K 8x8 sprite elements. MAME's 18-bit descriptor
        // code is reduced modulo that element count.
        local_address = {1'b0, code[16:0], 3'b000} + py;
        sprite_gfx_addr = {1'b0, local_address} +
            (upper_planes ? 22'h100000 : 22'h000000);
    end
endfunction

wire global_flip_x = scroll_flip_latched[6];
wire global_flip_y = scroll_flip_latched[7];
wire [8:0] sprite_x_offset = global_flip_x ? 9'd379 : 9'd460;
wire [8:0] sprite_y_offset = global_flip_y ? 9'd264 : 9'd495;
wire [15:0] sprite_scroll_x_word = scrolls_latched[111:96];
wire [15:0] sprite_scroll_y_word = scrolls_latched[127:112];
wire [8:0] sprite_scroll_x = sprite_scroll_x_word[8:0];
wire [8:0] sprite_scroll_y = sprite_scroll_y_word[8:0];
wire [8:0] desc_raw_x = desc_x[15:7];
wire [8:0] desc_raw_y = desc_y[15:7];
wire [8:0] desc_position_x = desc_attr[14] ?
    old_x + desc_raw_x : desc_raw_x - sprite_scroll_x + sprite_x_offset;
wire [8:0] desc_position_y = desc_attr[14] ?
    old_y + desc_raw_y : desc_raw_y - sprite_scroll_y + sprite_y_offset;
wire desc_local_flip_x = desc_attr[12];
wire desc_local_flip_y = desc_attr[13];
wire desc_effective_flip_x = desc_local_flip_x ^ global_flip_x;
wire desc_effective_flip_y = desc_local_flip_y ^ global_flip_y;
wire [4:0] desc_x_chunks = {1'b0, desc_x[3:0]} + 5'd1;
wire [4:0] desc_y_chunks = {1'b0, desc_y[3:0]} + 5'd1;
wire signed [12:0] target_y_signed = $signed({4'b0000, target_y_latched});
// MAME forms an 18-bit descriptor code and applies modulo 2^16 elements.
wire [16:0] desc_code_base = {desc_attr[0], desc_code};
wire signed [11:0] desc_pre_x = wrapped_base(
    desc_position_x, desc_local_flip_x);
wire signed [11:0] desc_pre_y = wrapped_base(
    desc_position_y, desc_local_flip_y);
wire signed [11:0] desc_base_x = global_flip_x ?
    12'sd320 - desc_pre_x : desc_pre_x;
wire signed [11:0] desc_base_y = global_flip_y ?
    12'sd240 - desc_pre_y : desc_pre_y;
wire signed [12:0] desc_base_y_extended = {desc_base_y[11], desc_base_y};
wire signed [12:0] desc_line_delta = desc_effective_flip_y ?
    desc_base_y_extended + 13'sd7 - target_y_signed :
    target_y_signed - desc_base_y_extended;
wire signed [12:0] desc_height =
    $signed({5'b00000, desc_y_chunks, 3'b000});
wire desc_hits_line = desc_attr[15] &&
    (desc_line_delta >= 13'sd0) && (desc_line_delta < desc_height);
wire [3:0] process_row_chunk = process_line_delta[6:3];
wire [8:0] process_row_code_offset =
    process_row_chunk * process_x_chunks;

wire signed [11:0] chunk_offset = $signed({4'b0000, chunk, 3'b000});
wire signed [11:0] chunk_x = sprite_flip_x ?
    sprite_x_base - chunk_offset : sprite_x_base + chunk_offset;
wire [16:0] current_code = sprite_code_row + chunk;
wire signed [12:0] chunk_x_extended = {chunk_x[11], chunk_x};
wire signed [12:0] chunk_x_end = chunk_x_extended + 13'sd7;
wire chunk_visible = (chunk_x_end >= 13'sd0) &&
                     (chunk_x_extended < 13'sd320);
wire chunk_budget_available =
    visible_chunk_count < MAX_VISIBLE_CHUNKS;
assign gfx_req = ((state == ST_GFX_LO) && chunk_visible &&
                  chunk_budget_available) ||
                 (state == ST_GFX_HI);
assign gfx_addr = sprite_gfx_addr(current_code, sprite_row,
                                  state == ST_GFX_HI);

wire signed [11:0] source_x_offset = sprite_flip_x ?
    $signed({9'b000000000, (3'd7 - pixel)}) :
    $signed({9'b000000000, pixel});
wire signed [11:0] draw_x = chunk_x + source_x_offset;
wire draw_x_visible = (draw_x >= 12'sd0) && (draw_x < 12'sd320);
wire [3:0] candidate_pen = packed_pixel(fetch_pixels, pixel);
wire [14:0] candidate_pixel = {
    sprite_priority, 1'b0, sprite_color, candidate_pen
};

wire [8:0] line_read_addr = (state == ST_CLEAR) ? clear_x : draw_x[8:0];
wire [14:0] line_bank0_build_q;
wire [14:0] line_bank1_build_q;
wire [14:0] line_bank0_scan_q;
wire [14:0] line_bank1_scan_q;
wire [14:0] line_build_q = build_bank ?
    line_bank1_build_q : line_bank0_build_q;
wire pending_draw_wins = pending_draw_visible &&
                         (pending_draw_pixel[3:0] != 4'h0) &&
                         (pending_draw_pixel[14:11] >= line_build_q[14:11]);
wire line_clear_write = state == ST_CLEAR;
wire line_draw_write = ((state == ST_DRAW_WRITE) ||
                        pending_draw_flush) && pending_draw_wins;
wire [8:0] clear_x_odd = clear_x + 9'd1;
wire bank0_clear_port = line_clear_write && !build_bank;
wire bank1_clear_port = line_clear_write && build_bank;
wire bank0_draw_port = line_draw_write && !build_bank;
wire bank1_draw_port = line_draw_write && build_bank;
wire bank0_port1_write = bank0_clear_port || bank0_draw_port;
wire bank1_port1_write = bank1_clear_port || bank1_draw_port;
wire [8:0] bank0_port1_addr = bank0_clear_port ? clear_x_odd :
                             bank0_draw_port ? pending_draw_addr : scan_x;
wire [8:0] bank1_port1_addr = bank1_clear_port ? clear_x_odd :
                             bank1_draw_port ? pending_draw_addr : scan_x;
wire [14:0] bank0_port1_data = bank0_clear_port ? 15'd0 :
                               pending_draw_pixel;
wire [14:0] bank1_port1_data = bank1_clear_port ? 15'd0 :
                               pending_draw_pixel;

assign scan_pixel = active_bank ? line_bank1_scan_q : line_bank0_scan_q;

jtframe_dual_ram #(.DW(15), .AW(9)) u_line_bank0 (
    .clk0  (clk),
    .data0 (15'd0),
    .addr0 (line_read_addr),
    .we0   (line_clear_write && !build_bank),
    .q0    (line_bank0_build_q),
    .clk1  (clk),
    .data1 (bank0_port1_data),
    .addr1 (bank0_port1_addr),
    .we1   (bank0_port1_write),
    .q1    (line_bank0_scan_q)
);

jtframe_dual_ram #(.DW(15), .AW(9)) u_line_bank1 (
    .clk0  (clk),
    .data0 (15'd0),
    .addr0 (line_read_addr),
    .we0   (line_clear_write && build_bank),
    .q0    (line_bank1_build_q),
    .clk1  (clk),
    .data1 (bank1_port1_data),
    .addr1 (bank1_port1_addr),
    .we1   (bank1_port1_write),
    .q1    (line_bank1_scan_q)
);

always @(posedge clk) begin
    done <= 1'b0;
    deadline_miss <= 1'b0;

    if (rst) begin
        state <= ST_IDLE;
        busy <= 1'b0;
        clear_x <= 9'd0;
        descriptor <= 8'd0;
        desc_attr <= 16'd0;
        desc_code <= 16'd0;
        desc_x <= 16'd0;
        desc_y <= 16'd0;
        old_x <= 9'd0;
        old_y <= 9'd0;
        target_y_latched <= 9'd0;
        target_epoch_latched <= 1'b0;
        scrolls_latched <= 128'd0;
        scroll_flip_latched <= 8'd0;
        sprite_x_base <= 12'sd0;
        sprite_x_chunks <= 5'd1;
        sprite_code_row <= 17'd0;
        sprite_priority <= 4'd0;
        sprite_color <= 6'd0;
        sprite_flip_x <= 1'b0;
        chunk <= 4'd0;
        pixel <= 3'd0;
        sprite_row <= 3'd0;
        fetch_lo <= 16'd0;
        fetch_pixels <= 32'd0;
        process_hits_line <= 1'b0;
        process_x_chunks <= 5'd1;
        process_code_base <= 17'd0;
        process_priority <= 4'd0;
        process_color <= 6'd0;
        process_effective_flip_x <= 1'b0;
        process_x_base <= 12'sd0;
        process_line_delta <= 13'sd0;
        active_bank <= 1'b0;
        build_bank <= 1'b1;
        build_pending <= 1'b0;
        build_cycle_count <= 16'd0;
        visible_chunk_count <= 9'd0;
        capacity_overflow <= 1'b0;
        pending_draw_addr <= 9'd0;
        pending_draw_pixel <= 15'd0;
        pending_draw_visible <= 1'b0;
        pending_draw_flush <= 1'b0;
        last_build_cycles <= 16'd0;
        scan_y <= 9'd0;
        scan_epoch <= 1'b0;
        scan_valid <= 1'b0;
        object_addr <= 10'd0;
    end else begin
        pending_draw_flush <= 1'b0;

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
                    descriptor <= 8'd0;
                    old_x <= (scroll_flip[6] ? 9'd379 : 9'd460) -
                             scrolls[104:96];
                    old_y <= (scroll_flip[7] ? 9'd264 : 9'd495) -
                             scrolls[120:112];
                    build_cycle_count <= 16'd0;
                    visible_chunk_count <= 9'd0;
                    capacity_overflow <= 1'b0;
                    state <= ST_CLEAR;
                end
            end

            ST_CLEAR: begin
                if (clear_x == 9'd318) begin
                    object_addr <= 10'd0;
                    state <= ST_DESC_WAIT0;
                end else begin
                    clear_x <= clear_x + 9'd2;
                end
            end

            ST_DESC_WAIT0: begin
                object_addr <= {descriptor, 2'b01};
                state <= ST_DESC_CAPTURE0;
            end
            ST_DESC_CAPTURE0: begin
                desc_attr <= object_data;
                object_addr <= {descriptor, 2'b10};
                state <= ST_DESC_CAPTURE1;
            end
            ST_DESC_WAIT1: state <= ST_DESC_CAPTURE1;
            ST_DESC_CAPTURE1: begin
                desc_code <= object_data;
                object_addr <= {descriptor, 2'b11};
                state <= ST_DESC_CAPTURE2;
            end
            ST_DESC_WAIT2: state <= ST_DESC_CAPTURE2;
            ST_DESC_CAPTURE2: begin
                desc_x <= object_data;
                state <= ST_DESC_CAPTURE3;
            end
            ST_DESC_WAIT3: state <= ST_DESC_CAPTURE3;
            ST_DESC_CAPTURE3: begin
                desc_y <= object_data;
                state <= ST_DESC_PROCESS;
            end

            ST_DESC_PROCESS: begin
                if (desc_attr[15]) begin
                    old_x <= desc_position_x;
                    old_y <= desc_position_y;
                end
                process_hits_line <= desc_hits_line;
                process_x_chunks <= desc_x_chunks;
                process_code_base <= desc_code_base;
                process_priority <= desc_attr[11:8];
                process_color <= desc_attr[7:2];
                process_effective_flip_x <= desc_effective_flip_x;
                process_x_base <= desc_base_x;
                process_line_delta <= desc_line_delta;
                // Object RAM is immutable during a line build. Start the
                // next descriptor read now so its word 0 is waiting once
                // this descriptor has been accepted or rejected.
                if (descriptor != 8'hff)
                    object_addr <= {descriptor + 8'd1, 2'b00};
                state <= ST_DESC_DECIDE;
            end

            ST_DESC_DECIDE: begin
                if (process_hits_line) begin
                    sprite_x_base <= process_x_base;
                    sprite_x_chunks <= process_x_chunks;
                    sprite_code_row <= process_code_base +
                                       process_row_code_offset;
                    sprite_priority <= process_priority;
                    sprite_color <= process_color;
                    sprite_flip_x <= process_effective_flip_x;
                    sprite_row <= process_line_delta[2:0];
                    chunk <= 4'd0;
                    state <= ST_GFX_LO;
                end else if (descriptor == 8'hff) begin
                    state <= ST_DONE;
                end else begin
                    descriptor <= descriptor + 8'd1;
                    object_addr <= {descriptor + 8'd1, 2'b01};
                    state <= ST_DESC_CAPTURE0;
                end
            end

            ST_GFX_LO: begin
                if (!chunk_budget_available) begin
                    capacity_overflow <= 1'b1;
                    state <= ST_DONE;
                end else if (!chunk_visible) begin
                    if ({1'b0, chunk} + 5'd1 < sprite_x_chunks) begin
                        chunk <= chunk + 4'd1;
                        state <= ST_GFX_LO;
                    end else if (descriptor == 8'hff) begin
                        state <= ST_DONE;
                    end else begin
                        descriptor <= descriptor + 8'd1;
                        object_addr <= {descriptor + 8'd1, 2'b01};
                        state <= ST_DESC_CAPTURE0;
                    end
                end else if (gfx_ok) begin
                    fetch_lo <= gfx_data;
                    visible_chunk_count <= visible_chunk_count + 9'd1;
                    state <= ST_GFX_GAP;
                end
            end
            ST_GFX_GAP: state <= ST_GFX_HI;
            ST_GFX_HI: begin
                if (gfx_ok) begin
                    fetch_pixels <= decode_eight(fetch_lo, gfx_data);
                    pixel <= 3'd0;
                    state <= ST_DRAW_READ;
                end
            end

            // Port 0 reads candidate N while port 1 writes candidate N-1.
            ST_DRAW_READ: begin
                pending_draw_addr <= draw_x[8:0];
                pending_draw_pixel <= candidate_pixel;
                pending_draw_visible <= draw_x_visible;
                pixel <= 3'd1;
                state <= ST_DRAW_WRITE;
            end

            ST_DRAW_WRITE: begin
                pending_draw_addr <= draw_x[8:0];
                pending_draw_pixel <= candidate_pixel;
                pending_draw_visible <= draw_x_visible;
                if (pixel != 3'd7) begin
                    pixel <= pixel + 3'd1;
                end else begin
                    // Port 1 writes pixel 7 on the next edge while the
                    // state machine already advances to the next fetch.
                    pending_draw_flush <= 1'b1;
                    if ({1'b0, chunk} + 5'd1 < sprite_x_chunks) begin
                        chunk <= chunk + 4'd1;
                        state <= ST_GFX_LO;
                    end else if (descriptor == 8'hff) begin
                        state <= ST_DONE;
                    end else begin
                        descriptor <= descriptor + 8'd1;
                        object_addr <= {descriptor + 8'd1, 2'b01};
                        state <= ST_DESC_CAPTURE0;
                    end
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
