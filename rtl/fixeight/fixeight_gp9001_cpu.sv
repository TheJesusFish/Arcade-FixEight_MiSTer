// TP-026 GP9001 CPU-facing register, VRAM, and object-buffer behavior.
// Objects use a vblank-frozen bank; the game wrapper freezes tile/scroll state.
module fixeight_gp9001_cpu #(
    parameter [7:0] SS_IDX = 8'd5
) (
    input              clk,
    input              rst,

    input              start,
    input              rw,
    input      [3:0]   addr,
    input      [15:0]  din,
    input      [1:0]   we_mask,
    input              status_bit,

    output reg         busy,
    output reg         done,
    output reg [15:0]  dout,
    output reg         irq_clear,

    input      [12:0]  scan_addr,
    output     [15:0]  scan_dout,
    input              obj_buf_start,
    input      [9:0]   obj_scan_addr,
    output     [15:0]  obj_scan_dout,
    output             obj_buf_busy,
    output reg         obj_buf_miss,
    output     [12:0]  dbg_ptr,
    output     [7:0]   dbg_scroll_select,
    output     [127:0] scrolls,
    output     [7:0]   scroll_flip,
    output reg         vram_write,
    output reg         scroll_write,

    input              ss_hold,
    input              ss_restore_enable,
    input      [63:0]  ss_data,
    input      [31:0]  ss_addr,
    input      [7:0]   ss_select,
    input              ss_write,
    input              ss_read,
    input              ss_query,
    output     [63:0]  ss_data_out,
    output             ss_ack
);

localparam [2:0] ST_IDLE     = 3'd0;
localparam [2:0] ST_DISPATCH = 3'd1;
localparam [2:0] ST_RD_0     = 3'd2;
localparam [2:0] ST_RD_1     = 3'd3;
localparam [2:0] ST_RD_OUT   = 3'd4;
localparam [1:0] OBJ_SYNC_IDLE  = 2'd0;
localparam [1:0] OBJ_SYNC_READ  = 2'd1;
localparam [1:0] OBJ_SYNC_WAIT  = 2'd2;
localparam [1:0] OBJ_SYNC_WRITE = 2'd3;

reg [2:0]  state = ST_IDLE;
reg        req_rw = 1'b0;
reg [3:0]  req_addr = 4'd0;
reg [15:0] req_din = 16'd0;
reg [1:0]  req_we_mask = 2'b00;
reg        req_status_bit = 1'b0;

reg [12:0] ptr = 13'd0;
reg [7:0]  scroll_select = 8'd0;
reg [15:0] scroll_reg [0:7];
reg [7:0]  flip_reg = 8'd0;

reg [12:0] ram_addr = 13'd0;
reg [15:0] ram_din = 16'd0;
reg [1:0]  ram_we = 2'b00;
wire [15:0] ram_dout;

wire [3:0] op = {req_addr[3:2], 2'b00};
wire [12:0] ram_addr_mirrored =
    (ram_addr >= 13'h1c00) ? ram_addr - 13'h0400 : ram_addr;
wire [12:0] scan_addr_mirrored =
    (scan_addr >= 13'h1c00) ? scan_addr - 13'h0400 : scan_addr;

reg         obj_active_bank = 1'b0;
reg         obj_snapshot_valid = 1'b0;
reg         obj_init_active = 1'b1;
reg [9:0]   obj_init_index = 10'd0;
reg [1:0]   obj_sync_state = OBJ_SYNC_IDLE;
reg [9:0]   obj_sync_index = 10'd0;
reg [15:0]  obj_sync_data = 16'd0;
reg         obj_sync_start_pending = 1'b0;
reg         obj_restore_pending = 1'b0;
reg [1:0]   obj_restore_flags = 2'b00;
reg [1023:0] obj_staging_dirty_lo = 1024'd0;
reg [1023:0] obj_staging_dirty_hi = 1024'd0;

wire [9:0] obj_word_index = ram_addr_mirrored[9:0];
wire obj_staging_bank = ~obj_active_bank;
wire ram_obj_write = (|ram_we) &&
                     (ram_addr_mirrored >= 13'h1800) &&
                     (ram_addr_mirrored <= 13'h1bff);
wire obj_bank0_cpu_write = ram_obj_write &&
                           (!obj_snapshot_valid || !obj_staging_bank);
wire obj_bank1_cpu_write = ram_obj_write &&
                           (!obj_snapshot_valid || obj_staging_bank);
wire obj_init_clear_write = obj_init_active && !ram_obj_write;
wire [1:0] obj_init_clear_mask = {
    !obj_staging_dirty_hi[obj_init_index],
    !obj_staging_dirty_lo[obj_init_index]
};
wire obj_sync_copy_write = (obj_sync_state == OBJ_SYNC_WRITE) &&
                           !ram_obj_write;
wire [1:0] obj_sync_copy_mask = {
    !obj_staging_dirty_hi[obj_sync_index],
    !obj_staging_dirty_lo[obj_sync_index]
};
wire [9:0] obj_bank0_addr = obj_bank0_cpu_write ? obj_word_index :
                            obj_init_active ? obj_init_index : obj_sync_index;
wire [9:0] obj_bank1_addr = obj_bank1_cpu_write ? obj_word_index :
                            obj_init_active ? obj_init_index : obj_sync_index;
wire [15:0] obj_bank0_data = obj_bank0_cpu_write ? ram_din :
                             obj_init_active ? 16'h0000 : obj_sync_data;
wire [15:0] obj_bank1_data = obj_bank1_cpu_write ? ram_din :
                             obj_init_active ? 16'h0000 : obj_sync_data;
wire [1:0] obj_bank0_we = obj_bank0_cpu_write ? ram_we :
                          obj_init_clear_write ? obj_init_clear_mask :
                          (obj_sync_copy_write && !obj_staging_bank) ?
                              obj_sync_copy_mask : 2'b00;
wire [1:0] obj_bank1_we = obj_bank1_cpu_write ? ram_we :
                          obj_init_clear_write ? obj_init_clear_mask :
                          (obj_sync_copy_write && obj_staging_bank) ?
                              obj_sync_copy_mask : 2'b00;
wire [15:0] obj_bank0_copy_dout;
wire [15:0] obj_bank1_copy_dout;
wire [15:0] obj_bank0_scan_dout;
wire [15:0] obj_bank1_scan_dout;
wire [15:0] obj_active_copy_dout = obj_active_bank ?
                                   obj_bank1_copy_dout : obj_bank0_copy_dout;

localparam [31:0] SS_VRAM_BASE = 32'd0;
localparam [31:0] SS_REG_BASE  = 32'd8192;
localparam [31:0] SS_OBJ0_BASE = 32'd8203;
localparam [31:0] SS_OBJ1_BASE = 32'd9227;
localparam [31:0] SS_WORD_COUNT = 32'd10251;

wire ss_selected = ss_select == SS_IDX;
wire ss_reg_selected =
    ss_selected &&
    (ss_addr >= SS_REG_BASE) &&
    (ss_addr < SS_OBJ0_BASE);
wire [31:0] ss_reg_addr = ss_addr - SS_REG_BASE;
wire ss_reg_access = ss_reg_selected && !ss_query &&
                     (ss_read || ss_write);
wire ss_reg_restore_write = ss_reg_access && ss_write &&
                            ss_restore_enable;
reg [63:0] ss_reg_data_out = 64'd0;
reg        ss_reg_ack = 1'b0;
wire [63:0] ss_ram_data_out;
wire [63:0] ss_obj0_data_out;
wire [63:0] ss_obj1_data_out;
wire        ss_ram_ack;
wire        ss_obj0_ack;
wire        ss_obj1_ack;

assign ss_ack = ss_reg_ack || ss_ram_ack || ss_obj0_ack || ss_obj1_ack;
assign ss_data_out = ss_reg_ack ? ss_reg_data_out :
                     ss_ram_ack ? ss_ram_data_out :
                     ss_obj0_ack ? ss_obj0_data_out :
                     ss_obj1_ack ? ss_obj1_data_out :
                     64'd0;

assign obj_scan_dout = obj_active_bank ?
                       obj_bank1_scan_dout : obj_bank0_scan_dout;
assign obj_buf_busy = obj_init_active || obj_sync_start_pending ||
                      (obj_sync_state != OBJ_SYNC_IDLE);

function automatic [15:0] merge_word;
    input [15:0] old_word;
    input [15:0] new_word;
    input [1:0] mask;
    begin
        merge_word = {
            mask[1] ? new_word[15:8] : old_word[15:8],
            mask[0] ? new_word[7:0] : old_word[7:0]
        };
    end
endfunction

wire [15:0] ptr_merged =
    merge_word({3'b000, ptr}, req_din, req_we_mask);
wire [15:0] select_merged =
    merge_word({8'h00, scroll_select}, req_din, req_we_mask);
wire [15:0] scroll_merged =
    merge_word(scroll_reg[scroll_select[2:0]], req_din, req_we_mask);
wire scroll_index_valid = (scroll_select[6:0] <= 7'h07);
wire irq_selector = (scroll_select[6:0] == 7'h0e) ||
                    (scroll_select[6:0] == 7'h0f);

assign dbg_ptr = ptr;
assign dbg_scroll_select = scroll_select;
assign scrolls = {
    scroll_reg[7], scroll_reg[6], scroll_reg[5], scroll_reg[4],
    scroll_reg[3], scroll_reg[2], scroll_reg[1], scroll_reg[0]
};
assign scroll_flip = flip_reg;

// Register chunk: pointer, selector, eight scroll registers, then flip and
// object-bank ownership. Transient request/copy state is idle at quiesce.
always @(posedge clk) begin
    ss_reg_ack <= 1'b0;

    if (ss_selected && ss_query) begin
        ss_reg_data_out <= {SS_IDX, 22'd0, 2'd1, SS_WORD_COUNT};
        ss_reg_ack <= 1'b1;
    end else if (ss_reg_access) begin
        if (ss_write) begin
            ss_reg_ack <= 1'b1;
        end else if (ss_reg_addr == 32'd0) begin
            ss_reg_data_out <= {48'd0, 3'd0, ptr};
            ss_reg_ack <= 1'b1;
        end else if (ss_reg_addr == 32'd1) begin
            ss_reg_data_out <= {56'd0, scroll_select};
            ss_reg_ack <= 1'b1;
        end else if (ss_reg_addr < 32'd10) begin
            ss_reg_data_out <= {
                48'd0,
                scroll_reg[ss_reg_addr[2:0] - 3'd2]
            };
            ss_reg_ack <= 1'b1;
        end else begin
            ss_reg_data_out <= {
                54'd0,
                obj_snapshot_valid,
                obj_active_bank,
                flip_reg
            };
            ss_reg_ack <= 1'b1;
        end
    end
end

wire [12:0] ss_vram_addr;
wire [15:0] ss_vram_data;
wire [1:0]  ss_vram_we;

fixeight_ss_ram_port #(
    .WIDTH        (16),
    .ADDR_WIDTH   (13),
    .WE_WIDTH     (2),
    .SS_IDX       (SS_IDX),
    .STREAM_WIDTH (2'd1),
    .SS_BASE      (SS_VRAM_BASE),
    .SS_WORD_COUNT(SS_WORD_COUNT),
    .SS_QUERY_OWNER(1'b0)
) u_vram_ss (
    .clk            (clk),
    .restore_enable (ss_restore_enable),
    .normal_we      (2'b00),
    .normal_addr    (scan_addr_mirrored),
    .normal_data    (16'd0),
    .ram_we         (ss_vram_we),
    .ram_addr       (ss_vram_addr),
    .ram_data       (ss_vram_data),
    .ram_q          (scan_dout),
    .ss_data        (ss_data),
    .ss_addr        (ss_addr),
    .ss_select      (ss_select),
    .ss_write       (ss_write),
    .ss_read        (ss_read),
    .ss_query       (ss_query),
    .ss_data_out    (ss_ram_data_out),
    .ss_ack         (ss_ram_ack)
);

jtframe_dual_ram16 #(.AW(13)) u_vram (
    .clk0  (clk),
    .data0 (ram_din),
    .addr0 (ram_addr_mirrored),
    .we0   (ram_we),
    .q0    (ram_dout),
    .clk1  (clk),
    .data1 (ss_vram_data),
    .addr1 (ss_vram_addr),
    .we1   (ss_vram_we),
    .q1    (scan_dout)
);

wire [9:0]  ss_obj0_addr;
wire [15:0] ss_obj0_data;
wire [1:0]  ss_obj0_we;

fixeight_ss_ram_port #(
    .WIDTH        (16),
    .ADDR_WIDTH   (10),
    .WE_WIDTH     (2),
    .SS_IDX       (SS_IDX),
    .STREAM_WIDTH (2'd1),
    .SS_BASE      (SS_OBJ0_BASE),
    .SS_WORD_COUNT(SS_WORD_COUNT),
    .SS_QUERY_OWNER(1'b0)
) u_obj0_ss (
    .clk            (clk),
    .restore_enable (ss_restore_enable),
    .normal_we      (2'b00),
    .normal_addr    (obj_scan_addr),
    .normal_data    (16'd0),
    .ram_we         (ss_obj0_we),
    .ram_addr       (ss_obj0_addr),
    .ram_data       (ss_obj0_data),
    .ram_q          (obj_bank0_scan_dout),
    .ss_data        (ss_data),
    .ss_addr        (ss_addr),
    .ss_select      (ss_select),
    .ss_write       (ss_write),
    .ss_read        (ss_read),
    .ss_query       (ss_query),
    .ss_data_out    (ss_obj0_data_out),
    .ss_ack         (ss_obj0_ack)
);

jtframe_dual_ram16 #(.AW(10)) u_obj_bank0 (
    .clk0  (clk),
    .data0 (obj_bank0_data),
    .addr0 (obj_bank0_addr),
    .we0   (obj_bank0_we),
    .q0    (obj_bank0_copy_dout),
    .clk1  (clk),
    .data1 (ss_obj0_data),
    .addr1 (ss_obj0_addr),
    .we1   (ss_obj0_we),
    .q1    (obj_bank0_scan_dout)
);

wire [9:0]  ss_obj1_addr;
wire [15:0] ss_obj1_data;
wire [1:0]  ss_obj1_we;

fixeight_ss_ram_port #(
    .WIDTH        (16),
    .ADDR_WIDTH   (10),
    .WE_WIDTH     (2),
    .SS_IDX       (SS_IDX),
    .STREAM_WIDTH (2'd1),
    .SS_BASE      (SS_OBJ1_BASE),
    .SS_WORD_COUNT(SS_WORD_COUNT),
    .SS_QUERY_OWNER(1'b0)
) u_obj1_ss (
    .clk            (clk),
    .restore_enable (ss_restore_enable),
    .normal_we      (2'b00),
    .normal_addr    (obj_scan_addr),
    .normal_data    (16'd0),
    .ram_we         (ss_obj1_we),
    .ram_addr       (ss_obj1_addr),
    .ram_data       (ss_obj1_data),
    .ram_q          (obj_bank1_scan_dout),
    .ss_data        (ss_data),
    .ss_addr        (ss_addr),
    .ss_select      (ss_select),
    .ss_write       (ss_write),
    .ss_read        (ss_read),
    .ss_query       (ss_query),
    .ss_data_out    (ss_obj1_data_out),
    .ss_ack         (ss_obj1_ack)
);

jtframe_dual_ram16 #(.AW(10)) u_obj_bank1 (
    .clk0  (clk),
    .data0 (obj_bank1_data),
    .addr0 (obj_bank1_addr),
    .we0   (obj_bank1_we),
    .q0    (obj_bank1_copy_dout),
    .clk1  (clk),
    .data1 (ss_obj1_data),
    .addr1 (ss_obj1_addr),
    .we1   (ss_obj1_we),
    .q1    (obj_bank1_scan_dout)
);

// At rising vblank the staging bank becomes the immutable renderer bank.
// The previous bank is refreshed in the background. Dirty bits preserve CPU
// writes that arrive after the swap while the refresh is still in flight.
always @(posedge clk) begin
    if (rst) begin
        obj_active_bank <= 1'b0;
        obj_snapshot_valid <= 1'b0;
        obj_init_active <= 1'b1;
        obj_init_index <= 10'd0;
        obj_sync_state <= OBJ_SYNC_IDLE;
        obj_sync_index <= 10'd0;
        obj_sync_data <= 16'd0;
        obj_sync_start_pending <= 1'b0;
        obj_restore_pending <= 1'b0;
        obj_restore_flags <= 2'b00;
        obj_buf_miss <= 1'b0;
        obj_staging_dirty_lo <= 1024'd0;
        obj_staging_dirty_hi <= 1024'd0;
    end else if (ss_reg_restore_write && (ss_reg_addr == 32'd10)) begin
        obj_restore_pending <= 1'b1;
        obj_restore_flags <= ss_data[9:8];
    end else if (obj_restore_pending) begin
        obj_active_bank <= obj_restore_flags[0];
        obj_snapshot_valid <= obj_restore_flags[1];
        obj_init_active <= 1'b0;
        obj_init_index <= 10'd0;
        obj_sync_state <= OBJ_SYNC_IDLE;
        obj_sync_index <= 10'd0;
        obj_sync_data <= 16'd0;
        obj_sync_start_pending <= 1'b0;
        obj_restore_pending <= 1'b0;
        obj_buf_miss <= 1'b0;
        obj_staging_dirty_lo <= 1024'd0;
        obj_staging_dirty_hi <= 1024'd0;
    end else if (
        !ss_hold || obj_init_active || obj_sync_start_pending ||
        (obj_sync_state != OBJ_SYNC_IDLE)
    ) begin
        obj_buf_miss <= 1'b0;

        if (obj_buf_start && !ss_hold) begin
            if (!obj_init_active && !obj_sync_start_pending &&
                (obj_sync_state == OBJ_SYNC_IDLE)) begin
                obj_active_bank <= ~obj_active_bank;
                obj_sync_start_pending <= 1'b1;
                obj_sync_index <= 10'd0;
                obj_staging_dirty_lo <= 1024'd0;
                obj_staging_dirty_hi <= 1024'd0;
                obj_snapshot_valid <= 1'b1;
            end else begin
                obj_buf_miss <= 1'b1;
            end
        end else begin
            if (ram_obj_write) begin
                if (ram_we[0])
                    obj_staging_dirty_lo[obj_word_index] <= 1'b1;
                if (ram_we[1])
                    obj_staging_dirty_hi[obj_word_index] <= 1'b1;
            end

            if (obj_init_active) begin
                if (!ram_obj_write) begin
                    if (obj_init_index == 10'h3ff) begin
                        obj_init_active <= 1'b0;
                        obj_staging_dirty_lo <= 1024'd0;
                        obj_staging_dirty_hi <= 1024'd0;
                    end else begin
                        obj_init_index <= obj_init_index + 10'd1;
                    end
                end
            end else if (obj_sync_start_pending) begin
                obj_sync_start_pending <= 1'b0;
                obj_sync_state <= OBJ_SYNC_READ;
                obj_sync_index <= 10'd0;
            end else begin
                case (obj_sync_state)
                    OBJ_SYNC_READ: obj_sync_state <= OBJ_SYNC_WAIT;

                    OBJ_SYNC_WAIT: begin
                        obj_sync_data <= obj_active_copy_dout;
                        obj_sync_state <= OBJ_SYNC_WRITE;
                    end

                    OBJ_SYNC_WRITE: begin
                        if (!ram_obj_write) begin
                            if (obj_sync_index == 10'h3ff) begin
                                obj_sync_state <= OBJ_SYNC_IDLE;
                            end else begin
                                obj_sync_index <= obj_sync_index + 10'd1;
                                obj_sync_state <= OBJ_SYNC_READ;
                            end
                        end
                    end

                    default: begin
                    end
                endcase
            end
        end
    end
end

integer i;
always @(posedge clk) begin
    done <= 1'b0;
    irq_clear <= 1'b0;
    ram_we <= 2'b00;
    vram_write <= 1'b0;
    scroll_write <= 1'b0;

    if (rst) begin
        state <= ST_IDLE;
        busy <= 1'b0;
        req_rw <= 1'b0;
        req_addr <= 4'd0;
        req_din <= 16'd0;
        req_we_mask <= 2'b00;
        req_status_bit <= 1'b0;
        ptr <= 13'd0;
        scroll_select <= 8'd0;
        flip_reg <= 8'd0;
        ram_addr <= 13'd0;
        ram_din <= 16'd0;
        dout <= 16'hffff;
        for (i = 0; i < 8; i = i + 1)
            scroll_reg[i] <= 16'd0;
    end else if (ss_reg_restore_write && (ss_reg_addr < 32'd11)) begin
        state <= ST_IDLE;
        busy <= 1'b0;
        if (ss_reg_addr == 32'd0)
            ptr <= ss_data[12:0];
        else if (ss_reg_addr == 32'd1)
            scroll_select <= ss_data[7:0];
        else if (ss_reg_addr < 32'd10)
            scroll_reg[ss_reg_addr[2:0] - 3'd2] <= ss_data[15:0];
        else
            flip_reg <= ss_data[7:0];
    end else if (!ss_hold || busy || (state != ST_IDLE)) begin
        case (state)
            ST_IDLE: begin
                if (start && !busy && !ss_hold) begin
                    busy <= 1'b1;
                    req_rw <= rw;
                    req_addr <= addr;
                    req_din <= din;
                    req_we_mask <= we_mask;
                    req_status_bit <= status_bit;
                    state <= ST_DISPATCH;
                end
            end

            ST_DISPATCH: begin
                if (req_rw) begin
                    if (op == 4'h4) begin
                        ram_addr <= ptr;
                        ptr <= ptr + 13'd1;
                        state <= ST_RD_0;
                    end else begin
                        dout <= (op == 4'hc) ?
                                {15'd0, req_status_bit} : 16'hffff;
                        done <= 1'b1;
                        busy <= 1'b0;
                        state <= ST_IDLE;
                    end
                end else begin
                    case (op)
                        4'h0: ptr <= ptr_merged[12:0];

                        4'h4: begin
                            ram_addr <= ptr;
                            ram_din <= req_din;
                            ram_we <= req_we_mask;
                            ptr <= ptr + 13'd1;
                            vram_write <= |req_we_mask;
                        end

                        4'h8: scroll_select <= select_merged[7:0] & 8'h8f;

                        default: begin
                            if (scroll_index_valid) begin
                                scroll_reg[scroll_select[2:0]] <= scroll_merged;
                                flip_reg[scroll_select[2:0]] <= scroll_select[7];
                                scroll_write <= |req_we_mask;
                            end
                            if (irq_selector)
                                irq_clear <= 1'b1;
                        end
                    endcase

                    dout <= 16'hffff;
                    done <= 1'b1;
                    busy <= 1'b0;
                    state <= ST_IDLE;
                end
            end

            ST_RD_0: state <= ST_RD_1;
            ST_RD_1: state <= ST_RD_OUT;

            default: begin
                dout <= ram_dout;
                done <= 1'b1;
                busy <= 1'b0;
                state <= ST_IDLE;
            end
        endcase
    end
end

endmodule
