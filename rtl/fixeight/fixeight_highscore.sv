// FixEight high-score RAM persistence. The descriptor/ranges are from the
// official FixEight entries in MAME 0.288 plugins/hiscore/hiscore.dat.
// The HPS transport/idle-bus ownership follows the local Toaplan donor cores;
// no EEPROM, ROM, save-state format, or shared framework is modified.
//
// Index 4: exact 24-byte descriptor. Index 2: 272-byte F8HS v1 container:
//   0..134 payload, 135..255 zero, 256..259 "F8HS", 260 version,
//   261 set ID, 262 zero, 263 length, 264..267 EEPROM seed CRC32,
//   268..271 payload CRC32 (both CRCs big-endian).
// A two-bank synchronous RAM preserves the last complete valid file while
// incoming files and outgoing snapshots are staged in the other bank.
module fixeight_highscore #(
    parameter integer HOLD_TIMEOUT_BITS = 20,
    parameter integer UPLOAD_TIMEOUT_BITS = 30
) (
    input  logic        clk,
    input  logic        reset,
    input  logic        cpu_reset,
    input  logic        identity_valid,
    input  logic [3:0]  set_id,
    input  logic [31:0] seed_crc32,
    input  logic        config_download,
    input  logic        nvram_download,
    input  logic        nvram_upload,
    input  logic        ioctl_wr,
    input  logic        ioctl_rd,
    input  logic [26:0] ioctl_addr,
    input  logic [7:0]  ioctl_data,
    output logic [7:0]  nvram_q,
    output logic        nvram_wait,
    input  logic        ss_active,
    input  logic        ss_restore_commit,
    input  logic        ss_ready_known,
    input  logic        ss_ready_value,
    input  logic [12:0] normal_addr,
    input  logic [1:0]  normal_we,
    input  logic [15:0] normal_data,
    output logic        hold_request,
    input  logic        hold_ack,
    output logic [12:0] ram_addr,
    output logic [1:0]  ram_we,
    output logic [15:0] ram_data,
    input  logic [15:0] ram_q,
    output logic        dirty,
    output logic        ready,
    output logic        active,
    output logic        config_valid
);

localparam [7:0] PAYLOAD_SIZE = 8'd135;
localparam [8:0] FILE_SIZE = 9'd272;
typedef enum logic [2:0] {IDLE, HOLD, ADDRESS, READ_WAIT, TRANSFER} state_t;
typedef enum logic [1:0] {CHECK_INIT, RESTORE, SNAPSHOT} operation_t;
state_t state;
operation_t operation;
logic [7:0] cursor;
logic init_good, table_ready;
logic [4:0] guard_seen;
logic saved_bank, saved_valid, load_pending, boot_restore_allowed;
logic snapshot_ready, upload_rejected, late_dirty;
logic config_d, download_d, upload_d;
logic [4:0] config_count;
logic config_bad;
logic [8:0] download_count, upload_count;
logic download_bad, upload_bad;
logic [31:0] download_crc, snapshot_crc;
logic [7:0] duplicate_byte;
logic [HOLD_TIMEOUT_BITS-1:0] hold_timer;
logic [UPLOAD_TIMEOUT_BITS-1:0] upload_timer;
logic [3:0] saved_set;
logic [31:0] saved_seed;
logic [3:0] download_set;
logic [31:0] download_seed;
wire identity_ok = identity_valid && set_id < 4'd14;
wire cached_identity_ok = saved_set == set_id && saved_seed == seed_crc32;
wire available = identity_ok && config_valid;
wire [13:0] even_addr = {normal_addr, 1'b0};
wire [13:0] odd_addr = {normal_addr, 1'b1};

function automatic [31:0] crc_byte(input [31:0] crc, input [7:0] value);
    reg [31:0] c;
    integer bitno;
    begin
        c = crc ^ {24'd0, value};
        for (bitno = 0; bitno < 8; bitno = bitno + 1)
            c = c[0] ? (c >> 1) ^ 32'hedb88320 : c >> 1;
        crc_byte = c;
    end
endfunction

function automatic [13:0] payload_addr(input [7:0] index);
    if (index < 8'd4) payload_addr = 14'h0408 + {6'd0, index};
    else if (index < 8'd134) payload_addr = 14'h0438 + {6'd0, index};
    else payload_addr = 14'h040a;
endfunction

function automatic [13:0] guard_addr(input [2:0] index);
    case (index)
        0: guard_addr = 14'h0408;
        1: guard_addr = 14'h040b;
        2: guard_addr = 14'h043c;
        3: guard_addr = 14'h04bd;
        default: guard_addr = 14'h040a;
    endcase
endfunction

function automatic [7:0] guard_value(input [2:0] index);
    case (index)
        3: guard_value = 8'h04;
        4: guard_value = 8'h53;
        default: guard_value = 8'h00;
    endcase
endfunction

function automatic score_address(input [13:0] addr);
    score_address = (addr >= 14'h0408 && addr <= 14'h040b) ||
                    (addr >= 14'h043c && addr <= 14'h04bd);
endfunction

function automatic [7:0] descriptor_byte(input [4:0] addr);
    case (addr)
        1, 9, 17: descriptor_byte = 8'h10;
        2, 10, 18, 5, 15: descriptor_byte = 8'h04;
        3: descriptor_byte = 8'h08;
        11: descriptor_byte = 8'h3c;
        13: descriptor_byte = 8'h82;
        19: descriptor_byte = 8'h0a;
        21: descriptor_byte = 8'h01;
        22, 23: descriptor_byte = 8'h53;
        default: descriptor_byte = 8'h00;
    endcase
endfunction

function automatic [7:0] trailer_byte(input [3:0] addr, input [31:0] crc);
    case (addr)
        0: trailer_byte = "F";
        1: trailer_byte = "8";
        2: trailer_byte = "H";
        3: trailer_byte = "S";
        4: trailer_byte = 8'd1;
        5: trailer_byte = {4'd0, set_id};
        6: trailer_byte = 8'd0;
        7: trailer_byte = PAYLOAD_SIZE;
        8: trailer_byte = seed_crc32[31:24];
        9: trailer_byte = seed_crc32[23:16];
        10: trailer_byte = seed_crc32[15:8];
        11: trailer_byte = seed_crc32[7:0];
        12: trailer_byte = ~crc[31:24];
        13: trailer_byte = ~crc[23:16];
        14: trailer_byte = ~crc[15:8];
        default: trailer_byte = ~crc[7:0];
    endcase
endfunction

wire [13:0] byte_addr = operation == CHECK_INIT ?
                         guard_addr(cursor[2:0]) : payload_addr(cursor);
wire [7:0] ram_byte = byte_addr[0] ? ram_q[7:0] : ram_q[15:8];
wire score_write = (normal_we[1] && score_address(even_addr)) ||
                   (normal_we[0] && score_address(odd_addr));
wire transfer = state == TRANSFER && hold_ack && !cpu_reset && !ss_active;
wire capture_byte = transfer && operation == SNAPSHOT;
wire download_byte = nvram_download && ioctl_wr && !nvram_wait;
wire [8:0] expected_download = download_d ? download_count : 9'd0;
wire [4:0] expected_config = config_d ? config_count : 5'd0;
wire [31:0] incoming_crc = download_d ? download_crc : 32'hffffffff;
wire buffer_download_we = download_byte && ioctl_addr < 27'd135 &&
                          ioctl_addr == {18'd0, expected_download};
wire buffer_we = buffer_download_we || capture_byte;
wire [8:0] buffer_addr = buffer_download_we ? {~saved_bank, ioctl_addr[7:0]} :
                         {operation == RESTORE ? saved_bank : ~saved_bank, cursor};
wire [7:0] buffer_data = buffer_download_we ? ioctl_data : ram_byte;
wire [7:0] buffer_q, upload_q;

jtframe_dual_ram #(.DW(8), .AW(9)) u_score_buffer (
    .clk0(clk), .addr0(buffer_addr), .data0(buffer_data), .we0(buffer_we), .q0(buffer_q),
    .clk1(clk), .addr1({~saved_bank, ioctl_addr[7:0]}),
    .data1(8'd0), .we1(1'b0), .q1(upload_q)
);

assign hold_request = state != IDLE && !cpu_reset && !ss_active;
assign ram_addr = byte_addr[13:1];
assign ram_we = transfer && operation == RESTORE ?
                (byte_addr[0] ? 2'b01 : 2'b10) : 2'b00;
assign ram_data = {buffer_q, buffer_q};
assign ready = available && table_ready && !load_pending && !cpu_reset;
assign active = state != IDLE || config_download || nvram_download || nvram_upload;
assign nvram_wait = (nvram_download && (ss_active || state != IDLE)) ||
                    (nvram_upload && !snapshot_ready && !upload_rejected);
always_comb begin
    nvram_q = 8'd0;
    if (snapshot_ready && !upload_rejected) begin
        if (ioctl_addr < 27'd135) nvram_q = upload_q;
        else if (ioctl_addr >= 27'd256 && ioctl_addr < 27'd272)
            nvram_q = trailer_byte(ioctl_addr[3:0], snapshot_crc);
    end
end

integer g;
always_ff @(posedge clk) begin
    if (reset) begin
        state <= IDLE;
        operation <= CHECK_INIT;
        cursor <= 0;
        init_good <= 0;
        table_ready <= 0;
        guard_seen <= 0;
        saved_bank <= 0;
        saved_valid <= 0;
        saved_set <= 0;
        saved_seed <= 0;
        download_set <= 0;
        download_seed <= 0;
        load_pending <= 0;
        boot_restore_allowed <= 1;
        dirty <= 0;
        snapshot_ready <= 0;
        upload_rejected <= 0;
        late_dirty <= 0;
        config_d <= 0;
        download_d <= 0;
        upload_d <= 0;
        config_valid <= 0;
        config_count <= 0;
        config_bad <= 0;
        download_count <= 0;
        upload_count <= 0;
        download_bad <= 0;
        upload_bad <= 0;
        download_crc <= 32'hffffffff;
        snapshot_crc <= 32'hffffffff;
        duplicate_byte <= 0;
        hold_timer <= 0;
        upload_timer <= 0;
    end else begin
        config_d <= config_download;
        download_d <= nvram_download;
        upload_d <= nvram_upload;

        if (cpu_reset || !identity_ok || (saved_valid && !cached_identity_ok)) begin
            state <= IDLE;
            table_ready <= 0;
            guard_seen <= 0;
            dirty <= 0;
            load_pending <= saved_valid && cached_identity_ok;
            boot_restore_allowed <= 1;
            if (!identity_ok || !cached_identity_ok) begin
                saved_valid <= 0;
                load_pending <= 0;
            end
            if (nvram_upload) begin
                upload_rejected <= 1;
                snapshot_ready <= 0;
            end
        end else begin
            // Observe only acknowledged CPU byte writes, never state or HS writes.
            for (g = 0; g < 5; g = g + 1) begin
                if (normal_we[1] && even_addr == guard_addr(g[2:0]))
                    guard_seen[g] <= normal_data[15:8] == guard_value(g[2:0]);
                if (normal_we[0] && odd_addr == guard_addr(g[2:0]))
                    guard_seen[g] <= normal_data[7:0] == guard_value(g[2:0]);
            end
            if (table_ready && score_write) begin
                dirty <= 1;
                if (snapshot_ready) late_dirty <= 1;
            end

            if (ss_active) begin
                // Normal integration prevents overlap. Fail closed if a client
                // nevertheless asserts state ownership during an NVRAM handoff.
                state <= IDLE;
                if (state != IDLE && operation == SNAPSHOT) upload_rejected <= 1;
            end else case (state)
                IDLE: begin
                    hold_timer <= 0;
                    cursor <= 0;
                    if (available && !nvram_download && !config_download) begin
                        if (!table_ready && (&guard_seen)) begin
                            operation <= CHECK_INIT;
                            init_good <= 1;
                            state <= HOLD;
                        end else if (table_ready && load_pending && saved_valid) begin
                            operation <= RESTORE;
                            state <= HOLD;
                        end else if (nvram_upload && upload_d && table_ready &&
                                     !snapshot_ready && !upload_rejected) begin
                            operation <= SNAPSHOT;
                            snapshot_crc <= 32'hffffffff;
                            state <= HOLD;
                        end
                    end
                end
                HOLD: begin
                    if (hold_ack) state <= ADDRESS;
                    else if (&hold_timer) begin
                        state <= IDLE;
                        if (operation == SNAPSHOT) upload_rejected <= 1;
                        // Do not retry endlessly against a permanently busy CPU.
                        guard_seen <= 0;
                        load_pending <= 0;
                    end else hold_timer <= hold_timer + 1'b1;
                end
                ADDRESS: state <= READ_WAIT;
                READ_WAIT: state <= TRANSFER;
                TRANSFER: begin
                    if (!hold_ack) begin
                        state <= IDLE;
                        if (operation == SNAPSHOT) upload_rejected <= 1;
                    end else if (operation == CHECK_INIT) begin
                        init_good <= init_good && ram_byte == guard_value(cursor[2:0]);
                        // Keep matching guards when a post-state recheck sees
                        // a partly initialized table. Later CPU writes need
                        // only supply the missing guards, not rewrite all five.
                        guard_seen[cursor[2:0]] <= ram_byte == guard_value(cursor[2:0]);
                        if (cursor == 8'd4) begin
                            if (init_good && ram_byte == guard_value(3'd4)) begin
                                table_ready <= 1;
                                dirty <= 1;
                            end
                            state <= IDLE;
                        end else begin
                            cursor <= cursor + 1'b1;
                            state <= ADDRESS;
                        end
                    end else begin
                        if (operation == SNAPSHOT)
                            snapshot_crc <= crc_byte(snapshot_crc, ram_byte);
                        if (cursor == PAYLOAD_SIZE - 1'b1) begin
                            state <= IDLE;
                            if (operation == RESTORE) begin
                                load_pending <= 0;
                                boot_restore_allowed <= 0;
                                dirty <= 0;
                            end else begin
                                snapshot_ready <= 1;
                                late_dirty <= 0;
                            end
                        end else begin
                            cursor <= cursor + 1'b1;
                            state <= ADDRESS;
                        end
                    end
                end
                default: state <= IDLE;
            endcase
        end

        // A descriptor is accepted only in full, in order, and byte-exactly.
        if (config_download && !config_d) begin
            config_valid <= 0;
            config_count <= 0;
            config_bad <= 0;
        end
        if (config_download && ioctl_wr) begin
            if (ioctl_addr != {22'd0, expected_config} || expected_config >= 5'd24 ||
                ioctl_data != descriptor_byte(expected_config)) config_bad <= 1;
            if (expected_config < 5'd24) config_count <= expected_config + 1'b1;
        end
        if (!config_download && config_d)
            config_valid <= !config_bad && config_count == 5'd24;

        if (nvram_download && !download_d) begin
            download_count <= 0;
            download_bad <= 0;
            download_crc <= 32'hffffffff;
            download_set <= set_id;
            download_seed <= seed_crc32;
        end
        if (download_byte) begin
            if (ioctl_addr != {18'd0, expected_download} || expected_download >= FILE_SIZE)
                download_bad <= 1;
            if (expected_download < FILE_SIZE) download_count <= expected_download + 1'b1;
            if (ioctl_addr < 27'd135) begin
                download_crc <= crc_byte(incoming_crc, ioctl_data);
                if (ioctl_addr == 27'd2) duplicate_byte <= ioctl_data;
                if (ioctl_addr == 27'd134 && ioctl_data != duplicate_byte) download_bad <= 1;
            end else if (ioctl_addr < 27'd256) begin
                if (ioctl_data != 0) download_bad <= 1;
            end else if (ioctl_addr < 27'd272 &&
                         ioctl_data != trailer_byte(ioctl_addr[3:0], download_crc))
                download_bad <= 1;
        end
        if (!nvram_download && download_d && download_count == FILE_SIZE &&
            !download_bad && identity_ok && download_set == set_id && download_seed == seed_crc32) begin
            saved_bank <= ~saved_bank;
            saved_valid <= 1;
            saved_set <= set_id;
            saved_seed <= seed_crc32;
            load_pending <= boot_restore_allowed || cpu_reset;
        end

        if (nvram_upload && !upload_d) begin
            snapshot_ready <= 0;
            upload_rejected <= !ready;
            late_dirty <= 0;
            upload_count <= 0;
            upload_bad <= 0;
            upload_timer <= 0;
        end else if (nvram_upload && !snapshot_ready && !upload_rejected) begin
            if (&upload_timer) begin
                upload_rejected <= 1;
                state <= IDLE;
            end else upload_timer <= upload_timer + 1'b1;
        end
        if (nvram_upload && ioctl_rd && !nvram_wait) begin
            // hps_io captures ioctl_din and increments the address together;
            // its registered ioctl_rd reports the byte just consumed, so the
            // visible address here is one past that byte (unlike downloads).
            if (ioctl_addr != {18'd0, upload_count} + 27'd1 || upload_count >= FILE_SIZE)
                upload_bad <= 1;
            if (upload_count < FILE_SIZE) upload_count <= upload_count + 1'b1;
        end
        if (!nvram_upload && upload_d) begin
            if (snapshot_ready && !upload_rejected && !upload_bad && upload_count == FILE_SIZE) begin
                saved_bank <= ~saved_bank;
                saved_valid <= 1;
                saved_set <= set_id;
                saved_seed <= seed_crc32;
                if (!late_dirty && !score_write && !ss_restore_commit) dirty <= 0;
            end
            snapshot_ready <= 0;
            upload_rejected <= 0;
            if (state != IDLE && operation == SNAPSHOT) state <= IDLE;
        end
        if (ss_restore_commit) begin
            // Disk scores must never overwrite a state the user just restored.
            load_pending <= 0;
            boot_restore_allowed <= 0;
            // New schema-1 states carry readiness in previously unused scalar
            // bits. Factory score/character guard bytes can legitimately change
            // in gameplay, so they cannot validate an initialized table.
            // Legacy states without the marker retain the conservative recheck.
            table_ready <= ss_ready_known && ss_ready_value;
            guard_seen <= (ss_ready_known && ss_ready_value) ? 5'd0 : 5'b11111;
            dirty <= 1;
            if (nvram_upload) late_dirty <= 1;
        end
    end
end
endmodule
