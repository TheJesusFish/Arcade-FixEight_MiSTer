// TP-026 index-0 aggregate to JTFrame SDRAM and 93C46 seed bridge.
module fixeight_rom_loader #(
    parameter integer AW = 22
) (
    input  logic              clk,
    input  logic              reset,

    input  logic              ioctl_rom,
    input  logic [26:0]       ioctl_addr,
    input  logic [7:0]        ioctl_dout,
    input  logic              ioctl_wr,

    output logic              dwnld_busy,
    output logic              image_valid,
    output logic              image_identity_valid,
    output logic [3:0]        image_set_id,
    output logic [31:0]       image_eeprom_crc32,
    output logic [26:0]       image_aggregate_length,
    output logic [255:0]      image_aggregate_sha256,

    output logic [1:0]        prog_ba,
    output logic [AW-1:0]     prog_addr,
    output logic [15:0]       prog_data,
    output logic [1:0]        prog_mask,
    output logic              prog_rd,
    output logic              prog_we,
    input  logic              prog_rdy,
    input  logic              prog_ack,

    output logic              eeprom_seed_we,
    output logic [5:0]        eeprom_seed_addr,
    output logic [15:0]       eeprom_seed_data,

    output logic              accepted,
    output logic              range_error,
    output logic              overflow_error,
    output logic              sequence_error,
    output logic              length_error,
    output logic [26:0]       last_addr,
    output logic [26:0]       byte_count
);

localparam logic [26:0] PROGRAM_END  = 27'h0080000;
localparam logic [26:0] GRAPHICS_END = 27'h0480000;
localparam logic [26:0] OKI_END      = 27'h04c0000;
localparam logic [26:0] IMAGE_END    = 27'h04c0080;

logic ioctl_rom_q;
logic finalize_pending;
logic [6:0] eeprom_word_count;
logic [7:0] eeprom_high_byte;
logic [31:0] eeprom_crc_work;

wire download_start = ioctl_rom && !ioctl_rom_q;
wire download_end = !ioctl_rom && ioctl_rom_q;
wire in_range = ioctl_addr < IMAGE_END;
wire terminal_sentinel = ioctl_addr == IMAGE_END;
wire in_program = ioctl_addr < PROGRAM_END;
wire in_graphics =
    (ioctl_addr >= PROGRAM_END) && (ioctl_addr < GRAPHICS_END);
wire in_oki =
    (ioctl_addr >= GRAPHICS_END) && (ioctl_addr < OKI_END);
wire in_eeprom =
    (ioctl_addr >= OKI_END) && (ioctl_addr < IMAGE_END);
wire sequence_ok = ioctl_addr == byte_count;
wire can_accept = !prog_we || prog_ack;
wire [31:0] completed_eeprom_crc = ~eeprom_crc_work;

function automatic logic [31:0] crc32_byte(
    input logic [31:0] crc_in,
    input logic [7:0] data
);
    logic [31:0] crc;
    integer bit_index;
    begin
        crc = crc_in ^ {24'd0, data};
        for (bit_index = 0; bit_index < 8; bit_index = bit_index + 1)
            crc = crc[0] ? ((crc >> 1) ^ 32'hedb88320) : (crc >> 1);
        crc32_byte = crc;
    end
endfunction

function automatic logic official_eeprom_crc(input logic [31:0] crc);
    begin
        case (crc)
            32'h02e925d0, 32'h2bf17652, 32'he9c21987,
            32'h95dec584, 32'h57edaa51, 32'h21e22038,
            32'he3d14fed, 32'hcac91c6f, 32'h08fa73ba,
            32'hc0da4a05, 32'h74e6afb9, 32'hb6d5c06c,
            32'h5dfefc3b, 32'h9fcd93ee:
                official_eeprom_crc = 1'b1;
            default: official_eeprom_crc = 1'b0;
        endcase
    end
endfunction

function automatic logic [3:0] set_id_for_crc(input logic [31:0] crc);
    begin
        case (crc)
            32'h02e925d0: set_id_for_crc = 4'd0;
            32'hcac91c6f: set_id_for_crc = 4'd1;
            32'h08fa73ba: set_id_for_crc = 4'd2;
            32'h95dec584: set_id_for_crc = 4'd3;
            32'h57edaa51: set_id_for_crc = 4'd4;
            32'h74e6afb9: set_id_for_crc = 4'd5;
            32'hb6d5c06c: set_id_for_crc = 4'd6;
            32'h2bf17652: set_id_for_crc = 4'd7;
            32'he9c21987: set_id_for_crc = 4'd8;
            32'hc0da4a05: set_id_for_crc = 4'd9;
            32'h5dfefc3b: set_id_for_crc = 4'd10;
            32'h9fcd93ee: set_id_for_crc = 4'd11;
            32'h21e22038: set_id_for_crc = 4'd12;
            32'he3d14fed: set_id_for_crc = 4'd13;
            default: set_id_for_crc = 4'd15;
        endcase
    end
endfunction

function automatic logic [255:0] aggregate_sha_for_crc(
    input logic [31:0] crc
);
    begin
        case (crc)
            32'h02e925d0: aggregate_sha_for_crc =
                256'hd59addc1383c0687ba4199e393dd8009002147645ec6dc686eeb69687fcd3480;
            32'h2bf17652: aggregate_sha_for_crc =
                256'h4f7b4801255dfad6a7c40ab3ef000b6b9b344a2f485c77772143a0d5222b455c;
            32'he9c21987: aggregate_sha_for_crc =
                256'h365a8f200161c657346e0ca85736367dd7922e627d3895253958cb8624453563;
            32'h95dec584: aggregate_sha_for_crc =
                256'hd47b860c437ce251583de0ada5b2d2bff80901db1dbbc9243557e757e71de9c2;
            32'h57edaa51: aggregate_sha_for_crc =
                256'h61ccc91285e0667336210fb85a67e261e5c33eed551d1fdeb81ed5637a4951fc;
            32'h21e22038: aggregate_sha_for_crc =
                256'h6fa129e73bf271e09a2617adeb997eaa38a1dbb75e6545fe768459dec673300d;
            32'he3d14fed: aggregate_sha_for_crc =
                256'h2870ae20fc3ed29410c83baf0f5b467a9bac378fea9e4ab77af0b360567488ad;
            32'hcac91c6f: aggregate_sha_for_crc =
                256'hef17c86d558c1d5f57154fa4a41f970b9624e0c065163044c54b8959ebac2574;
            32'h08fa73ba: aggregate_sha_for_crc =
                256'heccf8979d2df7fb9dbaf7b1007478cbeeebcc666d117b998d1dae69ca7ab1d06;
            32'hc0da4a05: aggregate_sha_for_crc =
                256'h6463cf100e97d6bc7ccc26bb09e0725783463e9f4ef02e60f85e073640c121c3;
            32'h74e6afb9: aggregate_sha_for_crc =
                256'hca31ff7ea3c25708a45e326e2adb043b2c128aa2d7d9601901d30914923488cb;
            32'hb6d5c06c: aggregate_sha_for_crc =
                256'h1e120cb319e2fa0dceae21257f04d02ae27e7dfce41f585393ce103523048d3b;
            32'h5dfefc3b: aggregate_sha_for_crc =
                256'h8d7467dd63fb9d877a419f1b67764e0a2065f4827d1f610406a1744ae8f0189f;
            32'h9fcd93ee: aggregate_sha_for_crc =
                256'hd27db72b9e4e9a5706fbb63b04ea0140989dd7c22d6fa44678590cf4cf4f0cc0;
            default: aggregate_sha_for_crc = 256'd0;
        endcase
    end
endfunction

logic [26:0] local_byte_addr;
logic [21:0] local_word_addr;
logic [21:0] graphics_word_addr;
logic [1:0] target_bank;
logic write_low_lane;

always_comb begin
    if (in_program) begin
        local_byte_addr = ioctl_addr;
        target_bank = 2'd0;
    end else if (in_graphics) begin
        local_byte_addr = ioctl_addr - PROGRAM_END;
        target_bank = 2'd1;
    end else begin
        local_byte_addr = ioctl_addr - GRAPHICS_END;
        target_bank = 2'd2;
    end
    local_word_addr = local_byte_addr[22:1];

    // Program and graphics bytes are already in final big-endian word order
    // in the aggregate, so byte 0 occupies the upper word lane. JT6295's
    // byte reader selects address 0 from the lower SDRAM lane, so OKI uses
    // the opposite lane mapping.
    write_low_lane = in_oki ? ~local_byte_addr[0] :
                              local_byte_addr[0];
end

fixeight_gfx_repack u_gfx_repack (
    .logical_addr(local_word_addr),
    .physical_addr(graphics_word_addr)
);

assign prog_rd = 1'b0;
// Keep reset/backpressure asserted through the edge that observes the
// falling download request.  Without the registered request term there is
// a half-cycle low gap before finalize_pending is set.
assign dwnld_busy =
    ioctl_rom || ioctl_rom_q || prog_we || finalize_pending;

always_ff @(posedge clk) begin
    ioctl_rom_q <= ioctl_rom;
    accepted <= 1'b0;
    eeprom_seed_we <= 1'b0;

    if (reset) begin
        ioctl_rom_q <= 1'b0;
        finalize_pending <= 1'b0;
        image_valid <= 1'b0;
        image_identity_valid <= 1'b0;
        image_set_id <= 4'd15;
        image_eeprom_crc32 <= 32'd0;
        image_aggregate_length <= IMAGE_END;
        image_aggregate_sha256 <= 256'd0;
        prog_ba <= 2'd0;
        prog_addr <= {AW{1'b0}};
        prog_data <= 16'd0;
        prog_mask <= 2'b11;
        prog_we <= 1'b0;
        eeprom_seed_addr <= 6'd0;
        eeprom_seed_data <= 16'd0;
        eeprom_word_count <= 7'd0;
        eeprom_high_byte <= 8'd0;
        eeprom_crc_work <= 32'hffffffff;
        range_error <= 1'b0;
        overflow_error <= 1'b0;
        sequence_error <= 1'b0;
        length_error <= 1'b0;
        last_addr <= 27'd0;
        byte_count <= 27'd0;
    end else begin
        if (download_start) begin
            finalize_pending <= 1'b0;
            image_valid <= 1'b0;
            image_identity_valid <= 1'b0;
            image_set_id <= 4'd15;
            image_eeprom_crc32 <= 32'd0;
            image_aggregate_sha256 <= 256'd0;
            eeprom_word_count <= 7'd0;
            eeprom_high_byte <= 8'd0;
            eeprom_crc_work <= 32'hffffffff;
            range_error <= 1'b0;
            overflow_error <= 1'b0;
            sequence_error <= 1'b0;
            length_error <= 1'b0;
            last_addr <= 27'd0;
            byte_count <= 27'd0;
        end

        if (prog_we && prog_ack)
            prog_we <= 1'b0;

        if (ioctl_wr && ioctl_rom) begin
            last_addr <= ioctl_addr;
            if (!in_range) begin
                // Fast DDR replay may emit exactly one strobe at the image
                // length as a terminal sentinel.
                if (!terminal_sentinel)
                    range_error <= 1'b1;
                else if (byte_count != IMAGE_END)
                    length_error <= 1'b1;
            end else if (!sequence_ok) begin
                sequence_error <= 1'b1;
            end else if (!can_accept) begin
                overflow_error <= 1'b1;
            end else begin
                accepted <= 1'b1;
                byte_count <= byte_count + 27'd1;
                if (in_eeprom) begin
                    eeprom_crc_work <= crc32_byte(
                        eeprom_crc_work, ioctl_dout
                    );
                    if (!ioctl_addr[0]) begin
                        eeprom_high_byte <= ioctl_dout;
                    end else begin
                        eeprom_seed_addr <= ioctl_addr[6:1];
                        eeprom_seed_data <= {
                            eeprom_high_byte, ioctl_dout
                        };
                        eeprom_seed_we <= 1'b1;
                        eeprom_word_count <= eeprom_word_count + 7'd1;
                    end
                end else begin
                    prog_ba <= target_bank;
                    prog_addr <= in_graphics ?
                        graphics_word_addr[AW-1:0] :
                        local_word_addr[AW-1:0];
                    prog_data <= {ioctl_dout, ioctl_dout};
                    prog_mask <= write_low_lane ? 2'b10 : 2'b01;
                    prog_we <= 1'b1;
                end
            end
        end

        if (download_end) begin
            if (
                byte_count == IMAGE_END &&
                eeprom_word_count == 7'd64 &&
                !range_error && !overflow_error &&
                !sequence_error && !length_error &&
                official_eeprom_crc(completed_eeprom_crc)
            ) begin
                finalize_pending <= 1'b1;
                image_set_id <=
                    set_id_for_crc(completed_eeprom_crc);
                image_eeprom_crc32 <= completed_eeprom_crc;
                image_aggregate_sha256 <=
                    aggregate_sha_for_crc(completed_eeprom_crc);
            end else begin
                length_error <= 1'b1;
                image_valid <= 1'b0;
            end
        end

        if (finalize_pending && !prog_we) begin
            finalize_pending <= 1'b0;
            image_valid <= !range_error && !overflow_error &&
                           !sequence_error && !length_error;
            image_identity_valid <=
                official_eeprom_crc(image_eeprom_crc32);
        end
    end
end

// `prog_rdy` is level readiness, not acceptance. The request remains stable
// until `prog_ack`, and the framework download bridge supplies backpressure.

endmodule
