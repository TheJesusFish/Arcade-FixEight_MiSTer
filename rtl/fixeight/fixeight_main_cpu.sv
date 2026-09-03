// TP-026 MC68000 wrapper with explicit ROM and board request/ack channels.
module fixeight_main_cpu (
    input  logic        clk,
    input  logic        reset,
    input  logic        halt_n,
    input  logic        cpu_run,

    output logic        rom_req,
    output logic [18:0] rom_word_addr,
    input  logic [15:0] rom_rdata,
    input  logic        rom_ack,

    output logic        board_req,
    output logic [23:0] board_addr,
    output logic        board_rw,
    output logic [1:0]  board_byte_en,
    output logic [15:0] board_wdata,
    input  logic [15:0] board_rdata,
    input  logic        board_ack,

    input  logic        irq4_start,
    input  logic        irq4_clear,
    input  logic        state_irq7,
    input  logic        state_override,
    input  logic        state_reset,
    input  logic [63:0] state_reset_vector,
    input  logic        state_restore_commit,
    input  logic        state_restore_irq4,
    output logic        irq4_pending,
    output logic        cpu_bus_active,
    output logic        cpu_ack,
    output logic        cpu_iack,
    output logic        cpu_lds_n_debug,
    output logic [2:0]  cpu_fc,
    output logic [23:0] cpu_addr_debug,
    output logic [15:0] cpu_din_debug,
    output logic [15:0] cpu_dout_debug,
    output logic [31:0] bus_count,
    output logic [31:0] rom_read_count,
    output logic [31:0] board_access_count,
    output logic [31:0] unmapped_count,
    output logic [23:0] last_program_fetch
);

logic en_phi1;
logic en_phi2;
logic [23:1] cpu_word_addr;
logic [15:0] cpu_dout;
logic [15:0] cpu_din;
logic cpu_as_n;
logic cpu_lds_n;
logic cpu_uds_n;
logic cpu_rw;
logic cpu_fc0;
logic cpu_fc1;
logic cpu_fc2;
logic cpu_dtack_n;
logic cpu_vpa_n;
logic rom_cs;
logic rom_read_cs;
logic mapped_cs;
logic unmapped_cs;
logic wram_cs, p1_cs, p2_cs, p3_cs, system_cs, coin_cs, shared_cs;
logic gp_cs, palette_cs, text_vram_cs, text_line_select_cs;
logic text_line_scroll_cs, text_char_cs, sound_reset_cs, vcount_cs;
logic ack_seen;
logic bus_response;
logic program_space;
logic response_latched;
logic source_ready;
logic special_cs;
logic handler_cs;
logic reset_vector_cs;
logic irq_vector_cs;
logic cpu_core_reset;

assign board_addr = {cpu_word_addr, 1'b0};
assign cpu_addr_debug = board_addr;
assign board_rw = cpu_rw;
assign board_byte_en = {!cpu_uds_n, !cpu_lds_n};
assign board_wdata = cpu_dout;
assign cpu_bus_active = !cpu_as_n && (!cpu_uds_n || !cpu_lds_n);
assign cpu_iack = !cpu_as_n && cpu_fc0 && cpu_fc1 && cpu_fc2;
assign cpu_lds_n_debug = cpu_lds_n;
assign cpu_fc = {cpu_fc2, cpu_fc1, cpu_fc0};
assign program_space = cpu_fc1 && !cpu_fc0;
assign cpu_din_debug = cpu_din;
assign cpu_dout_debug = cpu_dout;
assign handler_cs =
    state_override && cpu_bus_active && (board_addr[23:8] == 16'hff00);
assign reset_vector_cs =
    state_override && cpu_bus_active && (board_addr < 24'h000008);
assign irq_vector_cs =
    state_override && cpu_bus_active &&
    ((board_addr == 24'h00007c) || (board_addr == 24'h00007e));
assign special_cs = handler_cs || reset_vector_cs || irq_vector_cs;
assign cpu_core_reset = reset || state_reset;

function automatic logic [15:0] state_handler_word(
    input logic [3:0] index
);
    begin
        case (index)
            4'h0: state_handler_word = 16'h48e7;
            4'h1: state_handler_word = 16'hfffe;
            4'h2: state_handler_word = 16'h4e6e;
            4'h3: state_handler_word = 16'h2f0e;
            4'h4: state_handler_word = 16'h4df9;
            4'h5: state_handler_word = 16'h00ff;
            4'h6: state_handler_word = 16'h0000;
            4'h7: state_handler_word = 16'h2c8f;
            4'h8: state_handler_word = 16'h2c5f;
            4'h9: state_handler_word = 16'h4e66;
            4'ha: state_handler_word = 16'h4cdf;
            4'hb: state_handler_word = 16'h7fff;
            4'hc: state_handler_word = 16'h4e73;
            default: state_handler_word = 16'h0000;
        endcase
    end
endfunction

function automatic logic [15:0] state_vector_word(
    input logic [1:0] index
);
    begin
        case (index)
            2'd0: state_vector_word = state_reset_vector[63:48];
            2'd1: state_vector_word = state_reset_vector[47:32];
            2'd2: state_vector_word = state_reset_vector[31:16];
            default: state_vector_word = state_reset_vector[15:0];
        endcase
    end
endfunction

fixeight_68k_phases u_phases (
    .clk, .reset(cpu_core_reset), .en_phi1, .en_phi2
);

fixeight_main_decode u_decode (
    .addr(board_addr),
    .bus_active(cpu_bus_active),
    .rw(cpu_rw),
    .rom_cs,
    .rom_read_cs,
    .wram_cs,
    .p1_cs,
    .p2_cs,
    .p3_cs,
    .system_cs,
    .coin_cs,
    .shared_cs,
    .gp_cs,
    .palette_cs,
    .text_vram_cs,
    .text_line_select_cs,
    .text_line_scroll_cs,
    .text_char_cs,
    .sound_reset_cs,
    .vcount_cs,
    .mapped_cs,
    .unmapped_cs
);

assign rom_req = rom_read_cs && !cpu_iack && !special_cs;
assign rom_word_addr = board_addr[19:1];
assign board_req =
    cpu_bus_active && !cpu_iack && !rom_cs && !special_cs;
assign source_ready =
    special_cs ||
    (rom_req && rom_ack) ||
    (board_req && (board_ack || unmapped_cs)) ||
    (cpu_bus_active && rom_cs && !cpu_rw);
assign bus_response = cpu_bus_active && response_latched;
assign cpu_dtack_n = !bus_response;
assign cpu_vpa_n = !cpu_iack;

always_comb begin
    if (handler_cs)
        cpu_din = state_handler_word(board_addr[4:1]);
    else if (reset_vector_cs)
        cpu_din = state_vector_word(board_addr[2:1]);
    else if (irq_vector_cs)
        cpu_din = board_addr[1] ? 16'h0000 : 16'h00ff;
    else if (rom_req)
        cpu_din = rom_rdata;
    else if (board_req && mapped_cs)
        cpu_din = board_rdata;
    else
        cpu_din = 16'hffff;
end

fx68k u_cpu (
    .clk,
    .HALTn(halt_n),
    .extReset(cpu_core_reset),
    .pwrUp(cpu_core_reset),
    .enPhi1(en_phi1 && cpu_run),
    .enPhi2(en_phi2 && cpu_run),
    .eRWn(cpu_rw),
    .ASn(cpu_as_n),
    .LDSn(cpu_lds_n),
    .UDSn(cpu_uds_n),
    .E(),
    .VMAn(),
    .FC0(cpu_fc0),
    .FC1(cpu_fc1),
    .FC2(cpu_fc2),
    .BGn(),
    .oRESETn(),
    .oHALTEDn(),
    .DTACKn(cpu_dtack_n),
    .VPAn(cpu_vpa_n),
    .BERRn(1'b1),
    .BRn(1'b1),
    .BGACKn(1'b1),
    .IPL0n(~state_irq7),
    .IPL1n(~state_irq7),
    .IPL2n(~(state_irq7 || irq4_pending)),
    .iEdb(cpu_din),
    .oEdb(cpu_dout),
    .eab(cpu_word_addr)
);

always_ff @(posedge clk) begin
    if (reset) begin
        irq4_pending <= 1'b0;
        ack_seen <= 1'b0;
        response_latched <= 1'b0;
        cpu_ack <= 1'b0;
        bus_count <= 32'd0;
        rom_read_count <= 32'd0;
        board_access_count <= 32'd0;
        unmapped_count <= 32'd0;
        last_program_fetch <= 24'd0;
    end else begin
        cpu_ack <= 1'b0;
        if (state_restore_commit)
            irq4_pending <= state_restore_irq4;
        else if (irq4_start)
            irq4_pending <= 1'b1;
        if (!state_restore_commit && irq4_clear)
            irq4_pending <= 1'b0;
        if (!cpu_bus_active) begin
            ack_seen <= 1'b0;
            response_latched <= 1'b0;
        end else if (source_ready) begin
            // Never acknowledge combinationally in the same system-clock
            // instant that AS/DS appears. Hold finite DTACK until bus release.
            response_latched <= 1'b1;
        end
        if (cpu_bus_active && bus_response && !ack_seen) begin
            ack_seen <= 1'b1;
            cpu_ack <= 1'b1;
            bus_count <= bus_count + 32'd1;
            if (rom_req) begin
                rom_read_count <= rom_read_count + 32'd1;
                if (program_space)
                    last_program_fetch <= board_addr;
            end
            if (board_req)
                board_access_count <= board_access_count + 32'd1;
            if (unmapped_cs && !special_cs)
                unmapped_count <= unmapped_count + 32'd1;
        end
    end
end

endmodule
