// Exact MC68000 word-bus decode for the original TP-026 board.
module fixeight_main_decode (
    input  logic [23:0] addr,
    input  logic        bus_active,
    input  logic        rw,
    output logic        rom_cs,
    output logic        rom_read_cs,
    output logic        wram_cs,
    output logic        p1_cs,
    output logic        p2_cs,
    output logic        p3_cs,
    output logic        system_cs,
    output logic        coin_cs,
    output logic        shared_cs,
    output logic        gp_cs,
    output logic        palette_cs,
    output logic        text_vram_cs,
    output logic        text_line_select_cs,
    output logic        text_line_scroll_cs,
    output logic        text_char_cs,
    output logic        sound_reset_cs,
    output logic        vcount_cs,
    output logic        mapped_cs,
    output logic        unmapped_cs
);

logic read_cycle;
assign read_cycle = bus_active && rw;

assign rom_cs = bus_active && (addr < 24'h080000);
assign rom_read_cs = rom_cs && rw;
assign wram_cs = bus_active && (addr >= 24'h100000) && (addr < 24'h104000);
assign p1_cs = read_cycle && ((addr & 24'hfffffe) == 24'h200000);
assign p2_cs = read_cycle && ((addr & 24'hfffffe) == 24'h200004);
assign p3_cs = read_cycle && ((addr & 24'hfffffe) == 24'h200008);
assign system_cs = read_cycle && ((addr & 24'hfffffe) == 24'h200010);
// fx68k exposes A23:A1, so an LDS write to byte 20001d appears at word base
// 20001c here. Accept either representation for standalone decode tests.
assign coin_cs =
    bus_active && !rw && ((addr & 24'hfffffe) == 24'h20001c);
assign shared_cs =
    bus_active && (addr >= 24'h280000) && (addr < 24'h290000);
assign gp_cs =
    bus_active && (addr >= 24'h300000) && (addr < 24'h30000e);
assign palette_cs =
    bus_active && (addr >= 24'h400000) && (addr < 24'h401000);
assign text_vram_cs =
    bus_active && (addr >= 24'h500000) && (addr < 24'h502000);
assign text_line_select_cs =
    bus_active && (addr >= 24'h502000) && (addr < 24'h502200);
assign text_line_scroll_cs =
    bus_active && (addr >= 24'h503000) && (addr < 24'h503200);
assign text_char_cs =
    bus_active && (addr >= 24'h600000) && (addr < 24'h610000);
assign sound_reset_cs =
    bus_active && !rw && ((addr & 24'hfffffe) == 24'h700000);
assign vcount_cs =
    read_cycle && ((addr & 24'hfffffe) == 24'h800000);

assign mapped_cs =
    rom_cs || wram_cs || p1_cs || p2_cs || p3_cs || system_cs || coin_cs ||
    shared_cs || gp_cs || palette_cs || text_vram_cs ||
    text_line_select_cs || text_line_scroll_cs || text_char_cs ||
    sound_reset_cs || vcount_cs;
assign unmapped_cs = bus_active && !mapped_cs;

endmodule
