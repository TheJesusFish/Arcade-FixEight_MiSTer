// Decode the frozen 0x4c0080 index-0 aggregate ABI.
module fixeight_rom_layout (
    input  logic [26:0] byte_addr,
    output logic        valid,
    output logic        sentinel,
    output logic        main_cs,
    output logic        gp_cs,
    output logic        oki_cs,
    output logic        eeprom_cs,
    output logic [22:0] local_addr
);

localparam logic [26:0] MAIN_END     = 27'h0080000;
localparam logic [26:0] GP_END       = 27'h0480000;
localparam logic [26:0] OKI_END      = 27'h04c0000;
localparam logic [26:0] EEPROM_END   = 27'h04c0080;

always_comb begin
    valid = byte_addr < EEPROM_END;
    sentinel = byte_addr == EEPROM_END;
    main_cs = valid && (byte_addr < MAIN_END);
    gp_cs = valid && (byte_addr >= MAIN_END) && (byte_addr < GP_END);
    oki_cs = valid && (byte_addr >= GP_END) && (byte_addr < OKI_END);
    eeprom_cs = valid && (byte_addr >= OKI_END);
    local_addr = 23'd0;
    if (main_cs)
        local_addr = byte_addr[22:0];
    else if (gp_cs)
        local_addr = byte_addr - MAIN_END;
    else if (oki_cs)
        local_addr = byte_addr - GP_END;
    else if (eeprom_cs)
        local_addr = byte_addr - OKI_END;
end

endmodule
