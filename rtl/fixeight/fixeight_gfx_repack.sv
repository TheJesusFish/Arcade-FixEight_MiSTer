// Pair corresponding TP-026 lower/upper bitplane words in physical SDRAM.
// Logical bit 20 selects one of the two 2 MiB graphics ROM halves.
module fixeight_gfx_repack (
    input  logic [21:0] logical_addr,
    output logic [21:0] physical_addr
);

assign physical_addr = {
    1'b0,
    logical_addr[19:0],
    logical_addr[20]
};

endmodule
