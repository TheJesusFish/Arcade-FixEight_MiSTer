// FixEight-local sync repositioner, derived from JTFrame jtframe_resync.
module fixeight_resync #(
    parameter CNTW  = 10,
    parameter HOFFW = 5,
    parameter VOFFW = 4
) (
    input              clk,
    input              pxl_cen,
    input              hs_in,
    input              vs_in,
    input              LVBL,
    input              LHBL,
    input  [HOFFW-1:0] hoffset,
    input  [VOFFW-1:0] voffset,
    output reg         hs_out,
    output reg         vs_out
);

reg [CNTW-1:0] hs_pos [0:1];
reg [CNTW-1:0] vs_hpos[0:1];
reg [CNTW-1:0] vs_vpos[0:1];
reg [CNTW-1:0] hs_len [0:1];
reg [CNTW-1:0] vs_len [0:1];
reg [CNTW-1:0] hs_cnt;
reg [CNTW-1:0] vs_cnt;
reg [CNTW-1:0] hs_hold;
reg [CNTW-1:0] vs_hold;
reg            last_LHBL;
reg            last_LVBL;
reg            last_hsin;
reg            last_vsin;
reg            field;

wire hb_edge   = LHBL && !last_LHBL;
// FixEight supplies active-low sync pulses.
wire hs_edge   = !hs_in && last_hsin;
wire hs_n_edge = hs_in && !last_hsin;
wire vb_edge   = LVBL && !last_LVBL;
wire vs_edge   = !vs_in && last_vsin;
wire vs_n_edge = vs_in && !last_vsin;

wire [CNTW-1:0] hpos_off =
    {{CNTW-HOFFW{hoffset[HOFFW-1]}}, hoffset};
wire [CNTW-1:0] vpos_off =
    {{CNTW-VOFFW{voffset[VOFFW-1]}}, voffset};
wire [CNTW-1:0] htrip    = hs_pos[field]  + hpos_off;
wire [CNTW-1:0] vs_htrip = vs_hpos[field] + hpos_off;
wire [CNTW-1:0] vs_vtrip = vs_vpos[field] + vpos_off;

always @(posedge clk) if (pxl_cen) begin
    last_LHBL <= LHBL;
    last_LVBL <= LVBL;
    last_hsin <= hs_in;
    last_vsin <= vs_in;

    hs_cnt <= hb_edge ? {CNTW{1'b0}} : hs_cnt + 1'b1;
    if (vb_edge) begin
        vs_cnt <= {CNTW{1'b0}};
        field <= ~field;
    end else if (hb_edge) begin
        vs_cnt <= vs_cnt + 1'b1;
    end

    if (hs_edge) begin
        hs_pos[field] <= hs_cnt;
    end
    if (hs_n_edge) begin
        hs_len[field] <= hs_cnt - hs_pos[field];
    end

    if (hs_cnt == htrip) begin
        hs_out <= 1'b0;
        hs_hold <= hs_len[field] - 1'b1;
    end else begin
        if (|hs_hold) begin
            hs_hold <= hs_hold - 1'b1;
        end
        if (hs_hold == 0) begin
            hs_out <= 1'b1;
        end
    end

    if (vs_edge) begin
        vs_hpos[field] <= hs_cnt;
        vs_vpos[field] <= vs_cnt;
    end
    if (vs_n_edge) begin
        vs_len[field] <= vs_cnt - vs_vpos[field];
    end

    if (hs_cnt == vs_htrip) begin
        if (vs_cnt == vs_vtrip) begin
            vs_hold <= vs_len[field] - 1'b1;
            vs_out <= 1'b0;
        end else begin
            if (|vs_hold) begin
                vs_hold <= vs_hold - 1'b1;
            end
            if (vs_hold == 0) begin
                vs_out <= 1'b1;
            end
        end
    end
end

`ifdef SIMULATION
initial begin
    hs_cnt = {CNTW{1'b0}};
    vs_cnt = {CNTW{1'b0}};
    hs_hold = {CNTW{1'b0}};
    vs_hold = {CNTW{1'b0}};
    hs_out = 1'b1;
    vs_out = 1'b1;
    last_LHBL = 1'b0;
    last_LVBL = 1'b0;
    last_hsin = 1'b1;
    last_vsin = 1'b1;
    field = 1'b0;
    hs_pos[0] = {CNTW{1'b0}};
    hs_pos[1] = {CNTW{1'b0}};
    vs_hpos[0] = {CNTW{1'b0}};
    vs_hpos[1] = {CNTW{1'b0}};
    vs_vpos[0] = {CNTW{1'b0}};
    vs_vpos[1] = {CNTW{1'b0}};
    hs_len[0] = {CNTW{1'b0}};
    hs_len[1] = {CNTW{1'b0}};
    vs_len[0] = {CNTW{1'b0}};
    vs_len[1] = {CNTW{1'b0}};
end
`endif

endmodule
