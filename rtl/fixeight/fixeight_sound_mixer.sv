// SPDX-License-Identifier: BSD-3-Clause

module fixeight_sound_mixer (
    input  logic                      clk,
    input  logic                      reset,
    input  logic                      sample,
    input  logic signed [15:0]        ym_left,
    input  logic signed [15:0]        ym_right,
    input  logic signed [13:0]        oki,
    input  logic                      ym_enable,
    input  logic                      oki_enable,
    input  logic [1:0]                fx_level,
    input  logic                      oki_ready,
    output logic signed [15:0]        mono
);

logic signed [19:0] ym_left_ext;
logic signed [19:0] ym_right_ext;
logic signed [19:0] oki_base;
logic signed [19:0] oki_ext;
logic signed [19:0] mono_scaled;
logic signed [19:0] previous_scaled;
logic signed [20:0] filter_sum;
logic signed [20:0] mono_filtered;

assign ym_left_ext =
    ym_enable ? {{4{ym_left[15]}}, ym_left} : 20'sd0;
assign ym_right_ext =
    ym_enable ? {{4{ym_right[15]}}, ym_right} : 20'sd0;
assign oki_base = {{6{oki[13]}}, oki};
// MAME routes both YM channels and OKI at 0.5. JT6295's numeric stream is
// eight times smaller, so the pre-mix shift restores the comparable scale.
// JTFrame decodes OSD bits 7:6 with XOR 2: 0=Very Low, 1=Low,
// 2=High (the unchanged default/MAME reference), 3=Very High.
always_comb begin
    case (fx_level)
        2'd0: oki_ext = oki_base <<< 2;
        2'd1: oki_ext = oki_base <<< 3;
        2'd3: oki_ext = oki_base <<< 5;
        default: oki_ext = oki_base <<< 4;
    endcase
    if (!oki_enable || !oki_ready) oki_ext = 20'sd0;
end
assign mono_scaled = (ym_left_ext + ym_right_ext + oki_ext) >>> 1;
assign filter_sum =
    {previous_scaled[19], previous_scaled} +
    {mono_scaled[19], mono_scaled};
assign mono_filtered = filter_sum >>> 1;

function automatic signed [15:0] saturate16(input signed [20:0] value);
    begin
        if (value[20:15] == {6{value[15]}})
            saturate16 = value[15:0];
        else
            saturate16 = {value[20], {15{~value[20]}}};
    end
endfunction

always_ff @(posedge clk) begin
    if (reset) begin
        previous_scaled <= 20'sd0;
        mono <= 16'sd0;
    end else if (sample) begin
        previous_scaled <= mono_scaled;
        mono <= saturate16(mono_filtered);
    end
end

endmodule
