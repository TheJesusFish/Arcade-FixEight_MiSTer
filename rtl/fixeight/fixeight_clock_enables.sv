// Exact TP-026 enables from the core's authoritative 94.5 MHz system clock.
module fixeight_clock_enables (
    input  logic clk,
    input  logic reset,
    output logic ce_vdp_27m,
    output logic ce_cpu_16m,
    output logic ce_pixel_6m75,
    output logic ce_ikaopm_3m375,
    output logic ce_oki_1m
);

logic [2:0] phase27;
logic [7:0] phase16;
logic [3:0] div14;
logic [4:0] div28;
logic [7:0] phase1;
logic [3:0] phase27_sum;
logic [8:0] phase16_sum;
logic [8:0] phase1_sum;

always_comb begin
    phase27_sum = {1'b0, phase27} + 4'd2;
    phase16_sum = {1'b0, phase16} + 9'd32;
    phase1_sum = {1'b0, phase1} + 9'd2;
end

always_ff @(posedge clk) begin
    ce_vdp_27m      <= 1'b0;
    ce_cpu_16m      <= 1'b0;
    ce_pixel_6m75   <= 1'b0;
    ce_ikaopm_3m375 <= 1'b0;
    ce_oki_1m       <= 1'b0;

    if (reset) begin
        phase27 <= 3'd0;
        phase16 <= 8'd0;
        div14 <= 4'd0;
        div28 <= 5'd0;
        phase1 <= 8'd0;
    end else begin
        if (phase27_sum >= 4'd7) begin
            phase27 <= phase27_sum[2:0] - 3'd7;
            ce_vdp_27m <= 1'b1;
        end else begin
            phase27 <= phase27_sum[2:0];
        end

        if (phase16_sum >= 9'd189) begin
            phase16 <= phase16_sum[7:0] - 8'd189;
            ce_cpu_16m <= 1'b1;
        end else begin
            phase16 <= phase16_sum[7:0];
        end

        if (div14 == 4'd13) begin
            div14 <= 4'd0;
            ce_pixel_6m75 <= 1'b1;
        end else begin
            div14 <= div14 + 4'd1;
        end

        if (div28 == 5'd27) begin
            div28 <= 5'd0;
            ce_ikaopm_3m375 <= 1'b1;
        end else begin
            div28 <= div28 + 5'd1;
        end

        if (phase1_sum >= 9'd189) begin
            phase1 <= phase1_sum[7:0] - 8'd189;
            ce_oki_1m <= 1'b1;
        end else begin
            phase1 <= phase1_sum[7:0];
        end
    end
end

endmodule
