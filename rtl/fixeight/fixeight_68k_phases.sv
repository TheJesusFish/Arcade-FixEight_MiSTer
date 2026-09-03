// Alternating PHI1/PHI2 enables: each phase is exactly 16 MHz from 94.5 MHz.
module fixeight_68k_phases (
    input  logic clk,
    input  logic reset,
    output logic en_phi1,
    output logic en_phi2
);

logic [7:0] phase;
logic       next_phi2;
logic [8:0] phase_sum;

// fx68k consumes one 16 MHz pulse for each half phase, so the combined
// event stream is 32 MHz: 64/189 of the 94.5 MHz system domain.
assign phase_sum = {1'b0, phase} + 9'd64;

always_ff @(posedge clk) begin
    en_phi1 <= 1'b0;
    en_phi2 <= 1'b0;
    if (reset) begin
        phase <= 8'd0;
        next_phi2 <= 1'b0;
    end else if (phase_sum >= 9'd189) begin
        phase <= phase_sum[7:0] - 8'd189;
        if (next_phi2)
            en_phi2 <= 1'b1;
        else
            en_phi1 <= 1'b1;
        next_phi2 <= ~next_phi2;
    end else begin
        phase <= phase_sum[7:0];
    end
end

endmodule
