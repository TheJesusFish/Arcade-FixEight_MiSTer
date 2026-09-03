// Convert framework active-low controls to the active-high TP-026 ports.
module fixeight_inputs (
    input  logic [6:0] joy1_n,
    input  logic [6:0] joy2_n,
    input  logic [6:0] joy3_n,
    input  logic [3:0] start_n,
    input  logic [3:0] coin_n,
    input  logic       tilt_n,
    input  logic       test_n,
    input  logic       region_reset_n,
    output logic [15:0] p1,
    output logic [15:0] p2,
    output logic [15:0] p3,
    output logic [15:0] system
);

function automatic logic [5:0] player_normal(input logic [6:0] joy_n);
    begin
        // MAME port bits are U,D,L,R,B1,B2. JTFrame joystick bits are
        // Right,Left,Down,Up,B1,B2,B3, so reverse the direction nibble.
        player_normal = {
            ~joy_n[5], ~joy_n[4],
            ~joy_n[0], ~joy_n[1], ~joy_n[2], ~joy_n[3]
        };
    end
endfunction

always_comb begin
    // P1/P2 bit 6 is the nonstandard suicide cheat and is deliberately zero.
    p1 = {8'h00, 1'b0, 1'b0, player_normal(joy1_n)};
    p2 = {8'h00, 1'b0, 1'b0, player_normal(joy2_n)};
    // P3 bit 6 is Start 3; bit 7 is the service region-reset input.
    p3 = {
        8'h00, ~region_reset_n, ~start_n[2], player_normal(joy3_n)
    };
    // bit 0 coin3, 1 tilt, 2 test, 3 coin1, 4 coin2, 5/6 starts.
    system = {
        8'h00, 1'b0, ~start_n[1], ~start_n[0], ~coin_n[1],
        ~coin_n[0], ~test_n, ~tilt_n, ~coin_n[2]
    };
end

endmodule
