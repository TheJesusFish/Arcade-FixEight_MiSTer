//
// 8086 ALU and flags logic, derived from z8086 by nand2mario.
//
module batsugun_z8086_alu (
    input  [4:0]  op,           // ALUOPC
    input  [15:0] src,
    input  [15:0] dst,
    input         is_byte,
    input  [15:0] flags,
    input         update_carry, // gate CF updates

    output [15:0] result,
    output [15:0] flags_out
);

// -----------------------------------------------------------------------------
// Per-bit ALU slice
// Returns {carry_out, result}
// -----------------------------------------------------------------------------
function [1:0] alu_slice_fn;
    input arg1;
    input arg2;
    input carry_in;
    input value_right;
    input shift_right;
    input ctrl_c0;
    input ctrl_c1;
    input ctrl_00;
    input ctrl_01;
    input ctrl_10;
    input ctrl_11;

    reg carry_gen;
    reg carry_prop;
begin
    // Select terms based on {arg1,arg2}
    case ({arg1, arg2})
        2'b00: begin
            carry_gen  = 1'b0;
            carry_prop = ctrl_00;
        end
        2'b01: begin
            carry_gen  = 1'b0;
            carry_prop = ctrl_01;
        end
        2'b10: begin
            carry_gen  = ctrl_c0;
            carry_prop = ctrl_10;
        end
        2'b11: begin
            carry_gen  = ctrl_c1;
            carry_prop = ctrl_11;
        end
    endcase

    // Include shift-right injected value
    carry_prop = carry_prop | (value_right & shift_right);

    // {carry_out, result}
    alu_slice_fn[1] = carry_gen | (carry_prop & carry_in);
    alu_slice_fn[0] = carry_prop ^ carry_in;
end
endfunction

// opcodes
localparam [4:0] ALU_ADD  = 5'b00000;
localparam [4:0] ALU_OR   = 5'b00001;
localparam [4:0] ALU_ADC  = 5'b00010;
localparam [4:0] ALU_SBB  = 5'b00011;
localparam [4:0] ALU_AND  = 5'b00100;
localparam [4:0] ALU_SUBT = 5'b00101;
localparam [4:0] ALU_XOR  = 5'b00110;
localparam [4:0] ALU_CMP  = 5'b00111;

localparam [4:0] ALU_ROL  = 5'b01000;
localparam [4:0] ALU_ROR  = 5'b01001;
localparam [4:0] ALU_LRCY = 5'b01010;
localparam [4:0] ALU_RRCY = 5'b01011;
localparam [4:0] ALU_SHL  = 5'b01100;
localparam [4:0] ALU_SHR  = 5'b01101;
localparam [4:0] ALU_SETMO= 5'b01110;
localparam [4:0] ALU_SAR  = 5'b01111;

localparam [4:0] ALU_PASS = 5'b10000;
// localparam [4:0] ALU_XI   = 5'b10001;  // not implemented here

localparam [4:0] ALU_DAA  = 5'b10100;
localparam [4:0] ALU_DAS  = 5'b10101;
localparam [4:0] ALU_AAA  = 5'b10110;
localparam [4:0] ALU_AAS  = 5'b10111;

localparam [4:0] ALU_INC  = 5'b11000;
localparam [4:0] ALU_DEC  = 5'b11001;
localparam [4:0] ALU_NOT  = 5'b11010;
localparam [4:0] ALU_NEG  = 5'b11011;

localparam [4:0] ALU_INC2 = 5'b11100;
localparam [4:0] ALU_DEC2 = 5'b11101;

// ----------------------------------------------------------------
// Control signals common to all slices for this op
// ----------------------------------------------------------------
reg        ctrl_c0, ctrl_c1;
reg        ctrl_00, ctrl_01, ctrl_10, ctrl_11;
reg        shift_right;
reg [15:0] arg1_bus;
reg [15:0] arg2_bus;
reg [15:0] val_right_bus;
reg        carry_in0;

// use_override: when 1, bypass slices and use result_override
reg [15:0] result_override;
reg        use_override;

`define CTRL(x) {ctrl_c0, ctrl_c1, ctrl_00, ctrl_01, ctrl_10, ctrl_11} = x;

// ----------------------------------------------------------------
// Configure control/bus signals from op/src/dst/flags
// ----------------------------------------------------------------
// Decimal/ASCII adjust helper signals (pre-op src/flags)
wire daa_low_adj  = (src[3:0] > 4'd9) || flags[4];
wire daa_high_adj = (src[7:0] > 8'h99) || flags[0];
wire [7:0] daa_adj8 = ({8{daa_low_adj}}  & 8'h06) |
                        ({8{daa_high_adj}} & 8'h60);
wire aaa_cond = (src[3:0] > 4'd9) || flags[4];
wire [15:0] aaa_adj = aaa_cond ? 16'h0106 : 16'h0000;

always @* begin
    // sane defaults
    `CTRL(6'b000011);
    shift_right   = 1'b0;
    // Default to src,dst ordering to match alu1 semantics
    arg1_bus      = src;
    arg2_bus      = dst;
    val_right_bus = 16'h0000;
    carry_in0     = 1'b0;

    result_override = dst;
    use_override    = 1'b0;

    case (op)
        // ---------------- ARITHMETIC (ADD-style slice controls) --------
        ALU_ADD,
        ALU_ADC,
        ALU_SUBT,
        ALU_SBB,
        ALU_CMP,
        ALU_INC,
        ALU_DEC,
        ALU_INC2,
        ALU_DEC2,
        ALU_NEG: begin
            // full adder G/P for A+B
            // G: 00->0 01->0 10->0 11->1
            // P: 00->0 01->1 10->1 11->0
            `CTRL(6'b010110);
            arg1_bus = src;
            arg2_bus = dst;
            carry_in0 = 1'b0;

            case (op)
                ALU_ADD: begin
                    arg1_bus  = src;
                    arg2_bus  = dst;
                    carry_in0 = 1'b0;
                end

                ALU_ADC: begin
                    arg1_bus  = src;
                    arg2_bus  = dst;
                    carry_in0 = flags[0];  // CF in flags[0]
                end

                ALU_SUBT,
                ALU_CMP: begin
                    arg1_bus  = src;
                    arg2_bus  = ~dst;
                    carry_in0 = 1'b1;      // +1 for two's complement
                end

                ALU_SBB: begin
                    arg1_bus  = src;
                    arg2_bus  = ~dst;
                    carry_in0 = ~flags[0]; // +1-CF
                end

                ALU_INC: begin
                    arg1_bus  = src;
                    arg2_bus  = 16'h0000;
                    carry_in0 = 1'b1;
                end

                ALU_DEC: begin
                    arg1_bus  = src;
                    arg2_bus  = ~16'h0001;
                    carry_in0 = 1'b1;
                end

                ALU_INC2: begin
                    arg1_bus  = src;
                    arg2_bus  = 16'h0002;
                    carry_in0 = 1'b0;
                end

                ALU_DEC2: begin
                    arg1_bus  = src;
                    arg2_bus  = ~16'h0002;
                    carry_in0 = 1'b1;
                end

                ALU_NEG: begin
                    arg1_bus  = 16'h0000;
                    arg2_bus  = ~src;
                    carry_in0 = 1'b1;
                end

                default: ;
            endcase
        end

        // ---------------- Decimal adjust after add/sub -----------------
        ALU_DAA: begin
            // result = src + {00,06,60,66} on low byte
            `CTRL(6'b010110);
            arg1_bus  = src;
            arg2_bus  = {8'h00, daa_adj8};
            carry_in0 = 1'b0;
        end

        ALU_DAS: begin
            // result = src - {00,06,60,66} on low byte
            `CTRL(6'b010110);
            arg1_bus  = src;
            arg2_bus  = ~{8'h00, daa_adj8};
            carry_in0 = 1'b1; // two's complement subtract
        end

        // ---------------- ASCII adjust after add/sub -------------------
        ALU_AAA: begin
            // result = src + (0106 if cond else 0000); later zero AL[7:4]
            `CTRL(6'b010110);
            arg1_bus  = src;
            arg2_bus  = aaa_adj;
            carry_in0 = 1'b0;
        end

        ALU_AAS: begin
            // result = src - (0106 if cond else 0000); later zero AL[7:4]
            `CTRL(6'b010110);
            arg1_bus  = src;
            arg2_bus  = ~aaa_adj;
            carry_in0 = 1'b1; // subtract
        end

        // ---------------- LOGICAL (no carry meaning) -------------------
        ALU_AND: begin
            // F = A & B
            `CTRL(6'b000001);
            arg1_bus = src;
            arg2_bus = dst;
        end

        ALU_OR: begin
            // F = A | B
            `CTRL(6'b000111);
            arg1_bus = src;
            arg2_bus = dst;
        end

        ALU_XOR: begin
            // F = A ^ B
            `CTRL(6'b000110);
            arg1_bus = src;
            arg2_bus = dst;
        end

        ALU_PASS: begin
            // Pass-through via slices: result = src
            `CTRL(6'b000011); // P=arg1, independent of arg2; result=P^Cin with Cin=0
            arg1_bus  = src;
            arg2_bus  = dst; // don't care
            carry_in0 = 1'b0;
        end

        ALU_NOT: begin
            // Bitwise NOT via slices: result = ~src
            `CTRL(6'b001100); // P = ~arg1
            arg1_bus  = src;
            arg2_bus  = dst; // don't care
            carry_in0 = 1'b0;
        end

        // ---------------- SHIFTS via value_right/shift_right -----------
        ALU_SHR: begin
            // logical shift right by 1 using value_right path for both sizes
            `CTRL(6'b000000);
            shift_right   = 1'b1;
            arg1_bus      = 16'h0000;
            arg2_bus      = 16'h0000;
            carry_in0     = 1'b0;
            val_right_bus = is_byte ? {7'b0, src[0], 1'b0, src[7:1]}
                                        : {/*src[0],*/ 1'b0, src[15:1]};
        end

        ALU_SAR: begin
            // arithmetic shift right by 1 using value_right path
            `CTRL(6'b000000);
            shift_right   = 1'b1;
            arg1_bus      = 16'h0000;
            arg2_bus      = 16'h0000;
            carry_in0     = 1'b0;
            val_right_bus = is_byte ? {7'b0, src[0], src[7], src[7:1]}
                                        : {src[15], src[15:1]};
        end

        // ---------------- Rotates -------------------
        // Rotate right by 1: use shift_right path with provided right values
        ALU_ROR: begin
            `CTRL(6'b000000);
            shift_right   = 1'b1;
            arg1_bus      = 16'h0000;
            arg2_bus      = 16'h0000;
            carry_in0     = 1'b0;
            val_right_bus = is_byte ? {8'b0, src[0], src[7:1]} : {src[0], src[15:1]};
        end

        // Rotate right
        ALU_RRCY: begin
            `CTRL(6'b000000);
            shift_right   = 1'b1;
            arg1_bus      = 16'h0000;
            arg2_bus      = 16'h0000;
            carry_in0     = 1'b0;
            val_right_bus = is_byte ? {8'b0, flags[0], src[7:1]} : {flags[0], src[15:1]};
        end

        // Rotate left by 1: use carry chain
        ALU_ROL: begin
            `CTRL(6'b010000); // carry_gen=src, carry_prop=0
            arg1_bus   = src;
            arg2_bus   = src;
            carry_in0  = is_byte ? src[7] : src[15]; // wrap MSB into LSB
        end

        // Rotate left through carry
        ALU_LRCY: begin
            `CTRL(6'b010000); // carry_gen=src, carry_prop=0
            arg1_bus   = src;
            arg2_bus   = src;
            carry_in0  = flags[0];
        end

        // Left shift (SHL/SAL)
        ALU_SHL: begin
            `CTRL(6'b010000); // carry_gen=src, carry_prop=0
            arg1_bus   = src;
            arg2_bus   = src;
            carry_in0  = 1'b0; // LSB gets zero
        end

        // SETMO: set mask out (all ones) via slices (P=arg2)
        ALU_SETMO: begin
            `CTRL(6'b000101); // ctrl_00=0, ctrl_01=1, ctrl_10=0, ctrl_11=1; P=arg2
            arg1_bus  = 16'h0000;
            arg2_bus  = is_byte ? 16'h00ff : 16'hffff;
            carry_in0 = 1'b0;
        end

        default: begin
            // keep defaults (PASS dst)
        end
    endcase

    // CMP only sets flags in alu1; result equals src
    if (op == ALU_CMP) begin
        result_override = src;
        use_override    = 1'b1;
    end
end

// ----------------------------------------------------------------
// 16 slices
// ----------------------------------------------------------------
wire [15:0] slice_result;
wire [15:0] slice_carry;

genvar i;
generate
    for (i = 0; i < 16; i = i + 1) begin : GEN_ALU_SLICES
        wire cin_i = (i == 0) ? carry_in0 : slice_carry[i-1];
        wire [1:0] fo = alu_slice_fn(
            arg1_bus[i],
            arg2_bus[i],
            cin_i,
            val_right_bus[i],
            shift_right,
            ctrl_c0,
            ctrl_c1,
            ctrl_00,
            ctrl_01,
            ctrl_10,
            ctrl_11
        );
        assign slice_result[i] = fo[0];
        assign slice_carry[i]  = fo[1];
    end
endgenerate

// Raw ALU result from slices / overrides
wire [15:0] alu_raw = use_override ? result_override : slice_result;

// Post-processing:
// - AAA/AAS: zero AL[7:4]
// - DAA/DAS: keep upper byte unchanged (ignore carry into bit8)
wire [15:0] alu_post = (op == ALU_AAA || op == ALU_AAS) ? {alu_raw[15:8], 4'h0, alu_raw[3:0]} :
                        (op == ALU_DAA || op == ALU_DAS) ? {src[15:8], alu_raw[7:0]} :
                        alu_raw;

// CMP is flags-only; result equals src in alu1. Keep a small output mux.
assign result = (op == ALU_CMP) ? src : alu_post;

// ----------------------------------------------------------------
// 8086 flags 
// ----------------------------------------------------------------
wire is_logic  = (op == ALU_AND) || (op == ALU_OR) || (op == ALU_XOR);
wire is_shift  = (op == ALU_SHL) || (op == ALU_SHR) || (op == ALU_SAR);
wire is_rotate = (op == ALU_ROL) || (op == ALU_ROR) || (op == ALU_LRCY) || (op == ALU_RRCY);
wire is_inc    = (op == ALU_INC);
wire is_dec    = (op == ALU_DEC);
wire is_incdec = is_inc || is_dec;
wire is_addfam = (op == ALU_ADD) || (op == ALU_ADC) || (op == ALU_INC) || (op == ALU_INC2);
wire is_subfam = (op == ALU_SUBT) || (op == ALU_SBB) || (op == ALU_CMP) || (op == ALU_DEC) || (op == ALU_DEC2) || (op == ALU_NEG);
wire is_adjust = (op == ALU_DAA) || (op == ALU_DAS) || (op == ALU_AAA) || (op == ALU_AAS);

wire [15:0] R = slice_result;
wire flag_byte_mode = is_byte || is_adjust;
wire [3:0] msb_idx = flag_byte_mode ? 4'd7 : 4'd15;
wire r_msb = R[msb_idx];
wire a_msb = is_byte ? src[7] : src[15];
wire cout_msb = slice_carry[msb_idx];
wire cin_msb  = (msb_idx == 0) ? carry_in0 : slice_carry[msb_idx-1];
wire cout_3   = slice_carry[4'd3];

reg [15:0] f2;
always @* begin
    reg cf_cand;

    f2 = flags;

    // ZF/SF/PF (unchanged for rotates; AAA/AAS undefined -> leave as-is)
    if (!is_rotate && !(op == ALU_AAA || op == ALU_AAS)) begin
        f2[7] = r_msb;
        f2[6] = flag_byte_mode ? (R[7:0] == 8'h00) : (R == 16'h0000);
        f2[2] = ~^R[7:0];
    end

    // OF: carry-in xor carry-out of MSB, with special cases for shifts/rotates
    f2[11] = cin_msb ^ cout_msb;
    if (is_shift || is_rotate) begin
        case (op)
            ALU_SAR: 
                f2[11] = 1'b0;                       // OF cleared
            ALU_ROR, ALU_RRCY:          
                f2[11] = r_msb ^ R[msb_idx-1];       // OF = MSB(result) XOR MSB-1(result)
            default: 
                f2[11] = r_msb ^ a_msb;              // OF = MSB(result) XOR MSB(src)
        endcase
    end 

    // AF
    if (is_addfam) begin
        f2[4] = cout_3;             // ADD/ADC/INC/INC2: AF from carry out of bit 3
    end else if (is_subfam) begin
        f2[4] = ~cout_3;            // SUB/SBB/CMP/NEG: AF set on borrow
    end else if (op == ALU_NOT || op == ALU_PASS || is_rotate) begin      
        f2[4] = flags[4];           // Preserve AF for NOT, PASS, ROTATE
    end else
        f2[4] = 1'b0;               // clear for others

    // CF
    cf_cand = flags[0];
    if (is_logic) begin
        cf_cand = 1'b0;
    end else if (is_shift || is_rotate) begin
        case (op)
            ALU_SHL, ALU_ROL, ALU_LRCY: cf_cand = a_msb;
            default:                    cf_cand = src[0]; // SHR, SAR, ROR, RRCY
        endcase
    end else if (is_addfam) begin
        cf_cand = cout_msb;
    end else if (is_subfam) begin
        if (op == ALU_NEG) cf_cand = (src != 16'h0000);
        else               cf_cand = ~cout_msb; // borrow
    end else if (op == ALU_SETMO) begin
        cf_cand = 1'b0;
    end
    f2[0] = update_carry ? cf_cand : flags[0];
    // SETMO forces CF=0 regardless
    if (op == ALU_SETMO) f2[0] = 1'b0;

    // Adjust ops (DAA/DAS/AAA/AAS) flag overrides per 8086
    if (op == ALU_DAA || op == ALU_DAS) begin
        f2[4]  = daa_low_adj;   // AF
        f2[0]  = daa_high_adj;  // CF
        f2[11] = 1'b0;      // OF cleared
    end else if (op == ALU_AAA || op == ALU_AAS) begin
        f2[4]  = aaa_cond;     // AF
        f2[0]  = aaa_cond;     // CF
    end
end

assign flags_out = f2;

endmodule
