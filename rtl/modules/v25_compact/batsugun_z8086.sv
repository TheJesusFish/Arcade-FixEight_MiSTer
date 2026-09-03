//
// Batsugun V25 base derived from z8086 by nand2mario.
// Upstream provenance and the microcode copyright caveat are documented in
// UPSTREAM.md next to this file.
//
`timescale 1ns/1ns

module batsugun_z8086 #(
    parameter bit POPF_HI_IS_F = 1'b1,
    parameter UCODE_FILE = "rtl/modules/v25_compact/ucode.hex"
) (
    input             clk,
    input             clock_enable,
    input             reset_n,
    output reg [19:0] addr,  // physical address
    input      [15:0] din,
    output reg [15:0] dout,
    output reg        wr,
    output reg        rd,
    output reg        io,
    output reg        word,  // 1: word access, 0: byte access
    output            data_cycle,
    input             ready, // memory or I/O operation is done
    input             intr,
    input             nmi,
    output reg        inta,  // interrupt acknowledge, expects intr num on din and ready pulse

    input             fetch_hold,
    input             instruction_start_hold,
    input             arch_load,
    input      [15:0] arch_load_ax,
    input      [15:0] arch_load_cx,
    input      [15:0] arch_load_dx,
    input      [15:0] arch_load_bx,
    input      [15:0] arch_load_sp,
    input      [15:0] arch_load_bp,
    input      [15:0] arch_load_si,
    input      [15:0] arch_load_di,
    input      [15:0] arch_load_cs,
    input      [15:0] arch_load_ds,
    input      [15:0] arch_load_es,
    input      [15:0] arch_load_ss,
    input      [15:0] arch_load_ip,
    input      [15:0] arch_load_flags,
    input             arch_load_halted,
    input             arch_flags_write,
    input      [15:0] arch_flags_in,

    output     [15:0] arch_ax,
    output     [15:0] arch_cx,
    output     [15:0] arch_dx,
    output     [15:0] arch_bx,
    output     [15:0] arch_sp,
    output     [15:0] arch_bp,
    output     [15:0] arch_si,
    output     [15:0] arch_di,
    output     [15:0] arch_cs,
    output     [15:0] arch_ds,
    output     [15:0] arch_es,
    output     [15:0] arch_ss,
    output     [15:0] arch_next_ip,
    output     [15:0] arch_flags,
    output            arch_halted,
    output            arch_segment_override,
    output     [15:0] arch_override_segment,
    output            instruction_done,
    output            instruction_idle,
    output            interrupt_deferred,

    output            v25_escape_active,
    output      [7:0] v25_escape_lead,
    output      [7:0] v25_escape_byte,
    output            v25_escape_byte_valid,
    input             v25_escape_pop,
    input             v25_escape_complete
);

//----------------------------------------------------------------- Global registers and signals

// CAPITALIZED variables like ROME are original ones from Intel documentation
// Lowercase variables are specific to this implementation

reg dbg_first_done;          // debug: first instruction finished execution

// Simulation config: control POPF high nibble behavior via plusargs
// Default is 1 (8088-style: F on high nibble). 80186 tests set this to 0.
reg popf_hi_is_f = POPF_HI_IS_F;

// Upper (BIU) register file
reg [15:0] CS = 16'hFFF0;    // segment registers
reg [15:0] DS, ES, SS;       
reg [15:0] IP;               // instruction pointer
reg [15:0] IND, OPR;         // indirect and operand regs for memory access
reg [15:0] IND_pre_update;   // IND after "move" and before "action"
reg [15:0] IND_next;

// Lower (EU) register file
reg [15:0] AX, CX, DX, BX, SP, BP, SI, DI;  // general purpose regs
reg [15:0] F = 2'b10;        // flags: 15 14 13 12 11 10 9  8  7  6  5  4  3  2  1  0
                             //                    OF DF IF TF SF ZF    AF    PF    CF
reg [15:0] F2;               // new flags from ALU
wire       OF = F[11];
wire       DF = F[10];
wire       SF = F[7];
wire       ZF = F[6];
wire       AF = F[4];
wire       PF = F[2];
wire       CF = F[0];

assign arch_ax = AX;
assign arch_cx = CX;
assign arch_dx = DX;
assign arch_bx = BX;
assign arch_sp = SP;
assign arch_bp = BP;
assign arch_si = SI;
assign arch_di = DI;
assign arch_cs = CS;
assign arch_ds = DS;
assign arch_es = ES;
assign arch_ss = SS;
assign arch_flags = F;

// ALU signals
reg [15:0] TMPA, TMPB, TMPC; // ALU temp regs
reg  [4:0] ALUOPC;           // operation 110
reg [16:0] SIGMA;            // ALU result
reg [15:0] s_bus;            // source bus
reg        s_bus_rdy;

// Microcode sequencer internal regs and signals
reg [7:0]  IR;               // IR 102: instruction opcode
reg [8:0]  AR;               // AR 112: microcode address prefix
reg [3:0]  CR;               // CR 124: microcode sequence counter
reg [12:0] SR;               // SR 126: microcode saved return address
reg [4:0]  M, M_next;        // M 104: source bus code (r/m or IR[2:0])
reg [4:0]  N, N_next;        // N 106: destination bus code (reg or IR[5:3])
reg [3:0]  X;                // X 108: ALU operation (IR[6:3])
wire       X0 = X[0];     
reg        BPE;              // base prefix enable
reg        BPL;              // base prefix low
reg        BPH;              // base prefix high
assign arch_segment_override = BPE;
assign arch_override_segment = BPH ? (BPL ? ES : SS) : (BPL ? DS : CS);
reg  [3:0] CNT;              // loop counter for MAXC,NCZ
reg        Z16;              // internal zero flag - NZ tests this for loops
reg        XC;               // extended condition code
     
// Microcode signals     
reg [20:0] uc;               // current 21-bit microcode
wire [8:0] uaddr;            // 9-bit microcode address (512 words)
reg        ROME;             // ROM-enable: `uc` is valid this cycle
wire       stall;            // memory access stalls microcode execution
assign uaddr = uaddr_pack({AR, CR});  // pack 13-bit {AR,CR} into 9 bits

wire [2:0] uc_typ;           // microcode type (LMN)
wire [4:0] uc_s;             // move source location (FGIJH)
wire [4:0] uc_d;             // move destination location (EDCBA)
assign uc_typ = uc[9:7];
assign uc_s = {uc[15:14],uc[12:11],uc[13]};
assign uc_d = {uc[16],uc[17],uc[18],uc[19],uc[20]};
reg [4:0] uc_s_l;

wire       T0 = uc_typ[2:1] == 2'd0;  // short jump
wire       T1 = uc_typ[2:1] == 2'd1;  // ALU operations
wire       T4 = uc_typ == 3'd4;       // bookkeeping
wire       T5 = uc_typ == 3'd5;       // long jump
wire       T6 = uc_typ == 3'd6;       // bus/memory operations
wire       T7 = uc_typ == 3'd7;       // long call
// Short jump
wire       NCZ   = T0 && uc[7:4] == 4'd4;
// ALU operations
wire       LRCY  = T1 && uc[7:3] == 5'b01010;
// Type 4 OPQR
wire       MAXC  = T4 && uc[6:3] == 4'd0;
wire       FLUSH = T4 && uc[6:3] == 4'd1;
wire       CF1   = T4 && uc[6:3] == 4'd2;
wire       CITF  = T4 && uc[6:3] == 4'd3;
wire       RCY   = T4 && uc[6:3] == 4'd4;
wire       CCOF  = T4 && uc[6:3] == 4'd6;
wire       SCOF  = T4 && uc[6:3] == 4'd7;
wire       WAIT  = T4 && uc[6:3] == 4'd8;
// Type 4 STU
// RNI is defined below
wire       WB    = T4 && uc[2:0] == 3'd1;
wire       CORR  = T4 && uc[2:0] == 3'd2;
wire       SUSP  = T4 && uc[2:0] == 3'd3;
wire       RTN   = T4 && uc[2:0] == 3'd4;

// Loader timing signals
wire       FC;               // loader first clock: opcode ready
wire       SC;               // loader second clock: group decode ready
wire       RNI;              // run next instruction
wire       NXT;              // next-to-last micro-instruction
reg        not_halted = 1'b1;// once this becomes 0, microcode execution is disabled
assign arch_halted = ~not_halted;

// Instruction decode
// g_* values from: https://static.righto.com/8086/groupRom.html
reg        g_inout;          // in/out instructions
reg        g_modrm_not;      // ModR/M byte exists and performs read/modify/write on its argument.
reg        g_grp345;         // group 3/4/5 opcode
reg        g_prefix;         // instruction prefix
reg        g_2br_not;        // two-byte ROM instruction (similar to out1)
reg        g_alu_top;        // top bit for ALU
reg        g_cond;           // instruction that sets/clears condition code directly
reg        g_axal;           // instruction using AL/AX
reg        g_seg_reg_bits;   // MOV instruction using a segment register
reg        g_d;              // instruction with a `d` bit
reg        g_1bl;            // One-byte logic (1BL) instruction: prefixes, HLT, condition-code instructions.
reg        g_w;              // instruction where bit 0 is byte/word indicator.
reg        g_forcebyte;      // byte operation
reg        g_len1;           // byte operation when bit 1 is set
reg        g_carry, g_carry2;// allows a carry update. 
wire       modrm_present = ~g_2br_not;
reg        g_alu_in_opcode;  // Column 12: an instruction with bits 5-3 specifying the ALU instruction.
reg        ea_uses_bp;       // rm that includes BP uses SS by default; [disp16] (mod==00 && rm==110) uses DS
reg        writes_memory;    // memory destination instruction has to end on a bus ucode (type 6)
reg        MOD1;             // jump if 1 byte offset in effective address
reg        L8;               // byte instruction
reg        L8_aux;           // aux L8 for ALU and memory operations
wire       L8_next;  
wire       L8_aux_next;  
reg        F1;               // REP prefix active, and sign in multiplication
reg        F1Z;              // REPZ vs REPNZ (IR[0])

// These are in original 8086 but not currently used in this implementation
// reg        g_imm_move;      // Column 10: immediate move instructions, bit 3 select byte vs word
// reg        g_cmc;           // Column 13: CMC instruction
// reg        g_hlt;           // Column 14: HLT instruction
// reg        g_rep_prefix;    // Column 31: REP prefix
// reg        g_seg_prefix;    // Column 32: segment prefix
// reg        g_lock_prefix;   // Column 33: LOCK prefix
// reg        g_cli;           // Column 34: CLI instruction

// Interrupt
reg        intr_pending;     // Latched INTR request
reg        nmi_pending;      // Latched NMI request
reg        nmi_r, nmi_rr;    // NMI synchronizer (edge detection)
wire       interrupt_request = nmi_pending | intr_pending;
reg        delay_interrupt;  // delay interrupt by one instruction,
                             // set after prefix / segment register instruction
assign interrupt_deferred = delay_interrupt;

// Prefetching: 3 x 16-bit words
reg [15:0] q_word [2:0];
reg  [1:0] q_rptr, q_wptr;   // 0..2 circular
reg        q_hl;             // 0: next byte low, 1: next byte high
wire [7:0] q_bus;            // prefetch queue bus
wire [2:0] q_len;
reg        q_suspended;
wire       q_full;
wire       q_empty;
reg        escape_active;
reg  [7:0] escape_lead;
wire       escape_start = q_bus == 8'h0f || q_bus == 8'h68 ||
                          q_bus == 8'h6a || q_bus == 8'hc0 || q_bus == 8'hc1;

assign v25_escape_active = escape_active;
assign v25_escape_lead = escape_lead;
assign v25_escape_byte = q_bus;
assign v25_escape_byte_valid = escape_active & ~q_empty;

// Loader
reg [1:0] loader_state = 2'd0;      // patent figure 3 state machine

// Locations
localparam [4:0] LOC_ES  = 5'b00000;    // ES / RA
localparam [4:0] LOC_CS  = 5'b00001;    // CS / RC
localparam [4:0] LOC_SS  = 5'b00010;    // SS / RS
localparam [4:0] LOC_DS  = 5'b00011;    // DS / RD
localparam [4:0] LOC_PC  = 5'b00100;    // PC (IP)
localparam [4:0] LOC_IND = 5'b00101;    // IND (offset)
localparam [4:0] LOC_OPR = 5'b00110;    // OPR (last bus word)

localparam [4:0] LOC_Q = 5'b00111;      // 7
localparam [4:0] LOC_AL = 5'b01000;     // 8
localparam [4:0] LOC_CL = 5'b01001;     // 9
localparam [4:0] LOC_DL = 5'b01010;     // A
localparam [4:0] LOC_BL = 5'b01011;     // B
localparam [4:0] LOC_TMPA = 5'b01100;   // C
localparam [4:0] LOC_TMPB = 5'b01101;   // D
localparam [4:0] LOC_TMPC = 5'b01110;   // E
localparam [4:0] LOC_F = 5'b01111;      // F
localparam [4:0] LOC_AH = 5'b10000;     // 10
localparam [4:0] LOC_CH = 5'b10001;     // 11
localparam [4:0] LOC_DH = 5'b10010;     // 12
localparam [4:0] LOC_BH = 5'b10011;     // 13
localparam [4:0] LOC_M = 5'b10010;      // 12, only in microcode
localparam [4:0] LOC_N = 5'b10011;      // 13, only in microcode
localparam [4:0] LOC_SIGMA = 5'b10100;  // 14
localparam [4:0] LOC_TMPAL = 5'b10100;  // 14, destination
localparam [4:0] LOC_TMPBL = 5'b10101;  // 15, destination
localparam [4:0] LOC_CR    = 5'b10110;  // 16, source
localparam [4:0] LOC_TMPAH = 5'b10110;  // 16, destination
localparam [4:0] LOC_TMPBH = 5'b10111;  // 17, destination
localparam [4:0] LOC_AX = 5'b11000;     // 18
localparam [4:0] LOC_CX = 5'b11001;     // 19
localparam [4:0] LOC_DX = 5'b11010;     // 1A
localparam [4:0] LOC_BX = 5'b11011;     // 1B
localparam [4:0] LOC_SP = 5'b11100;     // 1C
localparam [4:0] LOC_BP = 5'b11101;     // 1D
localparam [4:0] LOC_SI = 5'b11110;     // 1E
localparam [4:0] LOC_DI = 5'b11111;     // 1F
localparam [4:0] LOC_ONES = 5'b10101;   // 15, source
localparam [4:0] LOC_ZERO = 5'b10111;   // 17, source

localparam [4:0] ALU_LRCY = 5'b01010;
localparam [4:0] ALU_RRCY = 5'b01011;
localparam [4:0] ALU_XI = 5'b10001;

//-----------------------------------------------------------------  Bus control
typedef enum logic [2:0] {
    BUS_IDLE,
    BUS_WAIT,
    BUS_UNALIGNED,
    BUS_UNALIGNED_GAP,
    BUS_UNALIGNED_SECOND,
    BUS_DROP
} bus_t;
bus_t bus_state;

// Physical address calculation (single upper adder inputs)
// Select EA (eu) vs PC (pf) sources:
//  - EU when a Type-6 microcode op is executing or waiting for memory
//  - Otherwise PF uses CS:IP
reg         bus_pending;
reg         bus_wr;   // write cycle
reg         bus_inta; // interrupt cycle
reg         bus_io;
reg         bus_word;
reg         bus_cancel;
reg  [15:0] bus_seg;
reg  [15:0] bus_ind;
wire [19:0] bus_addr = {bus_seg, 4'b0000} + {4'b0000, bus_ind};
reg         bus_wait; // stall EU while a Type-6 bus operation is in flight
assign data_cycle = bus_wait;

// Bus access control state machine
always @(posedge clk) begin
    if (!reset_n) begin
        // Reset state: CS=FFFFh, IP=0 (physical FFFF0h)
        bus_state <= BUS_IDLE;
        bus_wait <= 1'b0;
        bus_pending <= 1'b0;
        bus_cancel <= 1'b0;
        addr <= 20'h00000;
        rd <= 1'b0;
        wr <= 1'b0;
        io <= 1'b0;
        dout <= 16'h0000;
        word <= 1'b0;
        CS <= 16'hFFFF;
        DS <= 16'h0000;
        ES <= 16'h0000;
        SS <= 16'h0000;
        IP <= 16'b0;
        AX <= 16'h0000;
        CX <= 16'h0000;
        DX <= 16'h0000;
        BX <= 16'h0000;
        SP <= 16'h0000;
        BP <= 16'h0000;
        SI <= 16'h0000;
        DI <= 16'h0000;
        TMPA <= 16'h0000;
        TMPB <= 16'h0000;
        TMPC <= 16'h0000;
        IND  <= 16'h0000;
        OPR  <= 16'h0000;
    end else if (clock_enable) begin
        // EU memory access start (type 6)
        if (ROME & ~stall & T6) begin
            automatic logic [15:0] segment;
            bus_pending <= 1'b1;
            bus_wr <= uc[6];
            bus_inta <= uc[5];   // IRQ bit
            bus_io <= 1'b0;
            bus_word <= ~L8_aux;
            case (uc[3:2])
            2'b00: segment = ES;    // DA -> ES 
            2'b01: begin
                segment = 16'h0;    // D0 -> either segment 0 or I/O port
                bus_io <= g_inout;
            end
            2'b10: segment = SS;    // DS -> SS
            2'b11:                  // DD: SS or DS
                segment = BPE ? seg_from_code({BPH, BPL}) : (ea_uses_bp ? SS : DS); 
            endcase
            bus_seg <= segment;
            bus_ind <= IND_pre_update;
        end

        if (ROME & ~stall & CORR) begin
            IP <= IP - q_len;
        end

        case (bus_state)
            BUS_IDLE: begin
                // Fetch when there is room for one 16-bit word in queue
                // EU has priority: issue EU Type-6 read when requested
                if (bus_cancel) begin
                    bus_cancel <= 1'b0;
                    rd <= 1'b0;
                    wr <= 1'b0;
                    io <= 1'b0;
                    inta <= 1'b0;
                end else if (bus_pending) begin
                    addr <= bus_addr;
                    io   <= bus_io;
                    rd   <= ~bus_wr & ~bus_inta;
                    wr   <= bus_wr;
                    inta <= bus_inta;
                    dout <= bus_word ? OPR : {8'b0, OPR[7:0]};
                    word <= bus_word;
                    bus_state <= BUS_WAIT;
                    bus_wait <= 1'b1;
                    bus_pending <= 1'b0;
                    if (~bus_io & bus_ind[0] & bus_word) begin  // unaligned word access
                        dout <= {8'b0, OPR[7:0]};
                        word <= 1'b0;
                        bus_ind <= bus_ind + 16'd1;        // prepare for next byte
                        bus_state <= BUS_UNALIGNED;
                    end
                end else if (!q_full & !q_suspended & !fetch_hold & !arch_load) begin
                    addr <= {CS, 4'b0000} + {4'b0000, IP};   // CS:IP via unified adder
                    word <= ~IP[0];    // word access if address is aligned
                    rd   <= 1'b1;
                    wr   <= 1'b0;
                    io   <= 1'b0;
                    inta <= 1'b0;
                    bus_state <= BUS_WAIT;
                end
            end
            BUS_WAIT: begin
                if (ready) begin
                    rd <= 1'b0;
                    wr <= 1'b0;
                    io <= 1'b0;
                    inta <= 1'b0;
                    if (bus_cancel) begin
                        bus_cancel <= 1'b0;
                    end else if (bus_wait) begin
                        // EU memory read / INTA completion
                        if (~bus_wr) begin
                            if (bus_word) begin
                                OPR <= din;
                            end else begin
                                OPR[7:0] <= din[7:0];
                            end
                        end
                    end else if (!q_suspended) begin  // drop data when suspended
                        // Store one fetched word into the queue
                        q_word[q_wptr] <= IP[0] ? {din[7:0], 8'b0} : din;   // byte read for odd address
                        // q_wptr         <= q_next_w;       // \ in Q management block
                        // if (IP[0]) q_hl <= 1'b1;          // /
                        IP <= {IP[15:1]+15'b1, 1'b0};
                    end
                    bus_wait  <= 1'b0;
                    bus_state <= BUS_IDLE;
                end else if (q_suspended & ~bus_wait) begin
                    bus_state <= BUS_DROP;
                end
            end
            BUS_DROP: if (ready) begin     // drop spurius prefetch data when suspended
                rd <= 1'b0;
                wr <= 1'b0;
                io <= 1'b0;
                inta <= 1'b0;
                bus_cancel <= 1'b0;
                bus_state <= BUS_IDLE;
            end
            BUS_UNALIGNED: begin      // first byte of an unaligned word access
                if (ready) begin
                    rd <= 1'b0;
                    wr <= 1'b0;
                    io <= 1'b0;
                    inta <= 1'b0;
                    if (bus_cancel) begin
                        bus_state <= BUS_IDLE;
                        bus_wait <= 1'b0;
                        bus_cancel <= 1'b0;
                    end else begin
                        addr <= bus_addr;
                        if (~bus_wr)
                            OPR[7:0] <= din[7:0];
                        if (bus_wr) dout <= {8'b0, OPR[15:8]};
                        bus_state <= BUS_UNALIGNED_GAP;
                    end
                end
            end
            BUS_UNALIGNED_GAP: begin
                // The outer bus serializer requires the request to drop
                // before it can accept the second byte.
                if (bus_cancel) begin
                    bus_cancel <= 1'b0;
                    bus_wait <= 1'b0;
                    bus_state <= BUS_IDLE;
                end else if (!ready) begin
                    rd <= ~bus_wr;
                    wr <= bus_wr;
                    bus_state <= BUS_UNALIGNED_SECOND;
                end
            end
            BUS_UNALIGNED_SECOND: begin
                if (ready) begin
                    if (~bus_cancel & ~bus_wr)
                        OPR[15:8] <= din[7:0];
                    rd <= 1'b0;
                    wr <= 1'b0;
                    io <= 1'b0;
                    inta <= 1'b0;
                    bus_cancel <= 1'b0;
                    bus_state <= BUS_IDLE;
                    bus_wait  <= 1'b0;
                end
            end
        endcase

        if (ROME & ~stall) begin
            if (uc_d == LOC_M && M == LOC_PC || uc_d == LOC_N && N == LOC_PC || uc_d == LOC_PC)
                IP <= s_bus;
        end

        // write to registers
        if (SC) begin
            TMPA <= 16'h0;
            TMPB <= 16'h0;
        end
        if (ROME && s_bus_rdy) begin
            uc_s_l <= uc_s;
            if (uc_d == LOC_M)
                write_loc(M, s_bus);
            else if (uc_d == LOC_N)
                write_loc(N, s_bus);
            else
                write_loc(uc_d, s_bus);
        end
        if (ROME & ~stall) IND <= IND_next;
        if (arch_load) begin
            AX <= arch_load_ax;
            CX <= arch_load_cx;
            DX <= arch_load_dx;
            BX <= arch_load_bx;
            SP <= arch_load_sp;
            BP <= arch_load_bp;
            SI <= arch_load_si;
            DI <= arch_load_di;
            CS <= arch_load_cs;
            DS <= arch_load_ds;
            ES <= arch_load_es;
            SS <= arch_load_ss;
            IP <= arch_load_ip;
            IND <= 16'h0000;
            OPR <= 16'h0000;
            bus_pending <= 1'b0;
            if (bus_state == BUS_IDLE) begin
                bus_state <= BUS_IDLE;
                bus_cancel <= 1'b0;
                bus_wait <= 1'b0;
                rd <= 1'b0;
                wr <= 1'b0;
                io <= 1'b0;
                inta <= 1'b0;
            end else begin
                bus_cancel <= 1'b1;
                bus_wait <= 1'b0;
            end
        end
    end
end

// Select default segment for data cycles: DS unless BP participates, then SS;
// Segment override prefix (BPE/BPH/BPL) supersedes the default.
function automatic [15:0] seg_from_code(input [1:0] code);
    case (code)
    2'b00: seg_from_code = CS; 
    2'b01: seg_from_code = DS; 
    2'b10: seg_from_code = SS; 
    default: seg_from_code = ES;
    endcase
endfunction

//-----------------------------------------------------------------  Prefetch queue

assign     q_bus = q_hl ? q_word[q_rptr][15:8] : q_word[q_rptr][7:0];   // prefetch queue bus
wire [1:0] q_next_w = (q_wptr == 2'd2) ? 2'd0 : (q_wptr + 2'd1);
assign     q_full   = (q_next_w == q_rptr);
assign     q_empty  = (q_rptr == q_wptr) | (ROME & FLUSH);
reg  [2:0] q_consumed;      // how many bytes have been consumed by the current instruction
assign     q_len =  q_rptr == q_wptr ? 3'd0 :
                      q_wptr > q_rptr ? 
                      {q_wptr - q_rptr, 1'b0} - q_hl: 
                      ((3'd3 + q_wptr - q_rptr) << 1) - q_hl;
// IP points to next fetch address
// arch_IP = IP - q_consumed - q_len
assign arch_next_ip = IP - q_len;

// Q ptr management
always @(posedge clk) begin
    if (!reset_n) begin
        q_rptr <= 0;
        q_wptr <= 0;
        q_hl   <= 0;
        q_suspended <= 0;
        q_consumed <= 0;
    end else if (clock_enable) begin
        // Q pop
        if (~q_empty & (
              FC |                          // load first byte
              (ROME & uc_s == LOC_Q & ~stall) | // source is Q
              (SC & ~g_1bl & modrm_present) | // consume ModR/M at SC for this instruction
              (escape_active & v25_escape_pop)
            )) begin   
            if (!q_hl) begin                // consumed low byte, next is high
                q_hl <= 1'b1;
            end else begin                  // consumed high byte, advance to next word
                q_hl   <= 1'b0;
                q_rptr <= (q_rptr == 2'd2) ? 2'd0 : (q_rptr + 2'd1);
            end
            q_consumed <= q_consumed + 3'b1;// increment consumed bytes when we pop a byte
        end
        if (loader_state == 2'd2 & RNI)
            q_consumed <= FC ? 3'b1 : 3'b0; // reset consumed bytes for a new instruction
        if (SC & g_1bl) 
            q_consumed <= 3'b0;             // reset consumed bytes for 1-byte logic instructions
        if (v25_escape_complete)
            q_consumed <= 3'b0;

        // write word to Q
        if (bus_state == BUS_WAIT & ready & ~bus_wait & ~q_suspended &
            ~bus_cancel) begin
            q_wptr <= q_next_w; 
            if (IP[0]) q_hl <= 1'b1;      // byte read for odd address
        end

        // suspend prefetching
        if (ROME & ~stall & SUSP) begin
            q_suspended <= 1'b1;
        end

        if (ROME & ~stall & FLUSH) begin
            q_suspended <= 1'b0;
            q_rptr <= 2'd0;
            q_wptr <= 2'd0;
            q_hl <= 1'b0;
            q_consumed <= 3'b0;
        end
        if (arch_load) begin
            q_suspended <= 1'b0;
            q_rptr <= 2'd0;
            q_wptr <= 2'd0;
            q_hl <= 1'b0;
            q_consumed <= 3'b0;
        end
    end
end

always @(posedge clk) begin
    if (!reset_n) begin
        escape_active <= 1'b0;
        escape_lead <= 8'h00;
    end else if (clock_enable) begin
        if (FC && escape_start && !interrupt_request) begin
            escape_active <= 1'b1;
            escape_lead <= q_bus;
        end
        if (v25_escape_complete || arch_load)
            escape_active <= 1'b0;
    end
end

//----------------------------------------------------------------- Loader
// Loader generates timing signals for the microcode sequencer:
// FC: when Q ready, load 1st byte into IR, uaddr(AR and CR), start group decode
// SC: group signals ready, Q ready if 2BR, loading first microcode, set microcode_active=1
// SC+1: microcode out, start execution, load uaddr_next
// SC+2: first microcode done, start executing 2nd microcode 

wire EXEC = loader_state[1];        // sequencer has control
reg RNI_null;  // WB nulls next uinstruction's RNI if EA refers to memory
// RNI and NXT as generated by ROM-OUT 144 to LOADER 130
assign RNI = ROME & ~RNI_null & ~bus_pending & ~bus_wait &
                 (T6 & uc[4] |                     // Letter Q when Typ=6 
                  T4 & uc[2:0] == 3'b000 & ~writes_memory);   // STU=0 when Typ=4
assign NXT = ROME & T1 & uc[0] & ~bus_pending & ~bus_wait & ~writes_memory;
wire uc_f = uc[10] == 1'b1;
assign instruction_done = clock_enable &
                          ((ROME & RNI) | (SC & g_1bl & ~g_prefix));
assign instruction_idle = loader_state == 2'd0 &&
                          !ROME && !escape_active;

always @(posedge clk) begin
    if (clock_enable && EXEC & ~stall) begin
        RNI_null <= 1'b0;
        if (WB & M == LOC_OPR) RNI_null <= 1'b1;
    end
end

// First Clock: ready to read the first byte of the instruction from Q
// Waiting for RNI avoids the upstream NXT overlap corrupting a following
// multi-byte instruction (the Batsugun reset sequence exposes this with
// DEC AX immediately followed by JMP FAR).
assign FC = (loader_state == 2'd0 | loader_state == 2'd2 & RNI) &
            q_full & ~(ROME & FLUSH) & ~escape_active & ~fetch_hold &
            ~instruction_start_hold;
// Second Clock: group decode is ready, ready to read the 2nd byte for 2BR instructions
//               or start microcode execution for 1-byte instructions
assign SC = (loader_state == 2'd1 | loader_state == 2'd3) &
            (g_2br_not | ~q_empty) & ~escape_active;

// translation ROM signals
wire t_mem = SC;
wire [4:0] t_mode_rm = t_mem ? {q_bus[7:6], q_bus[2:0]} :  // modrm: {mode[1:0], rm[2:0]}
                               {uc_typ[1], uc[3:0]};    // jump/call: {is_call, dest[3:0]}
reg t_disp_not_r;
wire t_disp_not = SC ? ~(q_bus[7] ^ q_bus[6]) : t_disp_not_r;  // mode=00 or 11
always @(posedge clk) if (clock_enable) t_disp_not_r <= t_disp_not;
wire t_mem_read_not = g_modrm_not;  // ModR/M byte exists and performs read/modify/write on its argument.
// translation ROM output: goes to {AR, CR} for modrm and jump/call
wire [12:0] t_out = translate(t_mem, t_mode_rm, t_disp_not, t_mem_read_not);  

// Loader state machine
always @(posedge clk) begin
    if (!reset_n) begin
        loader_state <= 2'd0;
    end else if (clock_enable) begin
        if (escape_active || arch_load) begin
            loader_state <= 2'd0;
        end else case (loader_state) 
        2'd0: if (FC)                loader_state <= 2'd1;
        2'd1: if (g_1bl)             loader_state <= 2'd0;
              else if (SC)           loader_state <= 2'd2;
        2'd2: if (FC & RNI)          loader_state <= 2'd1;
              else if (~FC & RNI)    loader_state <= 2'd0;
              else if (FC & NXT)     loader_state <= 2'd3;
        2'd3: if (SC & ~g_1bl)       loader_state <= 2'd2;
              else if (~SC)          loader_state <= 2'd1;
              else if (g_1bl)        loader_state <= 2'd0;
        endcase
    end
end

//----------------------------------------------------------------- Microcode sequencer

reg [23:0] ucode_rom [0:511];
initial begin
    $readmemh(UCODE_FILE, ucode_rom);
end

assign stall = (ROME & q_empty & (EXEC & uc_s == LOC_Q)) | bus_wait | bus_pending;
assign L8_next = g_w & ~IR[0] | g_len1 & IR[1] | g_forcebyte | IR[7:3] == 5'b1011_0;    // byte instruction
assign L8_aux_next = (g_w & ~IR[0] | g_forcebyte | IR[7:3] == 5'b1011_0);
always @(posedge clk) begin
    if (clock_enable) begin
        if (SC) begin 
            L8 <= L8_next; 
            L8_aux <= L8_aux_next; 
        end
        if (ROME & ~stall & AR[8:5] == 4'b1000)   // INTR routine needs word operation
            L8_aux <= 1'b0;
    end
end

// Microcode sequencing is a 3-stage pipeline:
// Stage 0 (FC): opcode (q_bus) -> u-addr ({AR,CR}). Once per instruction.
// Stage 1 (SC | loader_state[1] & ~stall): u-addr -> microcode, CR++, set ROME=1
// Stage 2 (ROME & ~stall): execute microcode
//  - if jump, ROME=0 and load new u-addr
always @(posedge clk) begin
    if (!reset_n) begin
        ROME <= 1'b0;
        dbg_first_done <= 1'b0;
        not_halted <= 1'b1;
    end else if (clock_enable) begin
        if (~stall | SC)
            uc <= ucode_rom[uaddr][20:0];

        // Advance CR and set ROME
        ROME <= stall ? ROME : 1'b0;
        if (EXEC & ~RNI & ~stall) begin  // normal execution
            ROME <= not_halted;        
            CR <= CR + 4'b1;
        end

        // First cycle: load new instruction
        if (FC && !escape_start) begin                           
            if (interrupt_request & ~delay_interrupt) begin
                // On interrupt, go to 1r0000000.00ab 
                // ab: 00 (trap), 01 (NMI), 10 (INTR)
                {AR, CR} <= {9'b1_0000_0000, 2'b00, nmi_pending ? 2'b01 : 2'b10};  // INTR microcode address
                IR <= 8'hCD;          // INT n

                // Block normal group decode
                {g_carry, g_len1, g_forcebyte, g_w, g_1bl, g_d,
                 g_seg_reg_bits, g_axal, g_cond, g_alu_top,
                 g_2br_not, g_prefix, g_grp345, g_modrm_not, g_inout} <= 15'b0;
                g_2br_not <= 1'b1;        // block 2nd byte
                g_alu_in_opcode <= 1'b0;
                M_next <= 3'b0;
                N_next <= 3'b0;
            end else begin
                // Set microcode entry address
                {AR, CR} <= {q_bus, 4'b0};
                IR <= q_bus;

                // group decode
                {g_carry, g_len1, g_forcebyte, g_w, g_1bl, g_d, g_seg_reg_bits, g_axal, g_cond, g_alu_top,
                 g_2br_not, g_prefix, g_grp345, g_modrm_not, g_inout} <= group_decode(q_bus);
                // non-group instructions
                g_alu_in_opcode <= ~(q_bus[7:4] == 4'hF & q_bus[2:1] == 2'b11 |   // group345
                                     q_bus[3:2] == 2'b0 & (q_bus[7:4] == 4'h8 | q_bus[7:4] == 4'hD)); // group 1 and 2

                // Store M/N for decode in SC and later
                M_next <= q_bus[2:0];               // M_next so NXT does not overwrite M
                N_next <= q_bus[5:3];
            end
        end
        // Second cycle: group decode complete
        if (SC & IR == 8'hF4) not_halted <= 1'b0; // handle HLT
        if (arch_load) not_halted <= ~arch_load_halted;
        if (SC & ~g_1bl) begin                  // Second Cycle
            ROME <= not_halted;
            CR <= CR + 4'b1;
            writes_memory <= 1'b0;
            // Capture ModR/M now
            if (modrm_present) begin
                automatic reg [4:0] M_expand, N_expand;
                automatic reg [8:0] AR_updated;   // Instruction start AR including GRP345
                // more decoding
                writes_memory <= 
                     q_bus[7:6] != 2'b11 & 
                     (g_d & ~IR[1] |          // ALU Eb/Ev and MOV Eb/Ev
                     IR[7:2] == 6'b1000_00 |  // GRP1
                     ~g_2br_not & IR[7:6] == 2'b11 & g_w // C6h,C7h,GRP2,GRP3a,GRP3b,GRP4,GRP5
                     );                     
                if (IR[7:1] == 7'b1111_011 && q_bus[5:4] != 2'b01) writes_memory <= 1'b0; // TEST/MUL/DIV in GRP3a/b, no memory write
                if (IR == 8'hFF && q_bus[5:4] == 2'b10) writes_memory <= 1'b0; // JMP FAR rm
                if (IR[7:3] == 5'b00111 ||
                    (IR[7:2] == 6'b1000_00 && q_bus[5:3] == 3'b111))
                    writes_memory <= 1'b0; // CMP updates flags without writing the operand back
                ea_uses_bp <= (q_bus[7:6] != 2'b11) &
                                (q_bus[2:0] == 3'b010 | q_bus[2:0] == 3'b011 |
                                (q_bus[2:0] == 3'b110 & q_bus[7:6] != 2'b00));
                MOD1 <= q_bus[6];

                // Map REG field to N
                // This does not use L8_next because https://www.righto.com/2023/07/undocumented-8086-instructions.html:
                // "However, the register operations use only bit 0 to select a byte or word transfer."
                if (g_seg_reg_bits)
                    N_expand = {3'b0, q_bus[4:3]};
                else
                    N_expand = gpr_bus_code(q_bus[5:3], ~L8_aux_next, 1'b0);  // normal case

                // Update instruction start address for Group 3-5
                AR_updated = AR;
                if (SC & g_grp345) begin  // GRP3 -> F0h-F7h, GRP4,GRP5 -> F8h-FFh
                    AR_updated[2:0] = q_bus[5:3];
                    AR_updated[8] = IR[0];// MSB of AR should be IR[0] for group 3-5.
                    CR <= 4'b0;      
                    ROME <= 1'b0;         // bubble as AR is changed
                end

                // For register-only addressing (mod==11), map r/m to M directly
                if (q_bus[7:6] == 2'b11) begin
                    M_expand = gpr_bus_code(q_bus[2:0], ~L8_aux_next, 1'b0);
                    AR <= AR_updated;     // start directly from instruction start address
                end else begin
                    // Jump to EA microcode via translate() and save instruction start address
                    {AR, CR} <= t_out;    // EA microcode address
                    SR       <= {AR_updated, 4'b0}; // Save instruction start address
                    M_expand =  LOC_OPR;  // M is OPR for memory addressing
                    ROME     <= 1'b0;     // bubble like a microcode jump
                end

                if (g_d & IR[1]) begin  // swap M and N for instruction with d bit
                    M <= N_expand;
                    N <= M_expand;
                end else begin          // normal case
                    M <= M_expand;
                    N <= N_expand;
                end
            end else begin
                // No ModR/M: decode M and R from opcode fields
                M <= gpr_bus_code(M_next[2:0], ~L8_aux_next, g_axal); // decode M into 5-bit location code
                if (IR[7:5] == 3'b0 && IR[2:1] == 2'b11)    // PUSH/POP seg
                    N <= {3'b0, N_next[1:0]};       // N is segment register
                else
                    N <= gpr_bus_code(N_next[2:0], ~L8_aux_next, g_axal); // normal case
                ea_uses_bp <= 1'b0;
            end
        end
        // Branches
        if (ROME & ~stall) begin
            automatic reg taken;
            // Microcode execution: deal with jumps
            taken = 1'b0;
            if (T0 | T5 | T7) begin
                casez (uc[9:4])
                /// Both long and short jumps
                6'b??1_000: taken = 1'b1;  // UNC
                6'b??1_010: taken = ~Z16;  // NZ tests Z16
                6'b??1_011: taken = X0;    // X0
                6'b??1_100: taken = ~CF;   // NCY
                6'b??1_101: taken = F1;    // F1 flag is active
                // 6'b??1_110: taken = INT;   // INT
                6'b??1_111: taken = XC;    // XC
                /// short jump only
                6'b000_000: taken = ZF ^ F1Z; // F1ZZ
                6'b000_001: taken = MOD1; 
                6'b000_010: taken = L8;   // L8: byte instruction or B0-B7
                6'b000_011: taken = ZF;   // Z
                6'b000_100: taken = CNT != 4'd0; // NCZ, Not Counter Zero
                // 6'b??1_101: // TEST pin
                6'b000_110: taken = OF;   // OF
                6'b000_111: taken = CF;   // CY
                6'b001_001: taken = ~F1;  // NF1: F1 flag is not active
                default: taken = 1'b0;
                endcase
                if (taken) begin
                    ROME <= 1'b0;    // one-cycle bubble as next ucode is useless
                    if (uc_typ[2]) begin  // far jump/call
                        {AR, CR} <= t_out;
                        if (uc_typ[1:0] == 2'b11) begin  // call - save return address
                            SR <= {AR, CR};
                        end
                    end else begin         // near jump/call
                        CR <= uc[3:0];
                    end
                end
            end
            // Type 6: bus ops, pause ucode
            // if (uc_typ == 3'd6) ROME <= 1'b0;
            // Type 4: bookkeeping operations
            if (RTN) begin      // Return to saved micro-address (used by EALOAD/EADONE)
                {AR, CR} <= SR;
                ROME <= 1'b0;
            end
        end
        // Finishing instruction
        if (ROME & RNI | SC & g_1bl & ~g_prefix) begin  // mark 1st instruction done
            dbg_first_done <= 1'b1;
        end
        if (escape_active || arch_load)
            ROME <= 1'b0;
    end
end

// prefix handling
always @(posedge clk) begin
    if (!reset_n) begin
        BPE <= 1'b0;
        BPL <= 1'b0;
        BPH <= 1'b0;
    end else if (clock_enable) begin
        if (SC) begin
            if (g_prefix & ~IR[7]) begin
                BPE <= 1'b1;
                case ({IR[4:3]})
                2'b00: {BPH,BPL} <= 2'b11;  // 26: ES
                2'b01: {BPH,BPL} <= 2'b00;  // 2E: CS
                2'b10: {BPH,BPL} <= 2'b10;  // 36: SS
                2'b11: {BPH,BPL} <= 2'b01;  // 3E: DS
                endcase
            end
        end else if (ROME & RNI) begin
            BPE <= 1'b0;
            BPL <= 1'b0;
            BPH <= 1'b0;
        end
        if (arch_load || v25_escape_complete) begin
            BPE <= 1'b0;
            BPL <= 1'b0;
            BPH <= 1'b0;
        end
    end
end

// Extended Condition Codes
always @* begin
    case (IR[3:1])
    3'h0: XC = OF;
    3'h1: XC = CF; 
    3'h2: XC = ZF;  
    3'h3: XC = CF | ZF;
    3'h4: XC = SF; 
    3'h5: XC = PF; 
    3'h6: XC = SF ^ OF;
    default: XC = (SF ^ OF) | ZF;
    endcase
    if (IR[0]) XC = ~XC;
end

// Interrupt delay (8086 interrupt bug)
always @(posedge clk) begin
    if (!reset_n) begin
        delay_interrupt <= 1'b0;
    end else if (clock_enable) begin
        if (SC & (g_prefix | 
                  IR[7:5] == 8'h17 |     // POP SS         
                  g_seg_reg_bits         // MOV sr,xxx
            )) delay_interrupt <= 1'b1;
        if (FC) delay_interrupt <= 1'b0; // clear delay after next instruction
        if (arch_load) delay_interrupt <= 1'b0;
    end
end

//----------------------------------------------------------------- External Interrupt

// Sample external interrupts at instruction completion
always @(posedge clk) begin
    if (~reset_n) begin
        intr_pending <= 1'b0;
        nmi_pending <= 1'b0;
        nmi_r <= 1'b0;
        nmi_rr <= 1'b0;
    end else if (clock_enable) begin
        // NMI synchronizer and edge detection
        nmi_r <= nmi;
        nmi_rr <= nmi_r;
        // NMI has priority and cannot be masked
        if (nmi_r & ~nmi_rr) begin
            nmi_pending <= 1'b1;
        end

        // Instruction completion boundary
        if (ROME & RNI | SC & g_1bl & ~g_prefix) begin
            // INTR checked only if IF=1 (interrupts enabled)
            if (intr & F[9] & ~intr_pending) begin
                intr_pending <= 1'b1;
            end
        end

        // Clear pending flags when interrupt is serviced
        if (FC & interrupt_request & ~delay_interrupt) begin
            if (nmi_pending) nmi_pending <= 1'b0;
            else if (intr_pending) intr_pending <= 1'b0;
        end
        if (arch_load) begin
            intr_pending <= 1'b0;
            nmi_pending <= 1'b0;
        end
    end
end

//----------------------------------------------------------------- Internal FFs

// Flag updates
always @(posedge clk) begin
    if (!reset_n) begin
        F <= 16'h0002;
        F1 <= 1'b0;
        F1Z <= 1'b0;
    end else if (clock_enable) begin
        if (ROME & ~stall) begin
            if (uc_f) F <= F2;  // update flags from ALU
            if (s_bus_rdy && uc_d == LOC_F) begin // setting F from s_bus
                F <= s_bus;
                // POPF sets fixed values for F[15:12], F[5], F[3], F[1]
                F[5] <= 1'b0; F[3] <= 1'b0; F[1] <= 1'b1;   
                F[15:12] <= popf_hi_is_f ? 4'hF : 4'h0;    // undefined bits differ in tests
            end
            if (CF1) F1 <= ~F1;      // Invert F1 flag
            if (RCY) F[0] <= 1'b0;   // Reset carry
            if (CCOF) begin          // Clear carry and overflow
                F[11] <= 1'b0;
                F[0] <= 1'b0;
            end
            if (SCOF) begin          // Set carry and overflow
                F[11] <= 1'b1;
                F[0] <= 1'b1;
            end
            if (CITF) begin          // Clear interrupt and trap flags
                F[8] <= 1'b0;
                F[9] <= 1'b0;
            end
            if ((ALUOPC == ALU_LRCY || ALUOPC == ALU_RRCY) && uc_s == LOC_SIGMA) begin 
                F[0] <= F2[0];      // LRCY/RRCY updates CF when SIGMA is source. used in CORD
            end
        end
        if (SC & g_1bl) begin  // 1-byte logic instructions
            case (IR[3:0])
            4'h5: F[0] <= ~F[0];  // CMC
            4'h8: F[0] <= 1'b0;   // CLC
            4'h9: F[0] <= 1'b1;   // STC
            4'hA: F[9] <= 1'b0;   // CLI
            4'hB: F[9] <= 1'b1;   // STI
            4'hC: F[10] <= 1'b0;  // CLD
            4'hD: F[10] <= 1'b1;  // STD
            default: ;
            endcase
        end
        // Prefix flags
        if (SC) begin
            if (g_prefix & IR[7]) F1 <= 1'b1;          // REP/REPZ/REPE active
            if (IR[7:6] == 2'b11) F1Z <= IR[0];        // REPNZ/REPZ/LOOPNZ/LOOPZ active
        end
        if (SC & g_1bl & ~g_prefix | ROME & ~stall & RNI) begin
            F1 <= 1'b0;
        end
        if (arch_flags_write)
            F <= arch_flags_in;
        if (arch_load) begin
            F <= arch_load_flags;
            F1 <= 1'b0;
            F1Z <= 1'b0;
        end
    end
end

// Loop counter
always @(posedge clk) begin
    if (clock_enable && ROME & ~stall) begin
        if (MAXC) CNT <= L8 ? 4'h7 : 4'hF;
        if (NCZ) CNT <= CNT - 4'b1;
    end
end

// Internal condition flags
always @(posedge clk) begin
    if (clock_enable && ROME & ~stall) begin
        // NZ tests Z16. Z16 is updated when SIGMA is referenced as a move source.
        // So to update Z16 only, one uses "SIGMA -> no dest" in microcode.
        if (uc_s == LOC_SIGMA) Z16 <= SIGMA[15:0] == 16'd0;
    end
end

//----------------------------------------------------------------- ALU
// ALU operations
reg [1:0] alu_src;
always @(posedge clk) begin
    if (!reset_n) begin
    end else if (clock_enable) begin
        if (FC) begin
            X <= q_bus[6:3];
        end
        if (SC & ~g_alu_in_opcode) begin
            if ((IR == 8'hF6 || IR == 8'hF7) && q_bus[5:3] == 3'b000)
                X[2:0] <= 3'b100;         // Force AND for TEST in GRP3
            else
                X[2:0] <= q_bus[5:3];     // overwrite X[5:3] with modrm[5:3] for GRP instructions
        end
        if (SC) begin  // for EA address calculation
            ALUOPC <= 5'b0;
            alu_src <= 2'b0;  // TMPA
        end
        if (ROME & uc_typ[2:1] == 2'b01) begin
            ALUOPC <= uc[7:3] == ALU_XI ? {g_alu_top, X} : uc[7:3];
            alu_src <= uc[2:1];
        end
    end
end

batsugun_z8086_alu alu (
    .op(ALUOPC),
    .src(alu_src == 2'b00 ? TMPA : alu_src == 2'b01 ? TMPB : TMPC),
    .dst(TMPB),
    .is_byte(L8_aux),
    .flags(F),
    .update_carry(g_carry2),
    .result(SIGMA[15:0]),
    .flags_out(F2)
);

always @(posedge clk) begin
    if (clock_enable && SC) g_carry2 <= g_carry;       // late version of g_carry: updated *after* SC
end

//----------------------------------------------------------------- S bus read
// Upper register location codes (explicit names for readability when used as destinations)

always_comb begin
    s_bus_rdy = 1'b1;
    case (uc_s)
    LOC_M:    s_bus = read_loc(M);           // use M’s current location code
    LOC_N:    s_bus = read_loc(N);           // use N’s current location code (REG field)
    LOC_Q: begin
        // Keep queue data in this block's explicit sensitivity set. Some tools
        // do not include globals referenced only from inside a function.
        s_bus     = {8'h00, q_bus};
        s_bus_rdy = !q_empty;
    end
    LOC_SIGMA: s_bus = SIGMA[15:0];
    default:  s_bus = read_loc(uc_s);
    endcase
end

function automatic [15:0] read_loc(input [4:0] loc);
    begin
        case (loc)
        // Upper registers (EDCBA = 00000..00110)
        LOC_ES:   read_loc = ES;                // RA (ES)
        LOC_CS:   read_loc = CS;                // RC (CS)
        LOC_SS:   read_loc = SS;                // RS (SS)
        LOC_DS:   read_loc = DS;                // RD (DS)
        LOC_PC:   read_loc = IP;                // PC (IP)
        LOC_IND:  read_loc = IND;               // IND
        LOC_OPR:  read_loc = OPR;               // OPR
        LOC_AL:   read_loc = {8'h00, AX[7:0]};  // AL as 16-bit
        LOC_AH:   read_loc = {8'h00, AX[15:8]}; // AH as 16-bit
        LOC_CL:   read_loc = {8'h00, CX[7:0]};  // CL as 16-bit
        LOC_CH:   read_loc = {8'h00, CX[15:8]}; // CH as 16-bit
        LOC_DL:   read_loc = {8'h00, DX[7:0]};  // DL as 16-bit
        LOC_DH:   read_loc = {8'h00, DX[15:8]}; // DH as 16-bit
        LOC_BL:   read_loc = {8'h00, BX[7:0]};  // BL as 16-bit
        LOC_BH:   read_loc = {8'h00, BX[15:8]}; // BH as 16-bit
        LOC_F:    read_loc = F;                 // F
        LOC_CR:   read_loc = CR;                // CR
        LOC_AX:   read_loc = AX;                // AX
        LOC_CX:   read_loc = CX;                // CX
        LOC_DX:   read_loc = DX;                // DX
        LOC_BX:   read_loc = BX;                // BX
        LOC_SP:   read_loc = SP;                // SP
        LOC_BP:   read_loc = BP;                // BP
        LOC_SI:   read_loc = SI;                // SI / IJ
        LOC_DI:   read_loc = DI;                // DI / IK
        LOC_TMPA: read_loc = TMPA;              // TMPA
        LOC_TMPB: read_loc = TMPB;              // TMPB
        LOC_TMPC: read_loc = TMPC;              // TMPC
        LOC_ONES: read_loc = 16'hFFFF;          // ONES
        LOC_ZERO: read_loc = 16'h0000;          // ZERO
        LOC_Q:    read_loc = {8'h00, q_bus};    // Q as 16-bit
        LOC_SIGMA:read_loc = SIGMA[15:0];       // SIGMA
        default:  read_loc = 16'hXXXX;
        endcase
    end
endfunction

//-----------------------------------------------------------------  D bus write
task automatic write_loc(input [4:0] loc, input [15:0] value);
    begin
        case (loc)
        // Upper registers (EDCBA = 00000..00110)
        LOC_ES:    ES <= value;                   // ES
        LOC_CS:    CS <= value;                   // CS
        LOC_SS:    SS <= value;                   // SS
        LOC_DS:    DS <= value;                   // DS
        // LOC_PC:    IP <= value;                   // PC (IP)
        LOC_OPR:   OPR <= value;                  // OPR (last bus word)
        LOC_AX:    AX <= value;                   // AX
        LOC_CX:    CX <= value;                   // CX
        LOC_DX:    DX <= value;                   // DX
        LOC_BX:    BX <= value;                   // BX
        LOC_SP:    SP <= value;                   // SP
        LOC_BP:    BP <= value;                   // BP
        LOC_SI:    SI <= value;                   // SI
        LOC_DI:    DI <= value;                   // DI
        LOC_AL:    AX[7:0]  <= value[7:0];        // AL
        LOC_CL:    CX[7:0]  <= value[7:0];        // CL
        LOC_DL:    DX[7:0]  <= value[7:0];        // DL
        LOC_BL:    BX[7:0]  <= value[7:0];        // BL
        LOC_AH:    AX[15:8] <= value[7:0];        // AH
        LOC_CH:    CX[15:8] <= value[7:0];        // CH
        LOC_DH:    DX[15:8] <= value[7:0];        // DH
        LOC_BH:    BX[15:8] <= value[7:0];        // BH
        LOC_TMPA:  TMPA <= value;
        LOC_TMPB:  TMPB <= value;
        LOC_TMPC:  TMPC <= value;
        LOC_TMPAL: TMPA[7:0] <= value[7:0];
        LOC_TMPBL: TMPB <= {{8{value[7]}}, value[7:0]};    // sign-extend (needed by [i] @1de)
        LOC_TMPAH: TMPA[15:8] <= value[7:0];
        LOC_TMPBH: TMPB[15:8] <= value[7:0];
        // LOC_IND: // IND is updated separately
        // LOC_F:   // F is updated separately
        default: ;
        endcase
    end
endtask

// IND update logic
always @* begin
    IND_pre_update = IND;
    if (uc_d == LOC_IND || uc_d == LOC_M && M == LOC_IND || uc_d == LOC_N && N == LOC_IND) begin
        IND_pre_update = s_bus;
    end
    // Post update to IND
    IND_next = IND_pre_update;
    if (uc_typ == 3'd6) case (uc[1:0])
    2'b00: IND_next = IND_pre_update + 16'h2;  // P2
    2'b01: begin                               // BL
        if (DF) begin
            IND_next = L8_aux ? IND_pre_update - 16'h1 : IND_pre_update - 16'h2;  
        end else begin
            IND_next = L8_aux ? IND_pre_update + 16'h1 : IND_pre_update + 16'h2;  
        end
    end
    2'b10: IND_next = IND_pre_update - 16'h2;  // M2
    default: ;        // P0
    endcase
end


//----------------------------------------------------------------- Helper functions

// the group decode ROM: https://static.righto.com/8086/groupRom.html
function automatic [14:0] group_decode(input [7:0] ir);
    group_decode[0] = ir[7:4] == 4'hE & ir[2];                     // IN/OUT
    group_decode[1] = ir[7:6] == 2'h0 & ir[2] |
                      ir[7:6] == 2'h1 |
                      ir[7:4] == 4'h8 & (ir[3:1] == 3'h4 | ir[3:1] == 3'h6 | ir[3:0] == 4'hF) |
                      ir[7:4] == 4'h9 | ir[7:5] == 3'h5 | 
                      ir[7:4] == 4'hC & ir[3:1] == 2'h3 |
                      ir[7:4] == 4'hE & ir[2] |
                      ir[7:4] == 4'hF & ir[2:1] != 2'h3;           // mod/rm w/read
    group_decode[2] = ir[7:4] == 4'hF & ir[2:1] == 2'h3;           // group 3/4/5 opcode
    group_decode[3] = ir[7:5] == 3'h1 & ir[2:0] == 3'h6 | 
                      ir[7:4] == 4'hF & ir[3:2] == 2'h0;           // prefix
    group_decode[4] = ir[7:6] == 2'h0 & ir[2] |
                      ir[7:6] == 2'h1 |
                      ir[7:4] == 4'h9 | ir[7:5] == 3'h5 |
                      ir[7:4] == 4'hC & ir[3:2] != 2'h1 |
                      ir[7:4] == 4'hD & ir[3:2] == 2'h1 |
                      ir[7:4] == 4'hE |
                      ir[7:4] == 4'hF & ir[2:1] != 2'h3;           // 2BR (2 Byte Rom)
    group_decode[5] = ~(ir[7:6] == 2'h0 & (~ir[2] | ir[2:1] == 2'h2) | 
                      ir[7:4] == 4'h8 & ~ir[3] |
                      ir[7:4] == 4'hD & ir[3:2] == 2'h0);          // special ALU op
    group_decode[6] = ir[7:4] == 4'hF & ir[3] & ir[2:1] != 2'h3;   // cond-code
    group_decode[7] = ir[7:6] == 2'h0 & ir[2:1] == 2'h2 |
                      ir[7:4] == 4'hA |
                      ir[7:4] == 4'hE & ir[2];                     // AX/AL reg
    group_decode[8] = ir[7:4] == 4'h8 & ir[3:2] == 2'h3 & ~ir[0];  // seg reg bits
    group_decode[9] = ir[7:6] == 2'h0 & ~ir[2] |
                      ir[7:4] == 4'h8 & (ir[3:2] == 2'h2 | ir[3:2] == 2'h3 & ~ir[0]); // d bit
    group_decode[10] = ir[7:5] == 3'h1 & ir[2:0] == 3'h6 | 
                       ir[7:4] == 4'hF & ir[2:1] != 2'h3;          // 1BL (one byte logic)
    group_decode[11] = ir[7:6] == 2'h0 & ir[2:1] != 2'h3 |
                       ir[7:4] == 4'h8 & ir[3:2] != 2'h3 |
                       ir[7:4] == 4'hA |
                       ir[7:4] == 4'hC & ir[3:1] == 3'h3 |
                       ir[7:4] == 4'hD & ir[3:2] == 2'h0 |
                       ir[7:4] == 4'hE & ir[2] |
                       ir[7:4] == 4'hF & ir[2:1] == 2'h3;          // w bit
    group_decode[12] = ir[7:5] == 3'h1 & ir[2:0] == 3'h7 |
                       ir[7:4] == 4'hD & ir[3:2] == 2'h1;          // force-byte
    group_decode[13] = ir[7:4] == 4'h8 & ~ir[3] |
                       ir[7:4] == 4'hE & ~ir[2];                   // length from bit 1
    group_decode[14] = ~(ir[7:6] == 2'h1 | 
                       ir[7:4] == 4'hF & ir[3:1] == 3'h7);         // update carry
endfunction

// Pack the 13-bit micro-instruction address into 9 bits for the microcode ROM
function [8:0] uaddr_pack;
    input [12:0] uaddr;

    uaddr_pack[1:0] = uaddr[1:0];
    casez (uaddr[12:2])                               // See doc/microcode_8086.txt
    13'b0_1000_10??_00: uaddr_pack[8:2] = {5'h00, 2'b00};  // 0100010??.00  MOV rm<->r
    13'b0_1000_1101_00: uaddr_pack[8:2] = {5'h00, 2'b01};  // 010001101.00  LEA
    13'b0_00??_?0??_00: uaddr_pack[8:2] = {5'h00, 2'b10};  // 000???0??.00  alu rm<->r
    13'b0_1000_00??_00: uaddr_pack[8:2] = {5'h00, 2'b11};  // 0100000??.00  alu rm,i

    13'b0_?000_00??_01: uaddr_pack[8:2] = {5'h01, 2'b00};  // ?000000??.01
    13'b0_1100_011?_00: uaddr_pack[8:2] = {5'h01, 2'b01};  // 01100011?.00  MOV rm,i
    13'b0_00??_?10?_00: uaddr_pack[8:2] = {5'h01, 2'b10};  // 000???10?.00  alu A,i
    13'b0_1011_????_00: uaddr_pack[8:2] = {5'h01, 2'b11};  // 01011????.00  MOV r,i

    13'b?_1111_100?_00: uaddr_pack[8:2] = {5'h02, 2'b00};  // ?1111100?.00   INC/DEC rm
    13'b?_1111_111?_00: uaddr_pack[8:2] = {5'h02, 2'b01};  // ?1111111?.00   PUSH rm
    13'b0_0101_0???_00: uaddr_pack[8:2] = {5'h02, 2'b10};  // 001010???.00  PUSH rw
    13'b0_000?_?110_00: uaddr_pack[8:2] = {5'h02, 2'b11};  // 0000??110.00  PUSH sr

    13'b0_1001_1100_00: uaddr_pack[8:2] = {5'h03, 2'b00};  // 010011100.00  PUSHF
    13'b0_0101_1???_00: uaddr_pack[8:2] = {5'h03, 2'b01};  // 001011???.00  POP rw
    13'b0_000?_?111_00: uaddr_pack[8:2] = {5'h03, 2'b10};  // 0000??111.00  POP sr
    13'b0_1001_1101_00: uaddr_pack[8:2] = {5'h03, 2'b11};  // 010011101.00  POPF

    13'b0_1000_1111_00: uaddr_pack[8:2] = {5'h04, 2'b00};  // 010001111.00  POP rmw
    13'b0_1000_1111_01: uaddr_pack[8:2] = {5'h04, 2'b01};  // 010001111.01
    13'b?_1111_0010_00: uaddr_pack[8:2] = {5'h04, 2'b11};  // ?11110010.00   NOT rm

    13'b?_1111_0011_00: uaddr_pack[8:2] = {5'h05, 2'b00};  // ?11110011.00   NEG rm
    13'b0_1001_1000_00: uaddr_pack[8:2] = {5'h05, 2'b01};  // 010011000.00  CBW
    13'b0_1001_1001_00: uaddr_pack[8:2] = {5'h05, 2'b10};  // 010011001.00  CWD
    13'b0_1001_1001_01: uaddr_pack[8:2] = {5'h05, 2'b11};  // 010011001.01
    
    13'b0_1010_000?_00: uaddr_pack[8:2] = {5'h06, 2'b00};  // 01010000?.00  MOV A,[i]
    13'b0_1010_001?_00: uaddr_pack[8:2] = {5'h06, 2'b01};  // 01010001?.00  MOV [i],A
    13'b?_1111_1011_00: uaddr_pack[8:2] = {5'h06, 2'b10};  // ?11111011.00   CALL FAR rm
    13'b?_1111_1011_01: uaddr_pack[8:2] = {5'h06, 2'b11};  // ?11111011.01  FARCALL2
    
    13'b0_1001_1010_00: uaddr_pack[8:2] = {5'h07, 2'b00};  // 010011010.00  CALL cd
    13'b?_1111_1010_00: uaddr_pack[8:2] = {5'h07, 2'b01};  // ?11111010.00   CALL rm
    13'b?_1111_1010_01: uaddr_pack[8:2] = {5'h07, 2'b10};  // ?11111010.01
    13'b0_1110_1000_00: uaddr_pack[8:2] = {5'h07, 2'b11};  // 011101000.00  CALL cw

    13'b0_1110_1000_01: uaddr_pack[8:2] = {5'h08, 2'b00};  // 
    13'b0_1001_0???_00: uaddr_pack[8:2] = {5'h08, 2'b01};  // 010010???.00  XCHG AX,rw
    13'b0_1101_000?_00: uaddr_pack[8:2] = {5'h08, 2'b10};  // 01101000?.00  rot rm,1
    13'b0_1101_001?_00: uaddr_pack[8:2] = {5'h08, 2'b11};  // 01101001?.00  rot rm,CL

    13'b0_1101_001?_01: uaddr_pack[8:2] = {5'h09, 2'b00};  // 01101001?.01
    13'b0_1000_010?_00: uaddr_pack[8:2] = {5'h09, 2'b01};  // 01000010?.00  TEST rm,r
    13'b?_1111_000?_00: uaddr_pack[8:2] = {5'h09, 2'b10};  // ?1111000?.00   TEST rm,i
    13'b0_1010_100?_00: uaddr_pack[8:2] = {5'h09, 2'b11};  // 01010100?.00  TEST A,i

    13'b0_1101_0110_00: uaddr_pack[8:2] = {5'h0A, 2'b00};  // 011010110.00  SALC
    13'b0_1000_011?_00: uaddr_pack[8:2] = {5'h0A, 2'b01};  // 01000011?.00  XCHG rm,r
    13'b0_1000_011?_01: uaddr_pack[8:2] = {5'h0A, 2'b10};  // 01000011?.01
    13'b0_1110_010?_00: uaddr_pack[8:2] = {5'h0A, 2'b11};  // 01110010?.00  IN A,ib

    13'b0_1110_011?_00: uaddr_pack[8:2] = {5'h0B, 2'b00};  // 01110011?.00  OUT ib,A
    13'b0_1110_110?_00: uaddr_pack[8:2] = {5'h0B, 2'b01};  // 01110110?.00  IN A,DX
    13'b0_1110_111?_00: uaddr_pack[8:2] = {5'h0B, 2'b10};  // 01110111?.00  OUT DX,A
    13'b0_1100_00?1_00: uaddr_pack[8:2] = {5'h0B, 2'b11};  // 0110000?1.00  RET

    13'b0_1100_10?1_00: uaddr_pack[8:2] = {5'h0C, 2'b00};  // 0110010?1.00  RETF
    13'b0_1100_10?1_01: uaddr_pack[8:2] = {5'h0C, 2'b01};  // 0110010?1.01
    13'b0_1100_1111_00: uaddr_pack[8:2] = {5'h0C, 2'b10};  // 011001111.00  IRET
    13'b0_1100_?0?0_00: uaddr_pack[8:2] = {5'h0C, 2'b11};  // 01100?0?0.00  RET/RETF iw

    13'b0_1110_10?1_00: uaddr_pack[8:2] = {5'h0D, 2'b00};  // 0111010?1.00  JMP cw/JMP cb
    13'b0_1110_10?1_01: uaddr_pack[8:2] = {5'h0D, 2'b01};  // 0111010?1.01
    13'b?_1111_1100_00: uaddr_pack[8:2] = {5'h0D, 2'b10};  // ?11111100.00   JMP rm
    13'b?_1111_1101_00: uaddr_pack[8:2] = {5'h0D, 2'b11};  // ?11111101.00   JMP FAR rm

    13'b0_1110_1010_00: uaddr_pack[8:2] = {5'h0E, 2'b00};  // 011101010.00  JMP cd
    13'b0_1110_1010_01: uaddr_pack[8:2] = {5'h0E, 2'b01};  // 011101010.01
    13'b0_011?_????_00: uaddr_pack[8:2] = {5'h0E, 2'b10};  // 0011?????.00  Jcond cb
    13'b0_1000_11?0_00: uaddr_pack[8:2] = {5'h0E, 2'b11};  // 0100011?0.00  MOV rmw<->sr

    13'b0_1100_0100_00: uaddr_pack[8:2] = {5'h0F, 2'b00};  // 011000100.00  LES
    13'b0_1100_0101_00: uaddr_pack[8:2] = {5'h0F, 2'b01};  // 011000101.00  LDS
    13'b0_1001_1011_00: uaddr_pack[8:2] = {5'h0F, 2'b10};  // 010011011.00  WAIT
    13'b0_1001_1011_01: uaddr_pack[8:2] = {5'h0F, 2'b11};  // 010011011.01

    13'b0_1001_1110_00: uaddr_pack[8:2] = {5'h10, 2'b00};  // 010011110.00  SAHF
    13'b0_1001_1111_00: uaddr_pack[8:2] = {5'h10, 2'b01};  // 010011111.00  LAHF
    13'b0_1101_1???_00: uaddr_pack[8:2] = {5'h10, 2'b10};  // 011011???.00  ESC
    13'b0_1101_0111_00: uaddr_pack[8:2] = {5'h10, 2'b11};  // 011010111.00  XLAT

    13'b0_1101_0111_01: uaddr_pack[8:2] = {5'h11, 2'b00};  // 011010111.01
    13'b0_1101_0111_10: uaddr_pack[8:2] = {5'h11, 2'b01};  // 011010111.10
    13'b0_1101_0111_11: uaddr_pack[8:2] = {5'h11, 2'b10};  // 011010111.11  RPTI
    13'b0_1010_101?_00: uaddr_pack[8:2] = {5'h11, 2'b11};  // 01010101?.00  STOS

    13'b0_1010_?11?_00: uaddr_pack[8:2] = {5'h12, 2'b00};  // 01010?11?.00  CMPS/SCAS
    13'b0_1010_?11?_01: uaddr_pack[8:2] = {5'h12, 2'b01};  // 01010?11?.01
    13'b0_1010_?11?_10: uaddr_pack[8:2] = {5'h12, 2'b10};  // 01010?11?.10
    13'b0_1010_?10?_00: uaddr_pack[8:2] = {5'h12, 2'b11};  // 01010?10?.00  MOVS/LODS

    13'b0_1010_?10?_01: uaddr_pack[8:2] = {5'h13, 2'b00};  // 01010?10?.01
    13'b0_1110_0011_00: uaddr_pack[8:2] = {5'h13, 2'b01};  // 011100011.00  JCXZ
    13'b0_1110_000?_00: uaddr_pack[8:2] = {5'h13, 2'b10};  // 01110000?.00  LOOPNE/LOOPE
    13'b0_1110_000?_01: uaddr_pack[8:2] = {5'h13, 2'b11};  // 01110000?.01

    13'b0_1110_0010_00: uaddr_pack[8:2] = {5'h14, 2'b00};  // 011100010.00  LOOP
    13'b0_0010_?111_00: uaddr_pack[8:2] = {5'h14, 2'b01};  // 00010?111.00  DAA/DAS
    13'b0_0011_?111_00: uaddr_pack[8:2] = {5'h14, 2'b10};  // 00011?111.00  AAA/AAS
    13'b0_0011_?111_01: uaddr_pack[8:2] = {5'h14, 2'b11};  // 00011?111.01

    13'b0_1111_010?_00: uaddr_pack[8:2] = {5'h15, 2'b00};  // 01111010?.00   iMUL rmb
    13'b0_1111_010?_01: uaddr_pack[8:2] = {5'h15, 2'b01};  // 01111010?.01
    13'b1_1111_010?_00: uaddr_pack[8:2] = {5'h15, 2'b10};  // 11111010?.00   iMUL rmw
    13'b1_1111_010?_01: uaddr_pack[8:2] = {5'h15, 2'b11};  // 11111010?.01

    13'b0_1111_011?_00: uaddr_pack[8:2] = {5'h16, 2'b00};  // 01111011?.00   iDIV rmb
    13'b0_1111_011?_01: uaddr_pack[8:2] = {5'h16, 2'b01};  // 01111011?.01
    13'b1_1111_011?_00: uaddr_pack[8:2] = {5'h16, 2'b10};  // 11111011?.00   iDIV rmw
    13'b1_1111_011?_01: uaddr_pack[8:2] = {5'h16, 2'b11};  // 11111011?.01

    13'b0_1101_0101_00: uaddr_pack[8:2] = {5'h17, 2'b00};  // 011010101.00  AAD
    13'b0_1101_0100_00: uaddr_pack[8:2] = {5'h17, 2'b01};  // 011010100.00  AAM
    13'b0_1101_0100_01: uaddr_pack[8:2] = {5'h17, 2'b10};  // 011010100.01
    13'b0_0100_????_00: uaddr_pack[8:2] = {5'h17, 2'b11};  // 00100????.00  INC/DEC

    13'b0_0100_????_01: uaddr_pack[8:2] = {5'h18, 2'b00};  // 00100????.01
    13'b0_0100_????_10: uaddr_pack[8:2] = {5'h18, 2'b01};  // 00100????.10
    13'b1_0010_0010_00: uaddr_pack[8:2] = {5'h18, 2'b10};  // 100100010.00  CORD
    13'b1_0010_0010_01: uaddr_pack[8:2] = {5'h18, 2'b11};  // 100100010.01

    13'b1_0010_0010_10: uaddr_pack[8:2] = {5'h19, 2'b00};  // 100100010.10
    13'b1_0010_0010_11: uaddr_pack[8:2] = {5'h19, 2'b01};  // 100100010.11
    13'b1_0000_0000_00: uaddr_pack[8:2] = {5'h19, 2'b10};  // 100000000.00  INT1
    13'b1_0000_0000_01: uaddr_pack[8:2] = {5'h19, 2'b11};  // 100000000.01

    13'b1_0000_0000_10: uaddr_pack[8:2] = {5'h1A, 2'b00};  // 100000000.10
    13'b1_0000_0000_11: uaddr_pack[8:2] = {5'h1A, 2'b01};  // 100000000.11
    13'b0_1100_1101_00: uaddr_pack[8:2] = {5'h1A, 2'b10};  // 011001101.00  INT ib
    13'b0_1100_1110_00: uaddr_pack[8:2] = {5'h1A, 2'b11};  // 011001110.00  INTO

    13'b0_1100_1100_00: uaddr_pack[8:2] = {5'h1B, 2'b00};  // 011001100.00  INT 3
    13'b1_0010_0011_00: uaddr_pack[8:2] = {5'h1B, 2'b01};  // 100100011.00  PREIDIV
    13'b1_0010_0011_01: uaddr_pack[8:2] = {5'h1B, 2'b10};  // 100100011.01
    13'b1_0010_0011_10: uaddr_pack[8:2] = {5'h1B, 2'b11};  // 100100011.10

    13'b1_0010_0011_11: uaddr_pack[8:2] = {5'h1C, 2'b00};  // 100100011.11  PREIMUL
    13'b1_0010_0100_00: uaddr_pack[8:2] = {5'h1C, 2'b01};  // 100100100.00  POSTIDIV
    13'b1_0010_0100_01: uaddr_pack[8:2] = {5'h1C, 2'b10};  // 100100100.01
    13'b1_0010_0100_10: uaddr_pack[8:2] = {5'h1C, 2'b11};  // 100100100.10

    13'b1_0010_0100_11: uaddr_pack[8:2] = {5'h1D, 2'b00};  // 100100100.11
    13'b1_0100_0000_00: uaddr_pack[8:2] = {5'h1D, 2'b01};  // 101000000.00  [BX+SI]
    13'b1_0100_0000_01: uaddr_pack[8:2] = {5'h1D, 2'b10};  // 101000000.01
    13'b1_0100_0000_10: uaddr_pack[8:2] = {5'h1D, 2'b11};  // 101000000.10  [iw]

    13'b1_0100_0000_11: uaddr_pack[8:2] = {5'h1E, 2'b00};  // 101000000.11
    13'b1_1000_0000_00: uaddr_pack[8:2] = {5'h1E, 2'b01};  // 110000000.00  RESET
    13'b1_1000_0000_01: uaddr_pack[8:2] = {5'h1E, 2'b10};  // 110000000.01
    13'b0_1001_1011_10: uaddr_pack[8:2] = {5'h1E, 2'b11};  // 010011011.10  WAIT continued

    13'b0_1010_101?_01: uaddr_pack[8:2] = {5'h1F, 2'b00};  // 01010101?.01  STOS continued
    13'b0_1010_?11?_11: uaddr_pack[8:2] = {5'h1F, 2'b01};  // 01010?11?.11  CMPS/SCAS continued
    13'b0_1010_?10?_10: uaddr_pack[8:2] = {5'h1F, 2'b10};  // 01010?10?.10  MOVS/LODS continued
    13'b0_1110_0011_01: uaddr_pack[8:2] = {5'h1F, 2'b11};  // 011100011.01  JCXZ continued

    default: uaddr_pack ={9{1'bx}};                   // Let the synthesizer optimize
    endcase
endfunction

// Translation ROM: doc/translation_8086.txt
// mode_rm: for memory addressing, {mode[1:0], rm[2:0]}
//          for jumps and calls,   {is_call, dest[3:0]}
// is_mem:  1 for memory addressing, 0 for jumps and calls
// disp_not: this instruction has no displacement
// mem_read_not: this instruction does not read memory
// return {AR, CR}
function automatic [12:0] translate(input is_mem, input [4:0] mode_rm, input disp_not, input mem_read_not);
    casez ({mode_rm, mem_read_not, is_mem, disp_not})
    // Memory addressing: {mode[1:0], rm[2:0], ?, is_mem=1, ?}
    8'b??000?1?: return 13'b101000000_0000;// 0x1d4 [BX+SI]
    8'b??001?1?: return 13'b101000000_0110;// 0x1da [BX+DI]
    8'b??010?1?: return 13'b101000000_0111;// 0x1db [BP+SI]
    8'b??011?1?: return 13'b101000000_0011;// 0x1d7 [BP+DI]
    8'b??100?1?: return 13'b010001011_0011;// 0x003 [SI]
    8'b??101?1?: return 13'b010111111_0011;// 0x01f [DI]
    8'b00110?1?: return 13'b101000000_1000;// 0x1dc [iw]
    8'b01110?1?: return 13'b111111001_0011;// 0x023 [BP]
    8'b10110?1?: return 13'b111111001_0011;// 0x023 [BP]
    8'b??111?1?: return 13'b001011111_0011;// 0x037 [BX]

    // Far jump destinations: {is_call=0, dest[3:0], disp_needed, is_mem=0, mem_read}
    8'b00000?0?: return 13'b111111011_0011;// 0x06b 0 FARCALL
    8'b00001?0?: return 13'b111111010_0011;// 0x077 1 NEARCALL
    8'b00010?0?: return 13'b011101011_0010;// 0x0d2 2 RELJMP
    //   begin EAOFFSET entries:
    8'b00011?00: return 13'b101000000_1010;// 0x1de 3 [i]
    8'b00011001: return 13'b101000000_1101;// 0x1e1 3 EALOAD
    8'b00011101: return 13'b101000000_1111;// 0x1e3 3 EADONE
    //   end EAOFFSET entries
    //   begin UNC EADONE
    8'b0010000?: return 13'b101000000_1101;// 0x1e1 4 EALOAD  
    8'b0010010?: return 13'b101000000_1111;// 0x1e3 4 EADONE
    //   end   UNC EADONE
    8'b00101?0?: return 13'b111111011_0100;// 0x06c 5 FARCALL2
    8'b00110?0?: return 13'b100000000_0101;// 0x19d 6 INTR
    8'b00111?0?: return 13'b100000000_1111;// 0x1a7 7 INT0
    8'b01000?0?: return 13'b011010111_1100;// 0x118 8 RPTI
    8'b01001?0?: return 13'b011010100_0101;// 0x179 9 AAEND

    // Far call destinations: {is_call=1, dest[3:0], ?, is_mem=0, ?}
    8'b10000?0?: return 13'b011001011_0010;// 0x0c2 0 FARRET
    8'b10001?0?: return 13'b011010111_0110;// 0x112 1 RPTS
    8'b10010?0?: return 13'b001001111_0011;// 0x17f 2 CORX
    8'b10011?0?: return 13'b100100010_0000;// 0x188 3 CORD
    8'b10100?0?: return 13'b100100011_1100;// 0x1c0 4 PREIMUL
    8'b10101?0?: return 13'b100100011_0010;// 0x1b6 5 NEGATE
    8'b10110?0?: return 13'b100100100_1001;// 0x1cd 6 IMULCOF
    8'b10111?0?: return 13'b100100100_1110;// 0x1d2 7 MULCOF
    8'b11000?0?: return 13'b100100011_0000;// 0x1b4 8 PREIDIV
    8'b11001?0?: return 13'b100100100_0000;// 0x1c4 9 POSTIDIV

    // 8'b??????0?: return 13'b111111111_1111;
    // 8'b??????0?: return 13'b111111111_1111;
    default: return 13'b0;
    endcase
endfunction

// Map “low 3 bits” + width into the 5-bit location code
function automatic [4:0] gpr_bus_code(input [2:0] rrr, input w, input axal);
    begin
        // This matches the S/D encoding
        // 11000 = AX, 01000 = AL, 10000 = AH, etc.
        if (axal) 
            gpr_bus_code = {w, 4'b1000};
        else if (w)  // word registers: AX..DI
            gpr_bus_code = {2'b11, rrr};
        else    // byte registers: AL..BH
            gpr_bus_code = {rrr[2] ? 3'b100 : 3'b010, rrr[1:0]};
    end
endfunction 

`ifdef VERILATOR
initial begin
    integer __flags_hi_opt;
    if ($value$plusargs("flags_hi_f=%d", __flags_hi_opt)) begin
        popf_hi_is_f = (__flags_hi_opt != 0);
    end
end
`endif

endmodule
