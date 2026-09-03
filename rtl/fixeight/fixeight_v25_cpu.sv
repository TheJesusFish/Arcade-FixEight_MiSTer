// SPDX-License-Identifier: BSD-3-Clause
//
// FixEight NEC V25 integration, adapted from the audited Dogyuun wrapper.
// The instruction engine is the local
// opcode-decrypting compact core; this wrapper supplies the V25 register banks,
// exercised extended opcodes, internal registers, and periodic timer.

`timescale 1ns/1ns

module fixeight_v25_cpu #(
    parameter [7:0] SS_IDX = 8'd11,
    parameter integer BUS_PHASES_USE_CEN = 1,
    parameter integer BUS_BYTE_PHASES = 4,
    parameter integer SOUND_WRITE_EXTRA_PHASES = 17,
    parameter integer INSTRUCTION_GAP_PHASES = 0,
    parameter integer REGISTER_INC_GAP_PHASES = 0,
    parameter integer OPCODE_TIMING_PROFILE = 0,
    parameter integer TEST1_IMMEDIATE_DELAY = 19,
    parameter integer TEST1_MEMORY_DELAY = 0
) (
    input             clk,
    input             reset,
    input             reset_n,
    input             clock_enable,

    input      [7:0]  port0_in,
    input      [7:0]  port1_in,
    input      [7:0]  portt_in,
    output     [7:0]  port0_out,

    output     [19:0] bus_addr,
    output     [7:0]  bus_dout,
    input      [7:0]  bus_din,
    output            bus_doe,
    output            bus_r_w,
    output            bus_mreq_n,
    output            bus_mstb_n,
    output            bus_iostb_n,

    output            halted,
    output            fault,
    output     [19:0] debug_pc,
    output            state_idle,

    input             ss_restore_enable,
    input             ss_restore_commit,
    input      [63:0] ss_data,
    input      [31:0] ss_addr,
    input      [7:0]  ss_select,
    input             ss_write,
    input             ss_read,
    input             ss_query,
    output reg [63:0] ss_data_out,
    output reg        ss_ack
);

localparam [31:0] SS_WORD_COUNT = 32'd39;

localparam [1:0]
    SS_PORT_IDLE         = 2'd0,
    SS_PORT_READ_RAM     = 2'd1,
    SS_PORT_WRITE_RAM    = 2'd2,
    SS_PORT_WAIT_RELEASE = 2'd3;

localparam [4:0]
    ESC_IDLE           = 5'd0,
    ESC_OPCODE         = 5'd1,
    ESC_MODRM          = 5'd2,
    ESC_DISP8          = 5'd3,
    ESC_DISP_LO        = 5'd4,
    ESC_DISP_HI        = 5'd5,
    ESC_IMMEDIATE      = 5'd6,
    ESC_PREPARE        = 5'd7,
    ESC_MEM_READ       = 5'd8,
    ESC_MEM_GAP        = 5'd9,
    ESC_MEM_WRITE      = 5'd10,
    ESC_FLAGS_COMMIT   = 5'd11,
    ESC_DONE_COMMIT    = 5'd12,
    ESC_LOAD_COMMIT    = 5'd13,
    ESC_FAULT_COMMIT   = 5'd14,
    ESC_PUSH_WRITE     = 5'd15;

localparam [3:0]
    BUS_IDLE           = 4'd0,
    BUS_INTERNAL       = 4'd1,
    BUS_EXT_WAIT1      = 4'd2,
    BUS_EXT_WAIT2      = 4'd3,
    BUS_EXT_GAP        = 4'd4,
    BUS_ACK_CPU        = 4'd5,
    BUS_DROP_CPU       = 4'd6,
    BUS_ACK_EXT        = 4'd7,
    BUS_DROP_EXT       = 4'd8,
    BUS_EXT_WAIT3      = 4'd9;

localparam [2:0]
    LOAD_PREP_NONE     = 3'd0,
    LOAD_PREP_REGISTER = 3'd1,
    LOAD_PREP_BANK     = 3'd2,
    LOAD_PREP_PUSH     = 3'd3,
    LOAD_PREP_STACK    = 3'd4;

reg [7:0] internal_ram [0:255];
reg [2:0] active_bank;

// Bank switches capture a complete register-bank transaction here, then
// commit it on the following master clock. This keeps the opcode decoder and
// compact CPU state off the distributed internal-RAM write path.
reg        bank_commit_pending;
reg        bank_commit_write_return;
reg [2:0]  bank_commit_source;
reg [2:0]  bank_commit_target;
reg [15:0] bank_commit_psw;
reg [15:0] bank_commit_ip;

// Every custom architectural load is prepared from this local snapshot on a
// later master edge. The compact CPU remains held in ESC_LOAD_COMMIT until
// the transaction retires, so this pipeline does not consume a V25 clock.
reg [15:0] arch_stage_ds;
reg [15:0] arch_stage_ss;
reg [15:0] arch_stage_cs;
reg [15:0] arch_stage_es;
reg [15:0] arch_stage_di;
reg [15:0] arch_stage_si;
reg [15:0] arch_stage_bp;
reg [15:0] arch_stage_sp;
reg [15:0] arch_stage_bx;
reg [15:0] arch_stage_dx;
reg [15:0] arch_stage_cx;
reg [15:0] arch_stage_ax;
reg        load_prepare_pending;
reg [2:0]  load_prepare_mode;
reg [2:0]  load_prepare_bank;
reg        load_prepare_target_was_active;
reg        load_prepare_word;
reg [2:0]  load_prepare_register;
reg [15:0] load_prepare_value;
reg [15:0] load_prepare_ip;
reg [15:0] load_prepare_flags;

reg [7:0] idb_reg;
reg       ramen;
reg [7:0] prc_reg;
reg [7:0] p0_latch;
reg [7:0] p1_latch;
reg [7:0] p2_latch;
reg [7:0] pm0_reg;
reg [7:0] pm1_reg;
reg [7:0] pm2_reg;
reg [7:0] pmt_reg;
reg [7:0] rfm_reg;
reg [7:0] wtc_lo;
reg [7:0] wtc_hi;
reg [15:0] md1_reg;
reg [7:0] tmc0_reg;
reg [7:0] tmc1_reg;
reg [7:0] tmic0_reg;
reg [7:0] tmic1_reg;
reg [7:0] tmic2_reg;
reg [7:0] ispr_reg;
reg [31:0] timer_count;
reg        timer2_pending;
reg        irq_entry_pending;

wire [19:0] cpu_addr;
wire [15:0] cpu_dout;
reg  [15:0] cpu_din;
wire        cpu_wr;
wire        cpu_rd;
wire        cpu_io;
wire        cpu_word;
wire        cpu_data_cycle;
wire        cpu_ready;

assign port0_out = p0_latch;

wire [15:0] arch_ax;
wire [15:0] arch_cx;
wire [15:0] arch_dx;
wire [15:0] arch_bx;
wire [15:0] arch_sp;
wire [15:0] arch_bp;
wire [15:0] arch_si;
wire [15:0] arch_di;
wire [15:0] arch_cs;
wire [15:0] arch_ds;
wire [15:0] arch_es;
wire [15:0] arch_ss;
wire [15:0] arch_next_ip;
wire [15:0] arch_flags;
wire        arch_halted;
wire        arch_segment_override;
wire [15:0] arch_override_segment;
wire        instruction_done;
wire [7:0]  instruction_opcode;
wire        instruction_idle;
wire        interrupt_deferred;

wire        escape_active;
wire [7:0]  escape_lead;
wire [7:0]  escape_byte;
wire        escape_byte_valid;
wire        escape_pop;
wire        escape_complete;

reg  [4:0]  escape_state;
reg  [4:0]  escape_delay_count;
reg  [4:0]  instruction_gap_count;
reg         escape_timing_profile;
reg  [7:0]  escape_opcode;
reg  [7:0]  escape_modrm;
reg  [15:0] escape_disp;
reg  [7:0]  escape_imm;
reg  [3:0]  escape_bit;
reg  [15:0] escape_mem_value;
reg  [15:0] commit_flags;
reg         fault_latch;

reg  [15:0] load_ax;
reg  [15:0] load_cx;
reg  [15:0] load_dx;
reg  [15:0] load_bx;
reg  [15:0] load_sp;
reg  [15:0] load_bp;
reg  [15:0] load_si;
reg  [15:0] load_di;
reg  [15:0] load_cs;
reg  [15:0] load_ds;
reg  [15:0] load_es;
reg  [15:0] load_ss;
reg  [15:0] load_ip;
reg  [15:0] load_flags;

reg  [15:0] ss_load_ax;
reg  [15:0] ss_load_cx;
reg  [15:0] ss_load_dx;
reg  [15:0] ss_load_bx;
reg  [15:0] ss_load_sp;
reg  [15:0] ss_load_bp;
reg  [15:0] ss_load_si;
reg  [15:0] ss_load_di;
reg  [15:0] ss_load_cs;
reg  [15:0] ss_load_ds;
reg  [15:0] ss_load_es;
reg  [15:0] ss_load_ss;
reg  [15:0] ss_load_ip;
reg  [15:0] ss_load_flags;
reg         ss_load_halted;
reg  [1:0]  ss_port_state;
reg  [7:0]  ss_ram_index;
reg  [2:0]  ss_byte_index;
reg  [63:0] ss_ram_buffer;
reg  [63:0] ss_ram_write_data;

reg         ext_req;
reg         ext_write;
reg         ext_word;
reg  [19:0] ext_addr;
reg  [15:0] ext_dout;
wire [15:0] ext_din;
wire        ext_ready;

reg  [3:0]  bus_state;
reg  [4:0]  bus_wait_phase;
reg         txn_ext;
reg         txn_write;
reg         txn_word;
reg         txn_io;
reg         txn_internal;
reg         txn_phase;
reg  [19:0] txn_addr;
reg  [15:0] txn_dout;
reg  [15:0] txn_din;

wire escape_timing_done = escape_delay_count == 5'd0;
wire base_arch_load = escape_timing_done &&
                      escape_state == ESC_LOAD_COMMIT &&
                      !load_prepare_pending;
// The parent save-state controller emits a one-clock commit pulse. Register
// and stretch it locally so the wide architectural restore bus is stable
// across several capture edges instead of depending on one high-fanout edge.
reg       ss_restore_load_pending = 1'b0;
reg [2:0] ss_restore_load_count = 3'd0;
always @(posedge clk) begin
    if (reset) begin
        ss_restore_load_pending <= 1'b0;
        ss_restore_load_count <= 3'd0;
    end else if (ss_restore_commit && ss_restore_enable) begin
        ss_restore_load_pending <= 1'b1;
        ss_restore_load_count <= 3'd0;
    end else if (!reset_n) begin
        ss_restore_load_count <= 3'd0;
    end else if (ss_restore_load_pending) begin
        ss_restore_load_pending <= 1'b0;
        ss_restore_load_count <= 3'd4;
    end else if (ss_restore_load_count != 3'd0) begin
        ss_restore_load_count <= ss_restore_load_count - 3'd1;
    end
end
wire ss_restore_load = ss_restore_load_count != 3'd0;
wire cpu_arch_load = base_arch_load || ss_restore_load;
wire cpu_clock_enable = clock_enable || ss_restore_load;
wire bus_phase_advance = !BUS_PHASES_USE_CEN || clock_enable;
wire txn_sound_write = txn_write &&
                       (txn_addr == 20'h00000 ||
                        txn_addr == 20'h00001 ||
                        txn_addr == 20'h00004);
wire [5:0] txn_byte_phases = BUS_BYTE_PHASES +
                             (txn_sound_write ?
                              SOUND_WRITE_EXTRA_PHASES : 0);
wire base_flags_write = escape_timing_done &&
                        escape_state == ESC_FLAGS_COMMIT;
wire timer2_irq_request = timer2_pending && !tmic2_reg[6] &&
                          arch_flags[9];
wire base_fetch_hold = escape_state == ESC_LOAD_COMMIT;
wire interrupt_start_hold = timer2_irq_request &&
                            !interrupt_deferred;
wire instruction_is_register_inc =
    instruction_opcode >= 8'h40 && instruction_opcode <= 8'h47;
wire [19:0] instruction_next_pc =
    {arch_cs, 4'b0000} + {4'b0000, arch_next_ip};
wire instruction_timing_profile_active =
    OPCODE_TIMING_PROFILE != 0 &&
    instruction_next_pc >= 20'ha4000 &&
    instruction_next_pc < 20'ha6000;

function automatic [4:0] opcode_timing_gap(input [7:0] opcode);
begin
    opcode_timing_gap = 5'd0;
    if (instruction_timing_profile_active) begin
        case (opcode)
            8'h0a: opcode_timing_gap = 5'd6;
            8'h0c: opcode_timing_gap = 5'd13;
            8'h24: opcode_timing_gap = 5'd12;
            8'h32: opcode_timing_gap = 5'd8;
            8'h3c: opcode_timing_gap = 5'd13;
            8'h43: opcode_timing_gap = 5'd2;
            8'h50: opcode_timing_gap = 5'd16;
            8'h51: opcode_timing_gap = 5'd13;
            8'h55: opcode_timing_gap = 5'd13;
            8'h58: opcode_timing_gap = 5'd13;
            8'h59: opcode_timing_gap = 5'd15;
            8'h5d: opcode_timing_gap = 5'd8;
            8'h88: opcode_timing_gap = 5'd4;
            8'h8b: opcode_timing_gap = 5'd3;
            8'hb1: opcode_timing_gap = 5'd11;
            8'hb5: opcode_timing_gap = 5'd13;
            8'hbb: opcode_timing_gap = 5'd16;
            8'hbf: opcode_timing_gap = 5'd11;
            8'hc3: opcode_timing_gap = 5'd11;
            8'hd0: opcode_timing_gap = 5'd6;
            8'hd1: opcode_timing_gap = 5'd10;
            8'he8: opcode_timing_gap = 5'd7;
            8'hfe: opcode_timing_gap = 5'd1;
            default: opcode_timing_gap = 5'd0;
        endcase
    end
end
endfunction

wire [4:0] instruction_gap_load =
    INSTRUCTION_GAP_PHASES +
    (instruction_is_register_inc ? REGISTER_INC_GAP_PHASES : 0) +
    opcode_timing_gap(instruction_opcode);
wire instruction_gap_start =
    instruction_done && instruction_gap_load != 5'd0;
wire base_instruction_start_hold =
    interrupt_start_hold || instruction_gap_start ||
    instruction_gap_count != 5'd0;
wire irq_entry_safe = instruction_idle && bus_state == BUS_IDLE &&
                      !cpu_rd && !cpu_wr && !ext_req &&
                      !interrupt_deferred;
wire ss_selected = ss_select == SS_IDX;
wire ss_state_write = ss_selected && ss_write;
wire ss_ram_write_active = ss_port_state == SS_PORT_WRITE_RAM;
assign escape_complete = escape_timing_done &&
                         (escape_state == ESC_FLAGS_COMMIT ||
                          escape_state == ESC_DONE_COMMIT ||
                          escape_state == ESC_FAULT_COMMIT);
assign escape_pop = escape_byte_valid &&
                    (escape_state == ESC_OPCODE ||
                     escape_state == ESC_MODRM ||
                     escape_state == ESC_DISP8 ||
                     escape_state == ESC_DISP_LO ||
                     escape_state == ESC_DISP_HI ||
                     escape_state == ESC_IMMEDIATE);

assign halted = arch_halted;
assign fault = fault_latch;
assign debug_pc = instruction_next_pc;
assign state_idle = reset_n &&
                    instruction_idle &&
                    (bus_state == BUS_IDLE) &&
                    !cpu_rd &&
                    !cpu_wr &&
                    !ext_req &&
                    (escape_state == ESC_IDLE) &&
                    !escape_active &&
                    !arch_segment_override &&
                    !interrupt_deferred &&
                    !timer2_irq_request &&
                    !irq_entry_pending &&
                    !bank_commit_pending &&
                    !load_prepare_pending;

wire bus_external_active = bus_state == BUS_EXT_WAIT1;
wire [19:0] bus_byte_addr = txn_addr + (txn_phase ? 20'd1 : 20'd0);
assign bus_addr = bus_byte_addr;
assign bus_dout = txn_phase ? txn_dout[15:8] : txn_dout[7:0];
assign bus_doe = bus_external_active && txn_write;
assign bus_r_w = ~txn_write;
assign bus_mreq_n = ~(bus_external_active && !txn_io);
assign bus_mstb_n = ~bus_external_active;
assign bus_iostb_n = ~(bus_external_active && txn_io);
assign ext_din = txn_din;
assign ext_ready = bus_state == BUS_ACK_EXT;
assign cpu_ready = bus_state == BUS_ACK_CPU;

wire [8:0] int_index0 = txn_addr[8:0];
wire [8:0] int_index1 = txn_addr[8:0] + 9'd1;
wire int_write0 = bus_state == BUS_INTERNAL && txn_write;
wire int_write1 = int_write0 && txn_word;

function automatic [15:0] stored_bank_word;
    input [2:0] bank;
    input [4:0] offset;
    begin
        stored_bank_word = {
            internal_ram[{bank, 5'b00000} + offset + 5'd1],
            internal_ram[{bank, 5'b00000} + offset]
        };
    end
endfunction

function automatic [15:0] live_bank_word;
    input [2:0] bank;
    input [4:0] offset;
    begin
        if (bank == active_bank) begin
            case (offset)
                5'h08: live_bank_word = arch_ds;
                5'h0a: live_bank_word = arch_ss;
                5'h0c: live_bank_word = arch_cs;
                5'h0e: live_bank_word = arch_es;
                5'h10: live_bank_word = arch_di;
                5'h12: live_bank_word = arch_si;
                5'h14: live_bank_word = arch_bp;
                5'h16: live_bank_word = arch_sp;
                5'h18: live_bank_word = arch_bx;
                5'h1a: live_bank_word = arch_dx;
                5'h1c: live_bank_word = arch_cx;
                5'h1e: live_bank_word = arch_ax;
                default: live_bank_word = stored_bank_word(bank, offset);
            endcase
        end else begin
            live_bank_word = stored_bank_word(bank, offset);
        end
    end
endfunction

function automatic [7:0] internal_read_byte;
    input [8:0] index;
    reg [2:0] bank;
    reg [4:0] offset;
    reg [15:0] word_value;
    begin
        if (!index[8]) begin
            bank = index[7:5];
            offset = {index[4:1], 1'b0};
            word_value = live_bank_word(bank, offset);
            internal_read_byte = index[0] ? word_value[15:8] : word_value[7:0];
        end else begin
            case (index)
                9'h100: internal_read_byte = (p0_latch & ~pm0_reg) |
                                                   (port0_in & pm0_reg);
                9'h108: internal_read_byte = (p1_latch & ~pm1_reg) |
                                                   (port1_in & pm1_reg);
                9'h110: internal_read_byte = p2_latch & ~pm2_reg;
                9'h138: internal_read_byte = portt_in;
                9'h180: internal_read_byte = 8'hff;
                9'h181: internal_read_byte = 8'hff;
                9'h18a: internal_read_byte = md1_reg[7:0];
                9'h18b: internal_read_byte = md1_reg[15:8];
                9'h190: internal_read_byte = tmc0_reg;
                9'h191: internal_read_byte = tmc1_reg;
                9'h19c: internal_read_byte = tmic0_reg;
                9'h19d: internal_read_byte = tmic1_reg;
                9'h19e: internal_read_byte = {timer2_pending, tmic2_reg[6:0]};
                9'h1e1: internal_read_byte = rfm_reg;
                9'h1e8: internal_read_byte = wtc_lo;
                9'h1e9: internal_read_byte = wtc_hi;
                9'h1eb: internal_read_byte = prc_reg;
                9'h1fc: internal_read_byte = ispr_reg;
                9'h1ff: internal_read_byte = idb_reg;
                default: internal_read_byte = 8'h00;
            endcase
        end
    end
endfunction

function automatic internal_selected;
    input [19:0] address;
    input        data_access;
    begin
        internal_selected = data_access &&
            ((((address & 20'hffe00) == {idb_reg, 12'he00}) &&
              (ramen || address[8])) || address == 20'hfffff);
    end
endfunction

function automatic [31:0] timer_period;
    input [15:0] md_value;
    input [7:0]  control;
    input [7:0]  prc_value;
    reg [31:0] md_extended;
    reg [31:0] md_times_3;
    begin
        md_extended = {16'd0, md_value};
        md_times_3 = md_extended + (md_extended << 1);
        if (control[6]) begin
            case (prc_value[1:0])
                2'b00: timer_period = md_extended << 8;
                2'b01: timer_period = md_extended << 9;
                default: timer_period = md_extended << 10;
            endcase
        end else begin
            case (prc_value[1:0])
                2'b00: timer_period = md_times_3 << 2;
                2'b01: timer_period = md_times_3 << 3;
                default: timer_period = md_times_3 << 4;
            endcase
        end
    end
endfunction

function automatic [15:0] packed_psw;
    input [15:0] flags_value;
    input [2:0]  bank;
    begin
        packed_psw = {1'b1, bank, flags_value[11:2], 1'b1, flags_value[0]};
    end
endfunction

function automatic [15:0] gpr16_read;
    input [2:0] code;
    begin
        case (code)
            3'd0: gpr16_read = arch_ax;
            3'd1: gpr16_read = arch_cx;
            3'd2: gpr16_read = arch_dx;
            3'd3: gpr16_read = arch_bx;
            3'd4: gpr16_read = arch_sp;
            3'd5: gpr16_read = arch_bp;
            3'd6: gpr16_read = arch_si;
            default: gpr16_read = arch_di;
        endcase
    end
endfunction

function automatic [7:0] gpr8_read;
    input [2:0] code;
    begin
        case (code)
            3'd0: gpr8_read = arch_ax[7:0];
            3'd1: gpr8_read = arch_cx[7:0];
            3'd2: gpr8_read = arch_dx[7:0];
            3'd3: gpr8_read = arch_bx[7:0];
            3'd4: gpr8_read = arch_ax[15:8];
            3'd5: gpr8_read = arch_cx[15:8];
            3'd6: gpr8_read = arch_dx[15:8];
            default: gpr8_read = arch_bx[15:8];
        endcase
    end
endfunction

function automatic [15:0] ea_base;
    input [2:0] code;
    begin
        case (code)
            3'd0: ea_base = arch_bx + arch_si;
            3'd1: ea_base = arch_bx + arch_di;
            3'd2: ea_base = arch_bp + arch_si;
            3'd3: ea_base = arch_bp + arch_di;
            3'd4: ea_base = arch_si;
            3'd5: ea_base = arch_di;
            3'd6: ea_base = escape_modrm[7:6] == 2'b00 ? 16'h0000 : arch_bp;
            default: ea_base = arch_bx;
        endcase
    end
endfunction

function automatic ea_defaults_ss;
    input [2:0] code;
    begin
        ea_defaults_ss = code == 3'd2 || code == 3'd3 ||
                         (code == 3'd6 && escape_modrm[7:6] != 2'b00);
    end
endfunction

wire escape_is_word = escape_opcode[0];
wire escape_test = escape_opcode == 8'h10 || escape_opcode == 8'h11 ||
                   escape_opcode == 8'h18 || escape_opcode == 8'h19;
wire escape_clear = escape_opcode == 8'h12 || escape_opcode == 8'h13 ||
                    escape_opcode == 8'h1a || escape_opcode == 8'h1b;
wire escape_set = escape_opcode == 8'h14 || escape_opcode == 8'h15 ||
                  escape_opcode == 8'h1c || escape_opcode == 8'h1d;
wire escape_toggle = escape_opcode == 8'h16 || escape_opcode == 8'h17 ||
                     escape_opcode == 8'h1e || escape_opcode == 8'h1f;
wire [15:0] escape_register_value = escape_is_word ?
                                      gpr16_read(escape_modrm[2:0]) :
                                      {8'h00, gpr8_read(escape_modrm[2:0])};
wire [15:0] escape_bit_mask = 16'h0001 << escape_bit;

task automatic store_word;
    input [2:0] bank;
    input [4:0] offset;
    input [15:0] value;
    begin
        internal_ram[{bank, 5'b00000} + offset] <= value[7:0];
        internal_ram[{bank, 5'b00000} + offset + 5'd1] <= value[15:8];
    end
endtask

task automatic capture_arch_stage;
    begin
        arch_stage_ds <= arch_ds;
        arch_stage_ss <= arch_ss;
        arch_stage_cs <= arch_cs;
        arch_stage_es <= arch_es;
        arch_stage_di <= arch_di;
        arch_stage_si <= arch_si;
        arch_stage_bp <= arch_bp;
        arch_stage_sp <= arch_sp;
        arch_stage_bx <= arch_bx;
        arch_stage_dx <= arch_dx;
        arch_stage_cx <= arch_cx;
        arch_stage_ax <= arch_ax;
    end
endtask

task automatic stage_bank_switch;
    input        write_return;
    input [2:0]  return_bank;
    input [15:0] return_psw;
    input [15:0] return_ip_value;
    input [15:0] next_ip;
    input [15:0] next_flags;
    begin
        capture_arch_stage();
        bank_commit_pending <= 1'b1;
        bank_commit_write_return <= write_return;
        bank_commit_source <= active_bank;
        bank_commit_target <= return_bank;
        bank_commit_psw <= return_psw;
        bank_commit_ip <= return_ip_value;
        load_prepare_pending <= 1'b1;
        load_prepare_mode <= LOAD_PREP_BANK;
        load_prepare_bank <= return_bank;
        load_prepare_target_was_active <= return_bank == active_bank;
        load_prepare_ip <= next_ip;
        load_prepare_flags <= next_flags;
    end
endtask

task automatic stage_register_load;
    input [15:0] value;
    input [15:0] next_flags;
    begin
        capture_arch_stage();
        load_prepare_pending <= 1'b1;
        load_prepare_mode <= LOAD_PREP_REGISTER;
        load_prepare_word <= escape_is_word;
        load_prepare_register <= escape_modrm[2:0];
        load_prepare_value <= value;
        load_prepare_ip <= arch_next_ip;
        load_prepare_flags <= next_flags;
    end
endtask

task automatic stage_push_load;
    input [15:0] next_sp;
    begin
        capture_arch_stage();
        load_prepare_pending <= 1'b1;
        load_prepare_mode <= LOAD_PREP_PUSH;
        load_prepare_value <= next_sp;
        load_prepare_ip <= arch_next_ip;
        load_prepare_flags <= arch_flags;
    end
endtask

task automatic stage_stack_load;
    input [2:0] bank;
    begin
        capture_arch_stage();
        load_prepare_pending <= 1'b1;
        load_prepare_mode <= LOAD_PREP_STACK;
        load_prepare_bank <= bank;
        load_prepare_target_was_active <= bank == active_bank;
        load_prepare_ip <= arch_next_ip;
        load_prepare_flags <= arch_flags;
    end
endtask

task automatic apply_internal_write;
    input [8:0] index;
    input [7:0] value;
    begin
        if (!index[8]) begin
            internal_ram[index[7:0]] <= value;
        end else begin
            case (index)
                9'h100: p0_latch <= value;
                9'h101: pm0_reg <= value;
                9'h108: p1_latch <= value;
                9'h109: pm1_reg <= value;
                9'h110: p2_latch <= value;
                9'h111: pm2_reg <= value;
                9'h13b: pmt_reg <= value;
                9'h18a: md1_reg[7:0] <= value;
                9'h18b: md1_reg[15:8] <= value;
                9'h190: tmc0_reg <= value;
                9'h191: begin
                    tmc1_reg <= value & 8'hc0;
                    timer_count <= timer_period(md1_reg, value, prc_reg);
                end
                9'h19c: tmic0_reg <= value & 8'hd7;
                9'h19d: tmic1_reg <= value & 8'hd7;
                9'h19e: begin
                    tmic2_reg <= value & 8'hd7;
                    timer2_pending <= value[7];
                end
                9'h1e1: rfm_reg <= value;
                9'h1e8: wtc_lo <= value;
                9'h1e9: wtc_hi <= value;
                9'h1eb: begin
                    prc_reg <= value;
                    ramen <= value[6];
                end
                9'h1ff: idb_reg <= value;
                default: ;
            endcase
        end
    end
endtask

// The V25 is frozen at state_idle before this port is used. Internal RAM is
// transferred one byte per clock so save-state access does not synthesize
// eight additional 256:1 read and write ports.
always @(posedge clk) begin
    ss_ack <= 1'b0;

    if (!reset_n) begin
        ss_data_out <= 64'd0;
        ss_port_state <= SS_PORT_IDLE;
        ss_ram_index <= 8'd0;
        ss_byte_index <= 3'd0;
        ss_ram_buffer <= 64'd0;
        ss_ram_write_data <= 64'd0;
    end else begin
        case (ss_port_state)
            SS_PORT_IDLE: begin
                if (ss_selected) begin
                    if (ss_query) begin
                        ss_data_out <= {
                            SS_IDX, 22'd0, 2'd3, SS_WORD_COUNT
                        };
                        ss_ack <= 1'b1;
                    end else if (ss_read) begin
                        if (ss_addr < 32'd7) begin
                            case (ss_addr)
                                32'd0: ss_data_out <= {
                                    arch_bx, arch_dx, arch_cx, arch_ax
                                };
                                32'd1: ss_data_out <= {
                                    arch_di, arch_si, arch_bp, arch_sp
                                };
                                32'd2: ss_data_out <= {
                                    arch_ss, arch_es, arch_ds, arch_cs
                                };
                                32'd3: ss_data_out <= {
                                    23'd0,
                                    ramen,
                                    1'b0,
                                    irq_entry_pending,
                                    timer2_pending,
                                    fault_latch,
                                    arch_halted,
                                    active_bank,
                                    arch_flags,
                                    arch_next_ip
                                };
                                32'd4: ss_data_out <= {
                                    pm2_reg,
                                    pm1_reg,
                                    pm0_reg,
                                    p2_latch,
                                    p1_latch,
                                    p0_latch,
                                    prc_reg,
                                    idb_reg
                                };
                                32'd5: ss_data_out <= {
                                    tmic1_reg,
                                    tmic0_reg,
                                    tmc1_reg,
                                    tmc0_reg,
                                    wtc_hi,
                                    wtc_lo,
                                    rfm_reg,
                                    pmt_reg
                                };
                                32'd6: ss_data_out <= {
                                    timer_count,
                                    md1_reg,
                                    ispr_reg,
                                    tmic2_reg
                                };
                                default: ss_data_out <= 64'd0;
                            endcase
                            ss_ack <= 1'b1;
                        end else if (ss_addr < SS_WORD_COUNT) begin
                            ss_ram_index <=
                                (ss_addr[5:0] - 6'd7) << 3;
                            ss_byte_index <= 3'd0;
                            ss_ram_buffer <= 64'd0;
                            ss_port_state <= SS_PORT_READ_RAM;
                        end else begin
                            ss_data_out <= 64'd0;
                            ss_ack <= 1'b1;
                        end
                    end else if (ss_write) begin
                        if (ss_addr >= 32'd7 &&
                            ss_addr < SS_WORD_COUNT) begin
                            ss_ram_index <=
                                (ss_addr[5:0] - 6'd7) << 3;
                            ss_byte_index <= 3'd0;
                            ss_ram_write_data <= ss_data;
                            ss_port_state <= SS_PORT_WRITE_RAM;
                        end else begin
                            ss_ack <= 1'b1;
                        end
                    end
                end
            end

            SS_PORT_READ_RAM: begin
                if (ss_byte_index == 3'd7) begin
                    ss_data_out <= {
                        internal_ram[ss_ram_index],
                        ss_ram_buffer[55:0]
                    };
                    ss_ack <= 1'b1;
                    ss_port_state <= SS_PORT_WAIT_RELEASE;
                end else begin
                    ss_ram_buffer[ss_byte_index * 8 +: 8] <=
                        internal_ram[ss_ram_index];
                    ss_ram_index <= ss_ram_index + 8'd1;
                    ss_byte_index <= ss_byte_index + 3'd1;
                end
            end

            SS_PORT_WRITE_RAM: begin
                if (ss_byte_index == 3'd7) begin
                    ss_ack <= 1'b1;
                    ss_port_state <= SS_PORT_WAIT_RELEASE;
                end else begin
                    ss_ram_index <= ss_ram_index + 8'd1;
                    ss_byte_index <= ss_byte_index + 3'd1;
                end
            end

            SS_PORT_WAIT_RELEASE: begin
                if (!(ss_read || ss_write || ss_query))
                    ss_port_state <= SS_PORT_IDLE;
            end

            default: begin
                ss_port_state <= SS_PORT_IDLE;
                ss_ram_index <= 8'd0;
                ss_byte_index <= 3'd0;
                ss_ram_buffer <= 64'd0;
                ss_ram_write_data <= 64'd0;
            end
        endcase
    end
end

fixeight_z8086 u_cpu (
    .clk                    (clk),
    .clock_enable           (cpu_clock_enable),
    .reset_n                (reset_n),
    .addr                   (cpu_addr),
    .din                    (cpu_din),
    .dout                   (cpu_dout),
    .wr                     (cpu_wr),
    .rd                     (cpu_rd),
    .io                     (cpu_io),
    .word                   (cpu_word),
    .data_cycle             (cpu_data_cycle),
    .ready                  (cpu_ready),
    .intr                   (1'b0),
    .nmi                    (1'b0),
    .inta                   (),
    .secure_mode            (1'b1),
    .fetch_hold             (base_fetch_hold),
    .instruction_start_hold (base_instruction_start_hold),
    .arch_load              (cpu_arch_load),
    .arch_load_ax           (ss_restore_load ? ss_load_ax : load_ax),
    .arch_load_cx           (ss_restore_load ? ss_load_cx : load_cx),
    .arch_load_dx           (ss_restore_load ? ss_load_dx : load_dx),
    .arch_load_bx           (ss_restore_load ? ss_load_bx : load_bx),
    .arch_load_sp           (ss_restore_load ? ss_load_sp : load_sp),
    .arch_load_bp           (ss_restore_load ? ss_load_bp : load_bp),
    .arch_load_si           (ss_restore_load ? ss_load_si : load_si),
    .arch_load_di           (ss_restore_load ? ss_load_di : load_di),
    .arch_load_cs           (ss_restore_load ? ss_load_cs : load_cs),
    .arch_load_ds           (ss_restore_load ? ss_load_ds : load_ds),
    .arch_load_es           (ss_restore_load ? ss_load_es : load_es),
    .arch_load_ss           (ss_restore_load ? ss_load_ss : load_ss),
    .arch_load_ip           (ss_restore_load ? ss_load_ip : load_ip),
    .arch_load_flags        (ss_restore_load ? ss_load_flags : load_flags),
    .arch_load_halted       (ss_restore_load ? ss_load_halted : 1'b0),
    .arch_flags_write       (base_flags_write),
    .arch_flags_in          (commit_flags),
    .arch_ax                (arch_ax),
    .arch_cx                (arch_cx),
    .arch_dx                (arch_dx),
    .arch_bx                (arch_bx),
    .arch_sp                (arch_sp),
    .arch_bp                (arch_bp),
    .arch_si                (arch_si),
    .arch_di                (arch_di),
    .arch_cs                (arch_cs),
    .arch_ds                (arch_ds),
    .arch_es                (arch_es),
    .arch_ss                (arch_ss),
    .arch_next_ip           (arch_next_ip),
    .arch_flags             (arch_flags),
    .arch_halted            (arch_halted),
    .arch_segment_override  (arch_segment_override),
    .arch_override_segment  (arch_override_segment),
    .instruction_done       (instruction_done),
    .instruction_opcode     (instruction_opcode),
    .instruction_idle       (instruction_idle),
    .interrupt_deferred     (interrupt_deferred),
    .v25_escape_active      (escape_active),
    .v25_escape_lead        (escape_lead),
    .v25_escape_byte        (escape_byte),
    .v25_escape_byte_valid  (escape_byte_valid),
    .v25_escape_pop         (escape_pop),
    .v25_escape_complete    (escape_complete)
);

// Convert the compact core's 16-bit bus into the byte bus used by the sound
// subsystem. External transfer phases consume V25 clocks; the shared RAM is
// synchronous in the faster system domain and is therefore settled in time.
always @(posedge clk) begin
    if (!reset_n) begin
        bus_state <= BUS_IDLE;
        bus_wait_phase <= 5'd1;
        txn_ext <= 1'b0;
        txn_write <= 1'b0;
        txn_word <= 1'b0;
        txn_io <= 1'b0;
        txn_internal <= 1'b0;
        txn_phase <= 1'b0;
        txn_addr <= 20'h00000;
        txn_dout <= 16'h0000;
        txn_din <= 16'h0000;
        cpu_din <= 16'h0000;
    end else if (ss_restore_load) begin
        bus_state <= BUS_IDLE;
        bus_wait_phase <= 5'd1;
        txn_ext <= 1'b0;
        txn_write <= 1'b0;
        txn_word <= 1'b0;
        txn_io <= 1'b0;
        txn_internal <= 1'b0;
        txn_phase <= 1'b0;
        txn_addr <= 20'h00000;
        txn_dout <= 16'h0000;
        txn_din <= 16'h0000;
        cpu_din <= 16'h0000;
    end else begin
        case (bus_state)
            BUS_IDLE: begin
                txn_phase <= 1'b0;
                if (ext_req) begin
                    txn_ext <= 1'b1;
                    txn_write <= ext_write;
                    txn_word <= ext_word;
                    txn_io <= 1'b0;
                    txn_addr <= ext_addr;
                    txn_dout <= ext_dout;
                    txn_internal <= internal_selected(ext_addr, 1'b1);
                    bus_wait_phase <= 5'd1;
                    bus_state <= internal_selected(ext_addr, 1'b1) ?
                                 BUS_INTERNAL : BUS_EXT_WAIT1;
                end else if (cpu_rd || cpu_wr) begin
                    txn_ext <= 1'b0;
                    txn_write <= cpu_wr;
                    txn_word <= cpu_word;
                    txn_io <= cpu_io;
                    txn_addr <= cpu_addr;
                    txn_dout <= cpu_dout;
                    txn_internal <= !cpu_io &&
                                    internal_selected(cpu_addr, cpu_data_cycle);
                    bus_wait_phase <= 5'd1;
                    bus_state <= (!cpu_io &&
                                  internal_selected(cpu_addr, cpu_data_cycle)) ?
                                 BUS_INTERNAL : BUS_EXT_WAIT1;
                end
            end
            BUS_INTERNAL: begin
                if (bus_phase_advance) begin
                    if (!txn_write) begin
                        txn_din[7:0] <= internal_read_byte(int_index0);
                        txn_din[15:8] <= txn_word ?
                                          internal_read_byte(int_index1) : 8'h00;
                        cpu_din[7:0] <= internal_read_byte(int_index0);
                        cpu_din[15:8] <= txn_word ?
                                          internal_read_byte(int_index1) : 8'h00;
                    end
                    bus_state <= txn_ext ? BUS_ACK_EXT : BUS_ACK_CPU;
                end
            end
            BUS_EXT_WAIT1: begin
                if (bus_phase_advance) begin
                    if (bus_wait_phase < txn_byte_phases - 1'b1) begin
                        bus_wait_phase <= bus_wait_phase + 5'd1;
                    end else begin
                        if (!txn_write) begin
                            if (txn_phase)
                                txn_din[15:8] <= bus_din;
                            else begin
                                txn_din[7:0] <= bus_din;
                                cpu_din[7:0] <= bus_din;
                            end
                        end
                        bus_state <= BUS_EXT_GAP;
                    end
                end
            end
            BUS_EXT_GAP: begin
                if (bus_phase_advance) begin
                    if (txn_word && !txn_phase) begin
                        txn_phase <= 1'b1;
                        bus_wait_phase <= 5'd1;
                        bus_state <= BUS_EXT_WAIT1;
                    end else begin
                        if (!txn_write && txn_word)
                            cpu_din <= txn_din;
                        bus_state <= txn_ext ? BUS_ACK_EXT : BUS_ACK_CPU;
                    end
                end
            end
            BUS_ACK_CPU: begin
                if (clock_enable)
                    bus_state <= BUS_DROP_CPU;
            end
            BUS_DROP_CPU: begin
                if (!cpu_rd && !cpu_wr)
                    bus_state <= BUS_IDLE;
            end
            BUS_ACK_EXT: begin
                if (clock_enable)
                    bus_state <= BUS_DROP_EXT;
            end
            BUS_DROP_EXT: begin
                if (!ext_req)
                    bus_state <= BUS_IDLE;
            end
            default: bus_state <= BUS_IDLE;
        endcase
    end
end

integer reset_index;
reg [2:0] target_bank;
reg [15:0] saved_psw;
reg [15:0] return_ip;
reg [15:0] operation_value;
reg [15:0] operation_result;
reg [4:0] operation_count;
reg [15:0] data_segment;

// V25 extension controller, register banks, SFR writes, and timer.
always @(posedge clk) begin
    if (!reset_n) begin
        for (reset_index = 0; reset_index < 256; reset_index = reset_index + 1)
            internal_ram[reset_index] <= 8'h00;
        internal_ram[8'hec] <= 8'hff;
        internal_ram[8'hed] <= 8'hff;
        active_bank <= 3'd7;
        bank_commit_pending <= 1'b0;
        bank_commit_write_return <= 1'b0;
        bank_commit_source <= 3'd0;
        bank_commit_target <= 3'd0;
        bank_commit_psw <= 16'h0000;
        bank_commit_ip <= 16'h0000;
        arch_stage_ds <= 16'h0000;
        arch_stage_ss <= 16'h0000;
        arch_stage_cs <= 16'h0000;
        arch_stage_es <= 16'h0000;
        arch_stage_di <= 16'h0000;
        arch_stage_si <= 16'h0000;
        arch_stage_bp <= 16'h0000;
        arch_stage_sp <= 16'h0000;
        arch_stage_bx <= 16'h0000;
        arch_stage_dx <= 16'h0000;
        arch_stage_cx <= 16'h0000;
        arch_stage_ax <= 16'h0000;
        load_prepare_pending <= 1'b0;
        load_prepare_mode <= LOAD_PREP_NONE;
        load_prepare_bank <= 3'd0;
        load_prepare_target_was_active <= 1'b0;
        load_prepare_word <= 1'b0;
        load_prepare_register <= 3'd0;
        load_prepare_value <= 16'h0000;
        load_prepare_ip <= 16'h0000;
        load_prepare_flags <= 16'h0000;
        idb_reg <= 8'hff;
        ramen <= 1'b1;
        prc_reg <= 8'h4e;
        p0_latch <= 8'h00;
        p1_latch <= 8'h00;
        p2_latch <= 8'h00;
        pm0_reg <= 8'hff;
        pm1_reg <= 8'hff;
        pm2_reg <= 8'hff;
        pmt_reg <= 8'h00;
        rfm_reg <= 8'hfc;
        wtc_lo <= 8'hff;
        wtc_hi <= 8'hff;
        md1_reg <= 16'h0000;
        tmc0_reg <= 8'h00;
        tmc1_reg <= 8'h00;
        tmic0_reg <= 8'h47;
        tmic1_reg <= 8'h47;
        tmic2_reg <= 8'h47;
        ispr_reg <= 8'h00;
        timer_count <= 32'd0;
        timer2_pending <= 1'b0;
        irq_entry_pending <= 1'b0;
        escape_state <= ESC_IDLE;
        escape_delay_count <= 5'd0;
        instruction_gap_count <= 5'd0;
        escape_timing_profile <= 1'b0;
        escape_opcode <= 8'h00;
        escape_modrm <= 8'h00;
        escape_disp <= 16'h0000;
        escape_imm <= 8'h00;
        escape_bit <= 4'd0;
        escape_mem_value <= 16'h0000;
        commit_flags <= 16'h0002;
        fault_latch <= 1'b0;
        load_ax <= 16'h0000;
        load_cx <= 16'h0000;
        load_dx <= 16'h0000;
        load_bx <= 16'h0000;
        load_sp <= 16'h0000;
        load_bp <= 16'h0000;
        load_si <= 16'h0000;
        load_di <= 16'h0000;
        load_cs <= 16'hffff;
        load_ds <= 16'h0000;
        load_es <= 16'h0000;
        load_ss <= 16'h0000;
        load_ip <= 16'h0000;
        load_flags <= 16'hf002;
        ss_load_ax <= 16'h0000;
        ss_load_cx <= 16'h0000;
        ss_load_dx <= 16'h0000;
        ss_load_bx <= 16'h0000;
        ss_load_sp <= 16'h0000;
        ss_load_bp <= 16'h0000;
        ss_load_si <= 16'h0000;
        ss_load_di <= 16'h0000;
        ss_load_cs <= 16'hffff;
        ss_load_ds <= 16'h0000;
        ss_load_es <= 16'h0000;
        ss_load_ss <= 16'h0000;
        ss_load_ip <= 16'h0000;
        ss_load_flags <= 16'hf002;
        ss_load_halted <= 1'b0;
        ext_req <= 1'b0;
        ext_write <= 1'b0;
        ext_word <= 1'b0;
        ext_addr <= 20'h00000;
        ext_dout <= 16'h0000;
    end else if (ss_ram_write_active) begin
        if (ss_restore_enable) begin
            internal_ram[ss_ram_index] <=
                ss_ram_write_data[ss_byte_index * 8 +: 8];
        end
    end else if (ss_state_write && ss_restore_enable) begin
        case (ss_addr)
            32'd0: begin
                ss_load_ax <= ss_data[15:0];
                ss_load_cx <= ss_data[31:16];
                ss_load_dx <= ss_data[47:32];
                ss_load_bx <= ss_data[63:48];
            end
            32'd1: begin
                ss_load_sp <= ss_data[15:0];
                ss_load_bp <= ss_data[31:16];
                ss_load_si <= ss_data[47:32];
                ss_load_di <= ss_data[63:48];
            end
            32'd2: begin
                ss_load_cs <= ss_data[15:0];
                ss_load_ds <= ss_data[31:16];
                ss_load_es <= ss_data[47:32];
                ss_load_ss <= ss_data[63:48];
            end
            32'd3: begin
                ss_load_ip <= ss_data[15:0];
                ss_load_flags <= ss_data[31:16];
                active_bank <= ss_data[34:32];
                ss_load_halted <= ss_data[35];
                fault_latch <= ss_data[36];
                timer2_pending <= ss_data[37];
                irq_entry_pending <= ss_data[38];
                ramen <= ss_data[40];
            end
            32'd4: begin
                idb_reg <= ss_data[7:0];
                prc_reg <= ss_data[15:8];
                p0_latch <= ss_data[23:16];
                p1_latch <= ss_data[31:24];
                p2_latch <= ss_data[39:32];
                pm0_reg <= ss_data[47:40];
                pm1_reg <= ss_data[55:48];
                pm2_reg <= ss_data[63:56];
            end
            32'd5: begin
                pmt_reg <= ss_data[7:0];
                rfm_reg <= ss_data[15:8];
                wtc_lo <= ss_data[23:16];
                wtc_hi <= ss_data[31:24];
                tmc0_reg <= ss_data[39:32];
                tmc1_reg <= ss_data[47:40];
                tmic0_reg <= ss_data[55:48];
                tmic1_reg <= ss_data[63:56];
            end
            32'd6: begin
                tmic2_reg <= ss_data[7:0];
                ispr_reg <= ss_data[15:8];
                md1_reg <= ss_data[31:16];
                timer_count <= ss_data[63:32];
            end
            default: ;
        endcase
    end else if (ss_restore_load) begin
        bank_commit_pending <= 1'b0;
        bank_commit_write_return <= 1'b0;
        load_prepare_pending <= 1'b0;
        load_prepare_mode <= LOAD_PREP_NONE;
        escape_state <= ESC_IDLE;
        escape_delay_count <= 5'd0;
        instruction_gap_count <= 5'd0;
        escape_timing_profile <= 1'b0;
        escape_opcode <= 8'h00;
        escape_modrm <= 8'h00;
        escape_disp <= 16'h0000;
        escape_imm <= 8'h00;
        escape_bit <= 4'd0;
        escape_mem_value <= 16'h0000;
        commit_flags <= ss_load_flags;
        load_ax <= ss_load_ax;
        load_cx <= ss_load_cx;
        load_dx <= ss_load_dx;
        load_bx <= ss_load_bx;
        load_sp <= ss_load_sp;
        load_bp <= ss_load_bp;
        load_si <= ss_load_si;
        load_di <= ss_load_di;
        load_cs <= ss_load_cs;
        load_ds <= ss_load_ds;
        load_es <= ss_load_es;
        load_ss <= ss_load_ss;
        load_ip <= ss_load_ip;
        load_flags <= ss_load_flags;
        ext_req <= 1'b0;
        ext_write <= 1'b0;
        ext_word <= 1'b0;
        ext_addr <= 20'h00000;
        ext_dout <= 16'h0000;
    end else begin
        if (bank_commit_pending) begin
            store_word(bank_commit_source, 5'h08, arch_stage_ds);
            store_word(bank_commit_source, 5'h0a, arch_stage_ss);
            store_word(bank_commit_source, 5'h0c, arch_stage_cs);
            store_word(bank_commit_source, 5'h0e, arch_stage_es);
            store_word(bank_commit_source, 5'h10, arch_stage_di);
            store_word(bank_commit_source, 5'h12, arch_stage_si);
            store_word(bank_commit_source, 5'h14, arch_stage_bp);
            store_word(bank_commit_source, 5'h16, arch_stage_sp);
            store_word(bank_commit_source, 5'h18, arch_stage_bx);
            store_word(bank_commit_source, 5'h1a, arch_stage_dx);
            store_word(bank_commit_source, 5'h1c, arch_stage_cx);
            store_word(bank_commit_source, 5'h1e, arch_stage_ax);
            if (bank_commit_write_return) begin
                store_word(bank_commit_target, 5'h04, bank_commit_psw);
                store_word(bank_commit_target, 5'h06, bank_commit_ip);
            end
            bank_commit_pending <= 1'b0;
            bank_commit_write_return <= 1'b0;
        end

        if (load_prepare_pending) begin
            if (load_prepare_mode == LOAD_PREP_BANK &&
                !load_prepare_target_was_active) begin
                load_ax <= stored_bank_word(load_prepare_bank, 5'h1e);
                load_cx <= stored_bank_word(load_prepare_bank, 5'h1c);
                load_dx <= stored_bank_word(load_prepare_bank, 5'h1a);
                load_bx <= stored_bank_word(load_prepare_bank, 5'h18);
                load_sp <= stored_bank_word(load_prepare_bank, 5'h16);
                load_bp <= stored_bank_word(load_prepare_bank, 5'h14);
                load_si <= stored_bank_word(load_prepare_bank, 5'h12);
                load_di <= stored_bank_word(load_prepare_bank, 5'h10);
                load_cs <= stored_bank_word(load_prepare_bank, 5'h0c);
                load_ds <= stored_bank_word(load_prepare_bank, 5'h08);
                load_es <= stored_bank_word(load_prepare_bank, 5'h0e);
                load_ss <= stored_bank_word(load_prepare_bank, 5'h0a);
            end else begin
                load_ax <= arch_stage_ax;
                load_cx <= arch_stage_cx;
                load_dx <= arch_stage_dx;
                load_bx <= arch_stage_bx;
                load_sp <= arch_stage_sp;
                load_bp <= arch_stage_bp;
                load_si <= arch_stage_si;
                load_di <= arch_stage_di;
                load_cs <= arch_stage_cs;
                load_ds <= arch_stage_ds;
                load_es <= arch_stage_es;
                load_ss <= arch_stage_ss;
            end
            load_ip <= load_prepare_ip;
            load_flags <= load_prepare_flags;

            case (load_prepare_mode)
                LOAD_PREP_REGISTER: begin
                    if (load_prepare_word) begin
                        case (load_prepare_register)
                            3'd0: load_ax <= load_prepare_value;
                            3'd1: load_cx <= load_prepare_value;
                            3'd2: load_dx <= load_prepare_value;
                            3'd3: load_bx <= load_prepare_value;
                            3'd4: load_sp <= load_prepare_value;
                            3'd5: load_bp <= load_prepare_value;
                            3'd6: load_si <= load_prepare_value;
                            3'd7: load_di <= load_prepare_value;
                        endcase
                    end else begin
                        case (load_prepare_register)
                            3'd0: load_ax <= {
                                arch_stage_ax[15:8],
                                load_prepare_value[7:0]
                            };
                            3'd1: load_cx <= {
                                arch_stage_cx[15:8],
                                load_prepare_value[7:0]
                            };
                            3'd2: load_dx <= {
                                arch_stage_dx[15:8],
                                load_prepare_value[7:0]
                            };
                            3'd3: load_bx <= {
                                arch_stage_bx[15:8],
                                load_prepare_value[7:0]
                            };
                            3'd4: load_ax <= {
                                load_prepare_value[7:0],
                                arch_stage_ax[7:0]
                            };
                            3'd5: load_cx <= {
                                load_prepare_value[7:0],
                                arch_stage_cx[7:0]
                            };
                            3'd6: load_dx <= {
                                load_prepare_value[7:0],
                                arch_stage_dx[7:0]
                            };
                            3'd7: load_bx <= {
                                load_prepare_value[7:0],
                                arch_stage_bx[7:0]
                            };
                        endcase
                    end
                end

                LOAD_PREP_PUSH:
                    load_sp <= load_prepare_value;

                LOAD_PREP_STACK: begin
                    if (load_prepare_target_was_active) begin
                        load_ss <= arch_stage_ss;
                        load_sp <= arch_stage_sp;
                    end else begin
                        load_ss <= stored_bank_word(
                            load_prepare_bank, 5'h0a
                        );
                        load_sp <= stored_bank_word(
                            load_prepare_bank, 5'h16
                        );
                    end
                end

                default: ;
            endcase

            load_prepare_pending <= 1'b0;
            load_prepare_mode <= LOAD_PREP_NONE;
        end

        if (clock_enable && tmc1_reg[7] && md1_reg != 16'h0000) begin
            if (timer_count <= 32'd1) begin
                timer_count <= timer_period(md1_reg, tmc1_reg, prc_reg);
                timer2_pending <= 1'b1;
            end else begin
                timer_count <= timer_count - 32'd1;
            end
        end

        if (clock_enable) begin
            if (instruction_gap_start)
                instruction_gap_count <= instruction_gap_load;
            else if (instruction_gap_count != 5'd0)
                instruction_gap_count <= instruction_gap_count - 5'd1;
        end

        if (int_write0)
            apply_internal_write(int_index0, txn_dout[7:0]);
        if (int_write1)
            apply_internal_write(int_index1, txn_dout[15:8]);

        if (clock_enable) begin
            // A request may become pending while BRKCS is draining. Do not
            // carry that latch into the banked handler after its load clears
            // IF; the hardware leaves the timer pending and services it once
            // RETRBI restores an interruptible PSW.
            if (!timer2_irq_request)
                irq_entry_pending <= 1'b0;
            else if (!irq_entry_pending)
                irq_entry_pending <= 1'b1;

            if (escape_delay_count != 5'd0) begin
                escape_delay_count <= escape_delay_count - 5'd1;
            end else case (escape_state)
                ESC_IDLE: begin
                    if (escape_active) begin
                        escape_timing_profile <=
                            instruction_timing_profile_active;
                        if (escape_lead == 8'h0f) begin
                            escape_state <= ESC_OPCODE;
                        end else begin
                            escape_opcode <= escape_lead;
                            escape_disp <= 16'h0000;
                            if (escape_lead == 8'h68)
                                escape_state <= ESC_DISP_LO;
                            else if (escape_lead == 8'h6a)
                                escape_state <= ESC_IMMEDIATE;
                            else
                                escape_state <= ESC_MODRM;
                        end
                    end else if (irq_entry_pending &&
                                 timer2_irq_request &&
                                 irq_entry_safe) begin
                        target_bank = tmic0_reg[2:0];
                        saved_psw = packed_psw(arch_flags, active_bank);
                        stage_bank_switch(
                            1'b1,
                            target_bank,
                            saved_psw,
                            arch_next_ip,
                            live_bank_word(target_bank, 5'h02),
                            (saved_psw & 16'hfcff & 16'h8fff) |
                            {1'b0, target_bank, 12'h000}
                        );
                        active_bank <= target_bank;
                        ispr_reg[target_bank] <= 1'b1;
                        timer2_pending <= 1'b0;
                        irq_entry_pending <= 1'b0;
                        escape_state <= ESC_LOAD_COMMIT;
                    end
                end

                ESC_OPCODE: begin
                    if (escape_byte_valid) begin
                        escape_opcode <= escape_byte;
                        if ((escape_byte >= 8'h10 && escape_byte <= 8'h1f) ||
                            escape_byte == 8'h2d)
                            escape_state <= ESC_MODRM;
                        else if (escape_byte == 8'h25 ||
                                 escape_byte == 8'h91 ||
                                 escape_byte == 8'h92)
                            escape_state <= ESC_PREPARE;
                        else
                            escape_state <= ESC_FAULT_COMMIT;
                    end
                end

                ESC_MODRM: begin
                    if (escape_byte_valid) begin
                        escape_modrm <= escape_byte;
                        escape_disp <= 16'h0000;
                        if (escape_opcode == 8'h2d)
                            escape_state <= ESC_PREPARE;
                        else if (escape_byte[7:6] == 2'b01)
                            escape_state <= ESC_DISP8;
                        else if (escape_byte[7:6] == 2'b10 ||
                                 (escape_byte[7:6] == 2'b00 &&
                                  escape_byte[2:0] == 3'b110))
                            escape_state <= ESC_DISP_LO;
                        else if (escape_opcode >= 8'h18)
                            escape_state <= ESC_IMMEDIATE;
                        else
                            escape_state <= ESC_PREPARE;
                    end
                end

                ESC_DISP8: begin
                    if (escape_byte_valid) begin
                        escape_disp <= {{8{escape_byte[7]}}, escape_byte};
                        escape_state <= escape_opcode >= 8'h18 ?
                                        ESC_IMMEDIATE : ESC_PREPARE;
                    end
                end

                ESC_DISP_LO: begin
                    if (escape_byte_valid) begin
                        escape_disp[7:0] <= escape_byte;
                        escape_state <= ESC_DISP_HI;
                    end
                end

                ESC_DISP_HI: begin
                    if (escape_byte_valid) begin
                        escape_disp[15:8] <= escape_byte;
                        escape_state <= (escape_opcode >= 8'h18 &&
                                         escape_opcode != 8'h68) ?
                                        ESC_IMMEDIATE : ESC_PREPARE;
                    end
                end

                ESC_IMMEDIATE: begin
                    if (escape_byte_valid) begin
                        escape_imm <= escape_byte;
                        escape_bit <= escape_is_word ?
                                      escape_byte[3:0] : {1'b0, escape_byte[2:0]};
                        escape_state <= ESC_PREPARE;
                    end
                end

                ESC_PREPARE: begin
                    if (escape_opcode == 8'h68 || escape_opcode == 8'h6a) begin
                        stage_push_load(arch_sp - 16'd2);
                        ext_addr <= {arch_ss, 4'b0000} +
                                    {4'b0000, arch_sp - 16'd2};
                        ext_dout <= escape_opcode == 8'h68 ?
                                    escape_disp : {{8{escape_imm[7]}}, escape_imm};
                        ext_write <= 1'b1;
                        ext_word <= 1'b1;
                        ext_req <= 1'b1;
                        escape_state <= ESC_PUSH_WRITE;
                    end else if (escape_opcode == 8'hc0 || escape_opcode == 8'hc1) begin
                        if (escape_modrm[7:6] != 2'b11 ||
                            escape_modrm[5:3] > 3'd1) begin
                            escape_state <= ESC_FAULT_COMMIT;
                        end else begin
                            operation_value = escape_register_value;
                            operation_result = operation_value;
                            commit_flags = arch_flags;
                            operation_count = escape_is_word ?
                                              {1'b0, escape_imm[3:0]} :
                                              {2'b00, escape_imm[2:0]};

                            if (operation_count != 5'd0) begin
                                if (escape_modrm[5:3] == 3'd0) begin
                                    if (escape_is_word)
                                        operation_result =
                                            (operation_value << operation_count) |
                                            (operation_value >> (5'd16 - operation_count));
                                    else
                                        operation_result[7:0] =
                                            (operation_value[7:0] << operation_count) |
                                            (operation_value[7:0] >> (5'd8 - operation_count));
                                    commit_flags[0] = operation_result[0];
                                    if (operation_count == 5'd1)
                                        commit_flags[11] = escape_is_word ?
                                            operation_result[15] ^ operation_result[0] :
                                            operation_result[7] ^ operation_result[0];
                                end else begin
                                    if (escape_is_word)
                                        operation_result =
                                            (operation_value >> operation_count) |
                                            (operation_value << (5'd16 - operation_count));
                                    else
                                        operation_result[7:0] =
                                            (operation_value[7:0] >> operation_count) |
                                            (operation_value[7:0] << (5'd8 - operation_count));
                                    commit_flags[0] = escape_is_word ?
                                        operation_result[15] : operation_result[7];
                                    if (operation_count == 5'd1)
                                        commit_flags[11] = escape_is_word ?
                                            operation_result[15] ^ operation_result[14] :
                                            operation_result[7] ^ operation_result[6];
                                end
                            end

                            stage_register_load(
                                operation_result, commit_flags
                            );
                            escape_state <= ESC_LOAD_COMMIT;
                        end
                    end else if (escape_opcode == 8'h25) begin
                        saved_psw = live_bank_word(active_bank, 5'h04);
                        target_bank = saved_psw[14:12];
                        stage_stack_load(target_bank);
                        escape_state <= ESC_LOAD_COMMIT;
                    end else if (escape_opcode == 8'h91) begin
                        saved_psw = live_bank_word(active_bank, 5'h04);
                        return_ip = live_bank_word(active_bank, 5'h06);
                        target_bank = saved_psw[14:12];
                        stage_bank_switch(
                            1'b0,
                            target_bank,
                            16'h0000,
                            16'h0000,
                            return_ip,
                            saved_psw
                        );
                        active_bank <= target_bank;
                        escape_state <= ESC_LOAD_COMMIT;
                    end else if (escape_opcode == 8'h92) begin
                        if (ispr_reg[0]) ispr_reg[0] <= 1'b0;
                        else if (ispr_reg[1]) ispr_reg[1] <= 1'b0;
                        else if (ispr_reg[2]) ispr_reg[2] <= 1'b0;
                        else if (ispr_reg[3]) ispr_reg[3] <= 1'b0;
                        else if (ispr_reg[4]) ispr_reg[4] <= 1'b0;
                        else if (ispr_reg[5]) ispr_reg[5] <= 1'b0;
                        else if (ispr_reg[6]) ispr_reg[6] <= 1'b0;
                        else if (ispr_reg[7]) ispr_reg[7] <= 1'b0;
                        // MAME's V20 timing is ten processor clocks here;
                        // the compact escape path supplies eleven half-clocks.
                        escape_delay_count <= 5'd9;
                        escape_state <= ESC_DONE_COMMIT;
                    end else if (escape_opcode == 8'h2d) begin
                        if (escape_modrm[7:6] != 2'b11) begin
                            escape_state <= ESC_FAULT_COMMIT;
                        end else begin
                            operation_value = gpr16_read(escape_modrm[2:0]);
                            target_bank = operation_value[2:0];
                            saved_psw = packed_psw(arch_flags, active_bank);
                            stage_bank_switch(
                                1'b1,
                                target_bank,
                                saved_psw,
                                arch_next_ip,
                                live_bank_word(target_bank, 5'h02),
                                (saved_psw & 16'h8cff) |
                                {1'b0, target_bank, 12'h000}
                            );
                            active_bank <= target_bank;
                            escape_state <= ESC_LOAD_COMMIT;
                        end
                    end else if (escape_opcode >= 8'h10 &&
                                 escape_opcode <= 8'h1f) begin
                        if (escape_opcode < 8'h18)
                            escape_bit <= escape_is_word ?
                                          arch_cx[3:0] : {1'b0, arch_cx[2:0]};
                        if (escape_modrm[7:6] == 2'b11) begin
                            operation_value = escape_register_value;
                            if (escape_test) begin
                                commit_flags = arch_flags;
                                commit_flags[11] = 1'b0;
                                commit_flags[0] = 1'b0;
                                commit_flags[6] =
                                    (operation_value & escape_bit_mask) == 16'h0000;
                                // Register TEST1 is 16 clocks with a CL bit
                                // index and 20 clocks with an immediate index.
                                escape_delay_count <=
                                    escape_opcode >= 8'h18 ?
                                    TEST1_IMMEDIATE_DELAY : 5'd11;
                                escape_state <= ESC_FLAGS_COMMIT;
                            end else begin
                                operation_result = operation_value;
                                if (escape_clear)
                                    operation_result = operation_value & ~escape_bit_mask;
                                else if (escape_set)
                                    operation_result = operation_value | escape_bit_mask;
                                else if (escape_toggle)
                                    operation_result = operation_value ^ escape_bit_mask;
                                stage_register_load(
                                    operation_result, arch_flags
                                );
                                escape_state <= ESC_LOAD_COMMIT;
                            end
                        end else begin
                            data_segment = arch_segment_override ?
                                           arch_override_segment :
                                           (ea_defaults_ss(escape_modrm[2:0]) ?
                                            arch_ss : arch_ds);
                            ext_addr <= {data_segment, 4'b0000} +
                                        {4'b0000,
                                         ea_base(escape_modrm[2:0]) + escape_disp};
                            ext_write <= 1'b0;
                            ext_word <= escape_is_word;
                            ext_req <= 1'b1;
                            escape_state <= ESC_MEM_READ;
                        end
                    end else begin
                        escape_state <= ESC_FAULT_COMMIT;
                    end
                end

                ESC_MEM_READ: begin
                    if (ext_ready) begin
                        ext_req <= 1'b0;
                        escape_mem_value <= ext_din;
                        if (escape_test) begin
                            commit_flags = arch_flags;
                            commit_flags[11] = 1'b0;
                            commit_flags[0] = 1'b0;
                            commit_flags[6] =
                                (ext_din & escape_bit_mask) == 16'h0000;
                            if (escape_timing_profile)
                                escape_delay_count <=
                                    escape_opcode >= 8'h18 ? 5'd18 : 5'd8;
                            else
                                escape_delay_count <= TEST1_MEMORY_DELAY;
                            escape_state <= ESC_FLAGS_COMMIT;
                        end else begin
                            operation_result = ext_din;
                            if (escape_clear)
                                operation_result = ext_din & ~escape_bit_mask;
                            else if (escape_set)
                                operation_result = ext_din | escape_bit_mask;
                            else if (escape_toggle)
                                operation_result = ext_din ^ escape_bit_mask;
                            ext_dout <= operation_result;
                            escape_state <= ESC_MEM_GAP;
                        end
                    end
                end

                ESC_MEM_GAP: begin
                    ext_write <= 1'b1;
                    ext_req <= 1'b1;
                    escape_state <= ESC_MEM_WRITE;
                end

                ESC_MEM_WRITE: begin
                    if (ext_ready) begin
                        ext_req <= 1'b0;
                        ext_write <= 1'b0;
                        if (escape_opcode == 8'h1a ||
                            escape_opcode == 8'h1b)
                            escape_delay_count <= 5'd6;
                        escape_state <= ESC_DONE_COMMIT;
                    end
                end

                ESC_PUSH_WRITE: begin
                    if (ext_ready) begin
                        ext_req <= 1'b0;
                        ext_write <= 1'b0;
                        escape_state <= ESC_LOAD_COMMIT;
                    end
                end

                ESC_FLAGS_COMMIT,
                ESC_DONE_COMMIT: begin
                    escape_state <= ESC_IDLE;
                end

                ESC_LOAD_COMMIT: begin
                    if (!load_prepare_pending)
                        escape_state <= ESC_IDLE;
                end

                ESC_FAULT_COMMIT: begin
                    fault_latch <= 1'b1;
                    escape_state <= ESC_IDLE;
                end

                default: begin
                    fault_latch <= 1'b1;
                    escape_state <= ESC_FAULT_COMMIT;
                end
            endcase
        end
    end
end

`ifdef DOGYUUN_V25_TRACE
wire debug_ext_write_start =
    reset_n && bus_state == BUS_IDLE && ext_req && ext_write &&
    !internal_selected(ext_addr, 1'b1);
wire debug_cpu_write_start =
    reset_n && bus_state == BUS_IDLE && !ext_req &&
    cpu_wr && !cpu_io && !internal_selected(cpu_addr, cpu_data_cycle);
wire debug_bus_write_start =
    debug_ext_write_start || debug_cpu_write_start;
wire [19:0] debug_bus_write_addr =
    debug_ext_write_start ? ext_addr : cpu_addr;
wire [7:0] debug_bus_write_data =
    debug_ext_write_start ? ext_dout[7:0] : cpu_dout[7:0];

wire debug_timer_bank_start =
    clock_enable && escape_delay_count == 5'd0 &&
    escape_state == ESC_IDLE && !escape_active &&
    irq_entry_pending && timer2_irq_request && irq_entry_safe;
wire debug_retrbi_bank_start =
    clock_enable && escape_delay_count == 5'd0 &&
    escape_state == ESC_PREPARE && escape_opcode == 8'h91;
wire debug_brkcs_bank_start =
    clock_enable && escape_delay_count == 5'd0 &&
    escape_state == ESC_PREPARE && escape_opcode == 8'h2d &&
    escape_modrm[7:6] == 2'b11;

reg [2:0] debug_bank_cause;
always @(posedge clk) begin
    if (!reset_n || ss_restore_load) begin
        debug_bank_cause <= 3'd0;
    end else if (debug_timer_bank_start) begin
        debug_bank_cause <= 3'd1;
    end else if (debug_retrbi_bank_start) begin
        debug_bank_cause <= 3'd2;
    end else if (debug_brkcs_bank_start) begin
        debug_bank_cause <= 3'd3;
    end
end

wire [15:0] debug_bank_psw =
    debug_bank_cause == 3'd2 ? load_prepare_flags : bank_commit_psw;
wire [15:0] debug_bank_ip =
    debug_bank_cause == 3'd2 ? load_prepare_ip : bank_commit_ip;
wire debug_restore_event =
    ss_restore_load && ss_restore_load_count == 3'd4;

wire [31:0] v25_debug_source;
wire [127:0] v25_debug_probe;

fixeight_v25_debug_trace u_v25_debug_trace (
    .clk                 (clk),
    .reset_n             (reset_n),
    .clock_enable        (clock_enable),
    .instruction_done    (instruction_done),
    .debug_pc            (instruction_next_pc),
    .active_bank         (active_bank),
    .arch_flags          (arch_flags),
    .arch_cx             (arch_cx),
    .arch_ix             (arch_si),
    .halted              (arch_halted),
    .fault               (fault_latch),
    .timer2_pending      (timer2_pending),
    .timer2_irq_request  (timer2_irq_request),
    .irq_entry_pending   (irq_entry_pending),
    .tmic0_reg           (tmic0_reg),
    .ispr_reg            (ispr_reg),
    .escape_state        (escape_state),
    .escape_opcode       (escape_opcode),
    .bus_write_start     (debug_bus_write_start),
    .bus_write_addr      (debug_bus_write_addr),
    .bus_write_data      (debug_bus_write_data),
    .bank_commit         (bank_commit_pending),
    .bank_cause          (debug_bank_cause),
    .bank_source         (bank_commit_source),
    .bank_target         (bank_commit_target),
    .bank_psw            (debug_bank_psw),
    .bank_ip             (debug_bank_ip),
    .restore_event       (debug_restore_event),
    .control             (v25_debug_source),
    .probe               (v25_debug_probe)
);

altsource_probe #(
    .sld_auto_instance_index ("NO"),
    .sld_instance_index      (0),
    .instance_id             ("V25D"),
    .probe_width             (128),
    .source_width            (32),
    .source_initial_value    ("00000000"),
    .enable_metastability    ("NO")
) u_v25_debug_jtag (
    .probe  (v25_debug_probe),
    .source (v25_debug_source)
);
`endif

endmodule
