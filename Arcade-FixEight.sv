//============================================================================
//  JTFRAME by Jose Tejada Gomez. Twitter: @topapate
//
//  Port to MiSTer
//  Thanks to Sorgelig for his continuous support
//  Original repository: http://github.com/jotego/jt_gng
//
//  This program is free software; you can redistribute it and/or modify it
//  under the terms of the GNU General Public License as published by the Free
//  Software Foundation; either version 2 of the License, or (at your option)
//  any later version.
//
//  This program is distributed in the hope that it will be useful, but WITHOUT
//  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
//  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
//  more details.
//
//  You should have received a copy of the GNU General Public License along
//  with this program; if not, write to the Free Software Foundation, Inc.,
//  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
//============================================================================

`ifdef JTFRAME_VERTICAL
`define JTFRAME_MR_DDR
`endif

`ifdef JTFRAME_MR_DDRLOAD
`define JTFRAME_MR_DDR
`endif

module emu
(
	`include "sys/emu_ports.vh"
);


`ifdef JTFRAME_SDRAM_LARGE
    localparam SDRAMW=23; // 64 MB
`else
    localparam SDRAMW=22; // 32 MB
`endif

`ifndef JTFRAME_INTERLACED
assign VGA_F1=1'b0;
`else
wire   field;
assign VGA_F1=field;
`endif

// unused features
assign VGA_SCALER    = 0;
assign VGA_DISABLE   = 0;
assign HDMI_FREEZE   = ss_active &&
                       (ss_state_debug != 4'd1) &&
                       (ss_state_debug != 4'd2) &&
                       (ss_state_debug != 4'd6) &&
                       (ss_state_debug != 4'd14);
assign HDMI_BLACKOUT = 0;
assign HDMI_BOB_DEINT= 0;
assign AUDIO_MIX     = 0;
assign BUTTONS       = 0;
assign {SD_SCK, SD_MOSI, SD_CS} = 'Z;
assign {UART_RTS, UART_TXD, UART_DTR} = 0;

`include "rtl/fixeight/fixeight_conf_str.svh"

wire [1:0] dial_x, dial_y;
wire [1:0] rotate;
wire       db15_en;
wire       uart_en;
wire       gun_border_en;
wire       show_osd;

////////////////////   CLOCKS   ///////////////////

wire clk_sys, clk_rom, clk96, clk96sh, clk48, clk48sh, clk24;
wire game_rst, game_rst_n, game_service, game_tilt, rst, rst_n;
wire clk_pico;
wire pxl2_cen, pxl_cen;
wire rst96, rst48, rst24;
wire pll_locked;
reg  pll_rst = 1'b0;
wire sys_rst;

// Resets the PLL if it looses lock
jtframe_sync u_sync(
    .clk_in     ( CLK_50M   ),
    .clk_out    ( clk_sys   ),
    .raw        ( RESET     ),
    .sync       ( sys_rst   )
);

always @(posedge clk_sys or posedge sys_rst) begin : pll_controller
    reg last_locked;
    reg [7:0] rst_cnt;

    if( sys_rst ) begin
        pll_rst <= 1'b0;
        rst_cnt <= 8'hd0;
    end else begin
        last_locked <= pll_locked;
        if( last_locked && !pll_locked ) begin
            rst_cnt <= 8'hff; // keep reset high for 256 cycles
            pll_rst <= 1'b1;
        end else begin
            if( rst_cnt != 8'h00 )
                rst_cnt <= rst_cnt - 8'h1;
            else
                pll_rst <= 1'b0;
        end
    end
end

// There are many false paths defined in the
// SDC file between this PLL and the ones
// used in sys_top
raizingpll pll(
    .refclk     ( CLK_50M    ),
    .rst        ( pll_rst    ),
    .locked     ( pll_locked ),
    .outclk_0   ( clk48      ),
    .outclk_1   ( clk48sh    ),
    .outclk_2   ( clk24      ),
    .outclk_3   (            ),
    .outclk_4   ( clk96      ),
    .outclk_5   ( clk96sh    )
);

jtframe_rst_sync u_reset96(
    .rst        ( game_rst  ),
    .clk        ( clk96     ),
    .rst_sync   ( rst96     )
);

jtframe_rst_sync u_reset48(
    .rst        ( game_rst  ),
    .clk        ( clk48     ),
    .rst_sync   ( rst48     )
);

jtframe_rst_sync u_reset24(
    .rst        ( game_rst  ),
    .clk        ( clk24     ),
    .rst_sync   ( rst24     )
);

`ifdef JTFRAME_SDRAM96
    assign clk_rom = clk96;
    assign clk_sys = clk96;
`else
    assign clk_rom = clk48;
    assign clk_sys = clk48;
`endif

assign clk_pico = clk48;

generate
    if( `JTFRAME_180SHIFT == 0 ) begin
        `ifdef JTFRAME_SDRAM96
        assign SDRAM_CLK   = clk96sh;
        `else
        assign SDRAM_CLK   = clk48sh;
        `endif
    end else begin
        altddio_out
        #(
            .extend_oe_disable("OFF"),
            .intended_device_family("Cyclone V"),
            .invert_output("OFF"),
            .lpm_hint("UNUSED"),
            .lpm_type("altddio_out"),
            .oe_reg("UNREGISTERED"),
            .power_up_high("OFF"),
            .width(1)
        )
        sdramclk_ddr
        (
            .datain_h(1'b0),
            .datain_l(1'b1),
            .outclock(clk_rom),
            .dataout(SDRAM_CLK),
            .aclr(1'b0),
            .aset(1'b0),
            .oe(1'b1),
            .outclocken(1'b1),
            .sclr(1'b0),
            .sset(1'b0)
        );
    end
endgenerate

///////////////////////////////////////////////////

wire [127:0] status;
wire [ 1:0] buttons;
wire [15:0] status_menumask;
wire [21:0] gamma_bus;
wire        direct_video, force_scan2x, video_rotated;
wire [10:0] ps2_key;
wire        ss_save;
wire        ss_load;
wire        ss_busy;
wire        ss_stream_save;
wire        ss_stream_load;
wire        ss_active;
wire [ 3:0] ss_state_debug;
wire        ss_info_req;
wire [ 7:0] ss_info;
wire        ss_status_set;
wire [ 1:0] ss_slot;
wire [127:0] status_in = {status[127:66], ss_slot, status[63:0]};

wire [ 1:0] dip_fxlevel;
wire        dip_pause, dip_flip, dip_test;
wire [31:0] dipsw;

wire        ioctl_wr;
wire [26:0] ioctl_addr; // up to 128MB
wire [ 7:0] ioctl_dout, ioctl_din;

wire [ 9:0] game_joy1, game_joy2, game_joy3, game_joy4;
wire [ 3:0] game_coin, game_start;
wire [ 3:0] gfx_en;
wire [ 7:0] debug_bus, debug_view;
wire [15:0] joyana_l1, joyana_l2, joyana_l3, joyana_l4,
            joyana_r1, joyana_r2, joyana_r3, joyana_r4;
wire [31:0] joyusb_1_full, joyusb_2_full, joystick3_full, joystick4_full;
wire [15:0] joyusb_1   = joyusb_1_full[15:0];
wire [15:0] joyusb_2   = joyusb_2_full[15:0];
wire [15:0] joystick3  = joystick3_full[15:0];
wire [15:0] joystick4  = joystick4_full[15:0];
wire        ps2_kbd_clk, ps2_kbd_data;
wire [ 7:0] raw_paddle_1, raw_paddle_2, raw_paddle_3, raw_paddle_4;
wire [ 8:0] raw_spinner_1, raw_spinner_2, raw_spinner_3, raw_spinner_4;
wire [24:0] ps2_mouse;

wire        hps_download, hps_upload, hps_wr, hps_rd, hps_wait, sd_wait;
wire [15:0] hps_index;
wire [26:0] hps_addr;
wire [ 7:0] hps_dout;
wire [ 7:0] hps_din;
wire [ 7:0] framework_hps_din, hs_nvram_q;
wire        hs_nvram_wait, hs_dirty, hs_ready, hs_active;
wire        hs_config_selected = hps_index[5:0] == 6'd4;
wire        hs_nvram_selected = hps_index[5:0] == 6'd2;
// JTFrame reserves index 4 for cartridges. FixEight uses the donor high-score
// descriptor at that index, so keep it on the direct HPS side and prevent the
// framework downloader from momentarily taking SDRAM programming ownership
// while the running 68000 is fetching code.
wire        framework_hps_download = hps_download && !hs_config_selected;
wire        framework_hps_wr = hps_wr && !hs_config_selected;
reg         hs_upload_req = 1'b0;
reg         hs_osd_d = 1'b0;
reg         hs_manual_d = 1'b0;
wire [32:0] timestamp_full;
wire [31:0] timestamp = timestamp_full[31:0];

wire        rst_req   = sys_rst | status[0] | buttons[1];
wire signed [15:0] snd_left, snd_right;
wire [ 5:0] snd_en, snd_vu;
wire [ 7:0] snd_vol;
wire        snd_peak;
reg         video_mode_refresh;
reg         dwnld_busy_l;
reg         video_vs_l;
reg  [ 1:0] aspect_l;
reg  [ 1:0] rotate_menu_l;
reg  [ 6:0] video_refresh_frames;
wire [ 1:0] rotate_menu;
wire        framebuf_flip;

fixeight_rotation_status u_rotation_status (
    .status        ( frame_status  ),
    .menu          ( rotate_menu   ),
    .framebuf_flip ( framebuf_flip )
);

assign LED_DISK  = 2'b0;
assign LED_POWER = 2'b0;

// ROM download
wire          ioctl_rom, ioctl_cart, dwnld_busy;

// Let Main recalculate video/subcarrier/aspect parameters once FixEight video
// timing is stable after ROM load, and again when video OSD settings change.
always @(posedge clk_sys) begin
    dwnld_busy_l <= dwnld_busy;
    video_vs_l <= VGA_VS;
    aspect_l <= frame_status[17:16];
    rotate_menu_l <= rotate_menu;

    if (sys_rst) begin
        video_mode_refresh <= 1'b0;
        video_refresh_frames <= 7'd0;
        dwnld_busy_l <= 1'b0;
        video_vs_l <= 1'b0;
        aspect_l <= 2'b00;
        rotate_menu_l <= 2'b00;
    end else begin
        if ((dwnld_busy_l && !dwnld_busy) ||
            (aspect_l != frame_status[17:16]) ||
            (rotate_menu_l != rotate_menu)) begin
            video_refresh_frames <= 7'd60;
        end else if (video_refresh_frames != 7'd0 && !video_vs_l && VGA_VS) begin
            video_refresh_frames <= video_refresh_frames - 7'd1;
            if (video_refresh_frames == 7'd1) begin
                video_mode_refresh <= ~video_mode_refresh;
            end
        end
    end
end

wire [SDRAMW-1:0] prog_addr;
wire [15:0]   prog_data;
wire [ 1:0]   prog_mask, prog_ba;
wire          prog_we, prog_rd, prog_rdy, prog_ack, prog_dst, prog_dok;

// ROM access from game
wire [SDRAMW-1:0] ba0_addr, ba1_addr, ba2_addr, ba3_addr;
wire [ 3:0] ba_rd, ba_rdy, ba_ack, ba_dst, ba_dok;
wire        ba_wr;
wire [ 3:0] ba_wr_bus = {3'b000, ba_wr};
wire [15:0] ba0_din, ba1_din, ba2_din, ba3_din;
wire [ 1:0] ba0_dsn, ba1_dsn, ba2_dsn, ba3_dsn;
`ifdef JTFRAME_SDRAM_CACHE
wire [SDRAMW-1:0] burst_addr;
wire [ 1:0] burst_ba;
wire        burst_rd, burst_wr, burst_ack, burst_dst, burst_dok, burst_rdy;
wire [15:0] burst_din;
`endif
wire [15:0] sdram_dout;

wire [ 7:0] st_addr, st_dout;
wire [ 7:0] paddle_1,  paddle_2,  paddle_3,  paddle_4;
wire [ 8:0] spinner_1, spinner_2, spinner_3, spinner_4;
wire [15:0] mouse_1p, mouse_2p;
wire [ 1:0] mouse_strobe;
wire [ 8:0] gun_1p_x, gun_1p_y, gun_2p_x, gun_2p_y;

`ifdef JTFRAME_BUTTONS
localparam CORE_BUTTONS = `JTFRAME_BUTTONS;
`else
localparam CORE_BUTTONS = 2;
`endif

localparam CORE_GAME_INPUTS_ACTIVE_LOW = 1'b1;

`ifdef VIDEO_WIDTH
localparam CORE_VIDEO_WIDTH = `VIDEO_WIDTH;
`else
localparam CORE_VIDEO_WIDTH = 384;
`endif

`ifdef VIDEO_HEIGHT
localparam CORE_VIDEO_HEIGHT = `VIDEO_HEIGHT;
`else
localparam CORE_VIDEO_HEIGHT = 240;
`endif

localparam COLORW=`JTFRAME_COLORW;

wire [COLORW-1:0] red, green, blue;
wire              LHBL, LVBL;
wire              hs, vs, sample;
wire              ioctl_ram;
wire              game_rx, game_tx;

`ifndef JTFRAME_UART
    assign game_tx = 1;
`endif

`ifndef JTFRAME_SIGNED_SND
assign AUDIO_S = 1'b1; // Assume signed by default
`else
assign AUDIO_S = `JTFRAME_SIGNED_SND;
`endif

// Line-Frame buffer
wire [ 8:0] game_hdump,   ln_addr;
wire [ 7:0] game_vrender, ln_v;
wire        ln_done, ln_hs, ln_vs, ln_lvbl, ln_we;
wire [15:0] ln_dout, ln_pxl, ln_data;
hps_io #(
    .CONF_STR ( CONF_STR            ),
    .PS2DIV   ( 32                  ),
    .WIDE     ( 0                   ),
    .BLKSZ    ( 1                   )
) hps_io (
    .clk_sys         ( clk_rom         ),
    .HPS_BUS         ( HPS_BUS         ),

    .buttons         ( buttons         ),
    .status          ( status          ),
    .status_in       ( status_in       ),
    .status_set      ( ss_status_set   ),
    .status_menumask ( status_menumask ),
    .gamma_bus       ( gamma_bus       ),
    .direct_video    ( direct_video    ),
    .forced_scandoubler(force_scan2x   ),
    .video_rotated   ( video_rotated   ),
    .new_vmode       ( video_mode_refresh ),

    .ioctl_download  ( hps_download    ),
    .ioctl_wr        ( hps_wr          ),
    .ioctl_addr      ( hps_addr        ),
    .ioctl_dout      ( hps_dout        ),
    .ioctl_din       ( hps_din         ),
    .ioctl_index     ( hps_index       ),
    .ioctl_wait      ( hps_wait | sd_wait | hs_nvram_wait ),
    .ioctl_upload    ( hps_upload      ),
    .ioctl_upload_req( hs_upload_req   ),
    .ioctl_upload_index( 8'd2          ),
    .ioctl_rd        ( hps_rd          ),

    .joystick_0      ( joyusb_1_full   ),
    .joystick_1      ( joyusb_2_full   ),
    .joystick_2      ( joystick3_full  ),
    .joystick_3      ( joystick4_full  ),
    .joystick_l_analog_0( joyana_l1    ),
    .joystick_l_analog_1( joyana_l2    ),
    .joystick_l_analog_2( joyana_l3    ),
    .joystick_l_analog_3( joyana_l4    ),
    .joystick_r_analog_0( joyana_r1    ),
    .joystick_r_analog_1( joyana_r2    ),
    .joystick_r_analog_2( joyana_r3    ),
    .joystick_r_analog_3( joyana_r4    ),
    .ps2_kbd_clk_out ( ps2_kbd_clk     ),
    .ps2_kbd_data_out( ps2_kbd_data    ),
    .ps2_key         ( ps2_key         ),

    .paddle_0        ( raw_paddle_1    ),
    .paddle_1        ( raw_paddle_2    ),
    .paddle_2        ( raw_paddle_3    ),
    .paddle_3        ( raw_paddle_4    ),
    .spinner_0       ( raw_spinner_1   ),
    .spinner_1       ( raw_spinner_2   ),
    .spinner_2       ( raw_spinner_3   ),
    .spinner_3       ( raw_spinner_4   ),

    .TIMESTAMP       ( timestamp_full  ),
    .ps2_mouse       ( ps2_mouse       ),
    .info_req        ( ss_info_req     ),
    .info            ( ss_info         ),
    .EXT_BUS         (                 )
);

// Main emits a 1-then-0 pulse for T/R rows. Save once on its rising edge.
// Autosave follows the donor convention: opening the OSD with dirty scores.
always @(posedge clk_rom) begin
    hs_osd_d <= OSD_STATUS;
    hs_manual_d <= status[70];
    hs_upload_req <= hs_ready && !hs_active && !ss_active && !dwnld_busy &&
        ((status[69] && hs_dirty && OSD_STATUS && !hs_osd_d) ||
         (status[70] && !hs_manual_d));
    if (rst) begin
        hs_upload_req <= 0;
        hs_osd_d <= 0;
        hs_manual_d <= 0;
    end
end

wire [31:0] ss_joystick =
    joyusb_1_full | joyusb_2_full | joystick3_full | joystick4_full;

savestate_ui #(.INFO_TIMEOUT_BITS(25)) u_savestate_ui (
    .clk            ( clk_rom          ),
    .ps2_key        ( ps2_key          ),
    .allow_ss       ( !rst && !dwnld_busy && !ss_active && !hs_active ),
    .joySS          ( ss_joystick[13]  ),
    .joyRight       ( ss_joystick[0]   ),
    .joyLeft        ( ss_joystick[1]   ),
    .joyDown        ( ss_joystick[2]   ),
    .joyUp          ( ss_joystick[3]   ),
    .joyStart       ( 1'b0             ),
    .joyRewind      ( 1'b0             ),
    .rewindEnable   ( 1'b0             ),
    .status_slot    ( status[65:64]    ),
    .autoincslot    ( status[66]       ),
    .OSD_saveload   ( status[68:67]    ),
    .ss_save        ( ss_save          ),
    .ss_load        ( ss_load          ),
    .ss_info_req    ( ss_info_req      ),
    .ss_info        ( ss_info          ),
    .statusUpdate   ( ss_status_set    ),
    .selected_slot  ( ss_slot          )
);

ddr_if ddr_ss();
ssbus_if ssbus();
ssbus_if ssb[2]();

ssbus_mux #(.COUNT(2)) u_ssbus_mux (
    .clk     ( clk_rom ),
    .slave   ( ssbus   ),
    .masters ( ssb     )
);

save_state_data u_save_state_data (
    .clk         ( clk_rom ),
    .reset       ( rst     ),
    .ddr         ( ddr_ss  ),
    .read_start  ( ss_stream_load ),
    .write_start ( ss_stream_save ),
    .index       ( ss_slot ),
    .busy        ( ss_busy ),
    .ssbus       ( ssbus   )
);

localparam [63:0] SS_MAGIC   = 64'h4638_5353_3030_3031; // "F8SS0001"
localparam [63:0] SS_VERSION = 64'h0001_0000_0000_0000;

reg [63:0] ss_restored_magic = 64'd0;
reg [63:0] ss_restored_version = 64'd0;
wire ss_format_valid = (ss_restored_magic == SS_MAGIC) &&
                       (ss_restored_version == SS_VERSION);

// Save format metadata. This is deliberately the first chunk so all later
// restore writers can remain disabled until the schema has been validated.
always_ff @(posedge clk_rom) begin
    ssb[0].setup(0, 2, 3);

    if (rst || ss_stream_load) begin
        ss_restored_magic <= 64'd0;
        ss_restored_version <= 64'd0;
    end

    if (ssb[0].access(0)) begin
        if (ssb[0].read) begin
            ssb[0].read_response(
                0,
                ssb[0].addr[0] ? SS_VERSION : SS_MAGIC
            );
        end else if (ssb[0].write) begin
            if (ssb[0].addr[0])
                ss_restored_version <= ssb[0].data;
            else
                ss_restored_magic <= ssb[0].data;
            ssb[0].write_ack(0);
        end
    end
end

`include "rtl/fixeight/fixeight_game_instance.vh"

// Lower MiSTer/JTFrame integration. The root emu owns hps_io/CONF_STR.
wire [63:0] frame_status = status[63:0];
wire [ 7:0] ioctl_index;

wire [3*COLORW-1:0] base_rgb;
wire        base_lhbl, base_lvbl, base_hs, base_vs;

wire [ 3:0] hoffset, voffset;
wire [31:0] cheat;
wire        ioctl_cheat, ioctl_lock;

wire [15:0] joystick1, joystick2;
wire        lightgun_en;

wire [ 6:0] core_mod;
wire [ 7:0] st_lpbuf, game_vol;
// Mouse support
reg         ps2_mouse_l;
wire [ 8:0] mouse_dx, mouse_dy;
wire [ 7:0] mouse_f;
wire        mouse_st;

wire        hs_resync, vs_resync;
localparam [4:0] CRT_HOFFSET_DEFAULT_BIAS = 5'b1_1000; // -8, hidden from the OSD offset menu
wire       [4:0] hoffset_biased = {hoffset[3], hoffset} + CRT_HOFFSET_DEFAULT_BIAS;

// Horizontal scaling for CRT
wire        hsize_enable;
wire [3:0]  hsize_scale;
wire        hsize_hs, hsize_vs, hsize_hb, hsize_vb;
wire [COLORW-1:0] hsize_r, hsize_g, hsize_b;

// Screen rotation
wire [ 7:0] rot_burstcnt;
wire [28:0] rot_addr;
wire [63:0] rot_din;
wire        rot_we, rot_rd, rot_busy;
wire [ 7:0] rot_be;

// Fast DDR load
wire [ 7:0] ddrld_burstcnt;
wire [28:0] ddrld_addr;
wire        ddrld_rd, ddrld_busy;

// UART
wire        uart_rx, uart_tx;
wire [6:0]  joy_in, joy_out;

// Vertical crop
wire [12:0] raw_arx, raw_ary;
wire        raw_de;
reg  [11:0] crop_size;
reg   [4:0] crop_off; // -16...+15
wire  [2:0] crop_scale; //0 - normal, 1 - V-integer, 2 - HV-Integer-, 3 - HV-Integer+, 4 - HV-Integer
wire        crop_en;    // OSD control by the user
reg         crop_ok;    // whether the mister.ini video settings tolerate cropping
wire  [3:0] vcopt;
reg         en216p;
reg   [4:0] voff;
reg         pxl1_cen;

// Save/Load
wire [ 1:0] ram_save;
wire        ram_load;

wire  [7:0] target_info;

assign paddle_3  = raw_paddle_3;
assign paddle_4  = raw_paddle_4;
assign spinner_1 = raw_spinner_1;
assign spinner_2 = raw_spinner_2;
assign spinner_3 = raw_spinner_3;
assign spinner_4 = raw_spinner_4;

`ifdef MISTER_FB
assign FB_FORCE_BLANK = 1'b0;
`ifdef MISTER_FB_PALETTE
// The framebuffer palette interface is unused, but ascal still needs
// a live clock when the current framework exposes these ports.
assign FB_PAL_CLK  = clk_sys;
assign FB_PAL_ADDR = 8'd0;
assign FB_PAL_DOUT = 24'd0;
assign FB_PAL_WR   = 1'b0;
`endif
`endif

// UART
// The core and cheat UARTs are connected in parallel
// If JTFRAME_UART is not defined, the core side is disabled
// If JTFRAME_CHEAT is not defined, the cheat side is disabled
// Otherwise, both can listen and talk
always @(posedge clk_sys) begin
    USER_OUT <= db15_en ? joy_out :
        uart_en ? {~6'h0, uart_tx&game_tx } :
        7'h7f;
end

assign uart_rx  = uart_en ? USER_IN[1] : 1'b1;
assign game_rx  = uart_rx;
assign joy_in   = USER_IN;

// Mouse
assign mouse_st = ps2_mouse[24]^ps2_mouse_l;
assign mouse_f  = ps2_mouse[7:0];
assign mouse_dx = { mouse_f[4], ps2_mouse[15: 8] };
assign mouse_dy = { mouse_f[5], ps2_mouse[23:16] };

always @(posedge clk_sys)
    ps2_mouse_l <= ps2_mouse[24];

jtframe_mister_status u_status(
    .status         ( frame_status         ),
    .crop_en        ( crop_en        ),
    .vcopt          ( vcopt          ),
    .crop_scale     ( crop_scale     ),
    .voffset        ( voffset        ),
    .hoffset        ( hoffset        ),
    .hsize_enable   ( hsize_enable   ),
    .hsize_scale    ( hsize_scale    ),
    .ram_save       ( ram_save       ),
    .ram_load       ( ram_load       ),
    .gun_border_en  ( gun_border_en  ),
    .uart_en        ( uart_en        )
);

jtframe_target_info u_target_info(
    .clk            ( clk_sys        ),
    .joyana_l1      ( joyana_l1      ),
    .joyana_r1      ( joyana_r1      ),
    .joystick1      ( joystick1      ),
    .joystick2      ( joystick2      ),
    .mouse_1p       ( mouse_1p       ),
    .mouse_2p       ( mouse_2p       ),
    .hps_index      ( hps_index      ),
    .spinner_1      ( raw_spinner_1  ),
    .spinner_2      ( raw_spinner_2  ),
    .spinner_3      ( raw_spinner_3  ),
    .spinner_4      ( raw_spinner_4  ),
    .game_paddle_1  ( paddle_1       ),
    .game_paddle_2  ( paddle_2       ),
    .dial_x         ( dial_x         ),
    .dial_y         ( dial_y         ),
    .st_lpbuf       ( st_lpbuf       ),
    .ioctl_lock     ( ioctl_lock     ),
    .ioctl_cart     ( ioctl_cart     ),
    .ioctl_ram      ( ioctl_ram      ),
    .ioctl_rom      ( ioctl_rom      ),
    .ioctl_wr       ( ioctl_wr       ),
    .dwnld_busy     ( dwnld_busy     ),
    .hps_download   ( hps_download   ),
    .debug_bus      ( debug_bus      ),
    .target_info    ( target_info    )
);

fixeight_resync u_resync(
    .clk        ( clk_sys       ),
    .pxl_cen    ( pxl1_cen      ),
    .hs_in      ( hs            ),
    .vs_in      ( vs            ),
    .LVBL       ( LVBL          ),
    .LHBL       ( LHBL          ),
    .hoffset    ( hoffset_biased ),
    .voffset    ( voffset       ),
    .hs_out     ( hs_resync     ),
    .vs_out     ( vs_resync     )
);

// OSD option visibility. This is linked to the d(D) and h(H) prefixes in cfgstr
wire        vertical;

assign status_menumask[15:11] = 0,
       status_menumask[10]   = !hs_ready || hs_active || ss_active,
       status_menumask[9]    = ~crop_ok,
       status_menumask[8]    = ~rotate[0],
       status_menumask[7]    = ~crop_ok | ~crop_en,
       status_menumask[6]    = ~lightgun_en, // sinden lightgun borders
       status_menumask[5]    = rotate[0], // native versus rotated crop row
`ifdef JTFRAME_ROTATE       // extra rotate options for vertical games
       status_menumask[4]    = ~vertical,  // shown for vertical games
       status_menumask[1]    = 1,   // hidden
`else
       status_menumask[4]    = 1,   // hidden
       status_menumask[1]    = ~vertical,  // shown for vertical games
`endif
       status_menumask[3]    = video_rotated, // scanlines unavailable after rotation
       status_menumask[2]    = ~hsize_enable, // horizontal scaling
       status_menumask[0]    = direct_video;

// this places the pxl1_cen in the pixel centre
always @(posedge clk_sys) pxl1_cen <= pxl2_cen & ~pxl_cen;


jtframe_mister_dwnld u_dwnld(
    .rst            ( rst            ),
    .clk            ( clk_rom        ),

    .prog_we        ( prog_we        ),
    .prog_rdy       ( prog_rdy       ),
    .ioctl_rom      ( ioctl_rom      ),
    .dwnld_busy     ( dwnld_busy     ),

    .hps_download   ( framework_hps_download ),
    .hps_upload     ( hps_upload     ),
    .hps_index      ( hps_index[7:0] ),
    .hps_wr         ( framework_hps_wr ),
    .hps_addr       ( hps_addr       ),
    .hps_dout       ( hps_dout      ),
    .hps_wait       ( hps_wait       ),

    .ioctl_wr       ( ioctl_wr       ),
    .ioctl_addr     ( ioctl_addr     ),
    .ioctl_dout     ( ioctl_dout     ),
    .ioctl_ram      ( ioctl_ram      ),
    .ioctl_cart     ( ioctl_cart     ),
    .ioctl_cheat    ( ioctl_cheat    ),
    .ioctl_lock     ( ioctl_lock     ),

    // Configuration
    .core_mod       ( core_mod       ),
    .game_vol       ( game_vol       ),
    .status         ( frame_status[31:0] ),
    .dipsw          ( dipsw          ),
    .cheat          ( cheat          ),

    // DDR
    .ddram_busy     ( ddrld_busy       ),
    .ddram_burstcnt ( ddrld_burstcnt   ),
    .ddram_addr     ( ddrld_addr       ),
    .ddram_dout     ( DDRAM_DOUT       ),
    .ddram_dout_ready(DDRAM_DOUT_READY ),
    .ddram_rd       ( ddrld_rd         )
);

`ifndef JTFRAME_NO_DB15
assign db15_en  = frame_status[37];
jtframe_joymux #(.BUTTONS(CORE_BUTTONS)) u_joymux(
    .rst        ( rst       ),
    .clk        ( clk_sys   ),
    .show_osd   ( show_osd  ),

    // MiSTer pins
    .USER_IN    ( joy_in    ),
    .USER_OUT   ( joy_out   ),

    // joystick mux
    .db15_en    ( db15_en   ),
    .joyusb_1   ( joyusb_1  ),
    .joyusb_2   ( joyusb_2  ),
    .joymux_1   ( joystick1 ),
    .joymux_2   ( joystick2 )
);
`else
assign db15_en   = 0;
assign show_osd  = 0;
assign joystick1 = joyusb_1;
assign joystick2 = joyusb_2;
`endif

`ifdef JTFRAME_SHADOW
    jtframe_shadow #(
        .AW    ( SDRAMW              ),
        .START ( `JTFRAME_SHADOW     ),
        .LW    ( `JTFRAME_SHADOW_LEN )  // length of data to be dumped as a power of 2
    ) u_shadow (
        .clk_rom    ( clk_rom       ),

        // Capture SDRAM bank 0 inputs
        .ba0_addr   ( ba0_addr      ),
        .wr0        ( ba_wr         ),
        .din        ( ba0_din       ),
        .din_m      ( ba0_dsn       ),  // write mask -active low

        // Let data be dumped via NVRAM interface
        .ioctl_addr ( ioctl_addr    ),
        .ioctl_din  ( framework_hps_din )
    );
`else
    assign framework_hps_din = ioctl_din;
`endif

assign hps_din = (hps_upload && hs_nvram_selected) ? hs_nvram_q : framework_hps_din;

`ifdef JTFRAME_SAVEGAME
wire [31:0] sd_lba;
reg  [ 7:0] sd_buff_din;
wire [ 7:0] sd_buff_addr, sd_buff_dout;
wire        bk_ena, sd_ack, sd_wr, sd_rd, sd_buff_wr;
wire [63:0] img_size;
wire        img_mounted, img_readonly;

jtframe_mister_cartsave u_save(
    .clk         ( clk_sys      ),
    .OSD_STATUS  ( OSD_STATUS   ),
    .io_strobe   ( HPS_BUS[33]  ),
    .img_size    ( img_size     ),
    .img_mounted ( img_mounted  ),
    .img_readonly( img_readonly ),
    .ram_save    ( ram_save     ),
    .ram_load    ( ram_load     ),
    .downloading ( ioctl_cart   ),
    .sd_buff_addr( sd_buff_addr ),
    .sd_buff_dout( sd_buff_dout ),
    .sd_buff_din ( sd_buff_din  ),
    .sd_buff_wr  ( sd_buff_wr   ),
    .sd_ack      ( sd_ack       ),
    .sd_rd       ( sd_rd        ),
    .sd_wr       ( sd_wr        ),
    .bk_ena      ( bk_ena       ),
    .sd_lba      ( sd_lba       ),
    .sd_wait     ( sd_wait      ),
    .sav_change  ( sav_change   ),
    .sav_wait    ( sav_wait     ),
    .sav_done    ( sav_done     ),
    .sav_din     ( sav_din      ),
    .sav_dout    ( sav_dout     ),
    .sav_addr    ( sav_addr     ),
    .sav_ack     ( sav_ack      ),
    .sav_wr      ( sav_wr       )
);
`else
assign {sav_addr, sav_dout, sav_wr, sav_ack, sd_wait} = 0;
`endif

`ifndef DEBUG_NOHDMI
    // scales base video horizontally
    jtframe_hsize #(.COLORW(COLORW)) u_hsize(
        .clk        ( clk_sys   ),
        .pxl_cen    ( pxl1_cen  ),
        .pxl2_cen   ( pxl2_cen  ),

        .scale      ( hsize_scale  ),
        .offset     ( 5'd0         ),
        .enable     ( hsize_enable ),

        .r_in       ( red       ),
        .g_in       ( green     ),
        .b_in       ( blue      ),
        .HS_in      ( hs_resync ),
        .VS_in      ( vs_resync ),
        .HB_in      ( ~LHBL     ),
        .VB_in      ( ~LVBL     ),
        // filtered video
        .HS_out     ( hsize_hs  ),
        .VS_out     ( hsize_vs  ),
        .HB_out     ( hsize_hb  ),
        .VB_out     ( hsize_vb  ),
        .r_out      ( hsize_r   ),
        .g_out      ( hsize_g   ),
        .b_out      ( hsize_b   )
    );
`else
    assign hsize_hs = hs_resync;
    assign hsize_vs = vs_resync;
    assign hsize_hb = ~LHBL;
    assign hsize_vb = ~LVBL;
    assign hsize_r  = red;
    assign hsize_g  = green;
    assign hsize_b  = blue;
`endif

localparam VIDEO_DW = COLORW!=5 ? 3*COLORW : 24;

wire [VIDEO_DW-1:0] game_rgb;
wire [COLORW-1:0] base_r, base_g, base_b;

assign {base_r,base_g,base_b} = base_rgb;

// arcade video does not support 15bpp colour, so for that
// case we need to convert it to 24bpp
generate
    if( COLORW!=5 ) begin
        assign game_rgb = base_rgb;
    end else begin
        assign game_rgb = {
            base_r, base_r[4:2],
            base_g, base_g[4:2],
            base_b, base_b[4:2]   };
    end
endgenerate

wire [23:0] video_rgb = game_rgb;

// VIDEO_WIDTH does not include blanking:
arcade_video #(.WIDTH(CORE_VIDEO_WIDTH),.DW(VIDEO_DW))
u_arcade_video(
    .clk_video  ( clk_sys       ),
    .ce_pix     ( pxl_cen       ),

    .RGB_in     ( video_rgb     ),
    .HBlank     ( ~base_lhbl    ),
    .VBlank     ( ~base_lvbl    ),
    .HSync      ( hsize_hs      ),
    .VSync      ( hsize_vs      ),

    .CLK_VIDEO  ( CLK_VIDEO    ),
    .CE_PIXEL   ( CE_PIXEL    ),
    .VGA_R      ( VGA_R      ),
    .VGA_G      ( VGA_G      ),
    .VGA_B      ( VGA_B      ),
    .VGA_HS     ( VGA_HS     ),
    .VGA_VS     ( VGA_VS     ),
    .VGA_DE     ( raw_de        ),
    .VGA_SL     ( VGA_SL     ),

    .gamma_bus  ( gamma_bus     ),
    .fx         ( video_fx          ),
    .forced_scandoubler( force_scan2x )
);

jtframe_board #(
    .BUTTONS               ( CORE_BUTTONS         ),
    .GAME_INPUTS_ACTIVE_LOW(CORE_GAME_INPUTS_ACTIVE_LOW),
    .COLORW                ( COLORW               ),
    .VIDEO_WIDTH           ( CORE_VIDEO_WIDTH     ),
    .VIDEO_HEIGHT          ( CORE_VIDEO_HEIGHT    ),
    .SDRAMW                ( SDRAMW               ),
    .MISTER                ( 1                    )
) u_board(
    .rst            ( rst             ),
    .rst_n          ( rst_n           ),
    .game_rst       ( game_rst        ),
    .game_rst_n     ( game_rst_n      ),
    .rst_req        ( rst_req         ),
    .sdram_init     (                 ),

    .pll_locked     ( pll_locked      ),

    .ioctl_cart     ( ioctl_cart      ),
    .ioctl_ram      ( ioctl_ram       ),
    .dwnld_busy     ( dwnld_busy      ),

    .clk_sys        ( clk_sys         ),
    .clk_rom        ( clk_rom         ),
    .clk_pico       ( clk_pico        ),

    .core_mod       ( core_mod        ),
    .game_vol       ( game_vol        ),
    .vertical       ( vertical        ),
    .black_frame    (                 ),
    // Sound
    .snd_lin        ( snd_left       ),
    .snd_rin        ( snd_right      ),
    .snd_lout       ( AUDIO_L        ),
    .snd_rout       ( AUDIO_R        ),
    .snd_sample     ( sample         ),
    .snd_en         ( snd_en          ),
    .snd_vu         ( snd_vu          ),
    .snd_vol        ( snd_vol         ),
    .snd_peak       ( snd_peak        ),
    // joystick
    .ps2_kbd_clk    ( ps2_kbd_clk     ),
    .ps2_kbd_data   ( ps2_kbd_data    ),
    .board_joystick1( joystick1       ),
    .board_joystick2( joystick2       ),
    .board_joystick3( joystick3       ),
    .board_joystick4( joystick4       ),
    .joyana_l1      ( joyana_l1       ),
    .joyana_r1      ( joyana_r1       ),
    .joyana_l2      ( joyana_l2       ),
    .joyana_r2      ( joyana_r2       ),
    .board_start    ( 4'd0            ),
    .board_coin     ( 4'd0            ),
    .game_joystick1 ( game_joy1      ),
    .game_joystick2 ( game_joy2      ),
    .game_joystick3 ( game_joy3      ),
    .game_joystick4 ( game_joy4      ),
    .game_coin      ( game_coin       ),
    .game_start     ( game_start      ),
    .game_service   ( game_service    ),
    .game_tilt      ( game_tilt       ),
    // Mouse & paddle
    .bd_mouse_dx    ( mouse_dx        ),
    .bd_mouse_dy    ( mouse_dy        ),
    .bd_mouse_st    ( mouse_st        ),
    .bd_mouse_f     ( mouse_f         ),
    .bd_mouse_idx   ( 1'b0            ),    // MiSTer only supports one mouse

    .board_paddle_1 ( raw_paddle_1    ),
    .board_paddle_2 ( raw_paddle_2    ),
    .game_paddle_1  ( paddle_1       ),
    .game_paddle_2  ( paddle_2       ),
    .mouse_1p       ( mouse_1p        ),
    .mouse_2p       ( mouse_2p        ),
    .mouse_strobe   ( mouse_strobe    ),
    .spinner_1      ( raw_spinner_1   ),
    .spinner_2      ( raw_spinner_2   ),
    .dial_x         ( dial_x          ),
    .dial_y         ( dial_y          ),
    // Lightguns
    .gun_1p_x       ( gun_1p_x        ),
    .gun_1p_y       ( gun_1p_y        ),
    .gun_2p_x       ( gun_2p_x        ),
    .gun_2p_y       ( gun_2p_y        ),
    .lightgun_en    ( lightgun_en     ),
    // DIP and OSD settings
    .status         ( frame_status    ),
    .dipsw          ( dipsw           ),
    .dip_test       ( dip_test        ),
    .dip_pause      ( dip_pause       ),
    .dip_flip       ( dip_flip        ),
    .dip_fxlevel    ( dip_fxlevel     ),
    .timestamp      ( timestamp       ),
    // screen
    .hdmi_arx       ( raw_arx         ),
    .hdmi_ary       ( raw_ary         ),
    .rotate         ( rotate          ),
    .rot_osdonly    (                 ),
    // LED_USER
    .osd_shown      ( 1'b0            ),
    .led            ( LED_USER             ),
    // UART
    .uart_rx        ( uart_rx         ),
    .uart_tx        ( uart_tx         ),

    // SDRAM interface
    // Bank 0: allows R/W
    .ba0_addr   ( ba0_addr      ),
    .ba1_addr   ( ba1_addr      ),
    .ba2_addr   ( ba2_addr      ),
    .ba3_addr   ( ba3_addr      ),
`ifdef JTFRAME_SDRAM_CACHE
    .burst_addr ( burst_addr    ),
    .burst_ba   ( burst_ba      ),
    .burst_rd   ( burst_rd      ),
    .burst_wr   ( burst_wr      ),
    .burst_ack  ( burst_ack     ),
    .burst_dst  ( burst_dst     ),
    .burst_dok  ( burst_dok     ),
    .burst_rdy  ( burst_rdy     ),
`endif
    .ba_rd      ( ba_rd         ),
    .ba_wr      ( ba_wr_bus     ),
    .ba_dst     ( ba_dst        ),
    .ba_dok     ( ba_dok        ),
    .ba_rdy     ( ba_rdy        ),
    .ba_ack     ( ba_ack        ),
    .ba0_din    ( ba0_din       ),
    .ba0_dsn    ( ba0_dsn       ),
    .ba1_din    ( ba1_din       ),
    .ba1_dsn    ( ba1_dsn       ),
    .ba2_din    ( ba2_din       ),
    .ba2_dsn    ( ba2_dsn       ),
    .ba3_din    ( ba3_din       ),
    .ba3_dsn    ( ba3_dsn       ),
`ifdef JTFRAME_SDRAM_CACHE
    .burst_din  ( burst_din     ),
`endif

    // ROM-load interface
    .prog_addr  ( prog_addr     ),
    .prog_ba    ( prog_ba       ),
    .prog_rd    ( prog_rd       ),
    .prog_we    ( prog_we       ),
    .prog_data  ( prog_data     ),
    .prog_dsn   ( prog_mask     ),
    .prog_rdy   ( prog_rdy      ),
    .prog_dst   ( prog_dst      ),
    .prog_dok   ( prog_dok      ),
    .prog_ack   ( prog_ack      ),
    // SDRAM interface
    .SDRAM_DQ   ( SDRAM_DQ      ),
    .SDRAM_A    ( SDRAM_A       ),
    .SDRAM_DQML ( SDRAM_DQML    ),
    .SDRAM_DQMH ( SDRAM_DQMH    ),
    .SDRAM_nWE  ( SDRAM_nWE     ),
    .SDRAM_nCAS ( SDRAM_nCAS    ),
    .SDRAM_nRAS ( SDRAM_nRAS    ),
    .SDRAM_nCS  ( SDRAM_nCS     ),
    .SDRAM_BA   ( SDRAM_BA      ),
    .SDRAM_CKE  ( SDRAM_CKE     ),

    // Common signals
    .sdram_dout ( sdram_dout    ),

    // Cheat!
    .cheat          ( cheat           ),
    .prog_cheat     ( ioctl_cheat     ),
    .prog_lock      ( ioctl_lock      ),
    .ioctl_wr       ( hps_wr          ),
    .ioctl_dout     ( ioctl_dout      ),
    .ioctl_addr     ( ioctl_addr[7:0] ),
    .st_addr        ( st_addr         ),
    .st_dout        ( st_dout         ),
    .target_info    ( target_info     ),

    // input data recording
    .ioctl_din      (                 ),
    .ioctl_merged   (                 ),
    // input video
    .osd_rotate     ( rotate          ),
    .game_r         ( hsize_r         ),
    .game_g         ( hsize_g         ),
    .game_b         ( hsize_b         ),
    .LHBL           ( ~hsize_hb       ),
    .LVBL           ( ~hsize_vb       ),
    .hs             ( hsize_hs        ),
    .vs             ( hsize_vs        ),
    .pxl_cen        ( pxl_cen         ),
    .pxl2_cen       ( pxl2_cen        ),
    // output video with credits and debug informaiton
    .base_lhbl      ( base_lhbl         ),
    .base_lvbl      ( base_lvbl         ),
    .base_hs        ( base_hs           ), // Disconnected
    .base_vs        ( base_vs           ), // Disconnected
    .base_rgb       ( base_rgb          ),
    // Debug
    .gfx_en         ( gfx_en          ),
    .debug_bus      ( debug_bus       ),
    .debug_view     ( debug_view      )
);

wire [2:0] video_fx = frame_status[3] ? 3'd1 :
                      frame_status[5:4] == 2'd1 ? 3'd2 :
                      frame_status[5:4] == 2'd2 ? 3'd3 :
                      frame_status[5:4] == 2'd3 ? 3'd4 : 3'd0;
always @(posedge CLK_VIDEO) begin
    crop_ok <= HDMI_WIDTH == 1920 && HDMI_HEIGHT == 1080 && video_fx == 0 &&
               !force_scan2x && crop_scale == 0 && !direct_video;
    case (vcopt)
        0: crop_off <= 5'd0;
        1: crop_off <= 5'd2;
        2: crop_off <= 5'd4;
        3: crop_off <= 5'd8;
        4: crop_off <= 5'd10;
        5: crop_off <= 5'd12;
        6: crop_off <= -5'sd12;
        7: crop_off <= -5'sd10;
        8: crop_off <= -5'sd8;
        9: crop_off <= -5'sd6;
        10: crop_off <= -5'sd4;
        default: crop_off <= -5'sd2;
    endcase
    crop_size <= (crop_ok && crop_en && !rotate[0]) ? 12'd216 : 12'd0;
end

wire [12:0] crop_arx;
wire [12:0] crop_ary;
wire        original_aspect = frame_status[17:16] == 2'b00;
wire        rotated_crop_active;
wire        rotated_original_aspect = rotate[0] && original_aspect && crop_scale == 0 && (HDMI_HEIGHT != 12'd0);
wire [11:0] rotated_aspect_width = HDMI_HEIGHT - (HDMI_HEIGHT >> 2);

video_freak u_crop(
    .CLK_VIDEO  ( CLK_VIDEO    ),
    .CE_PIXEL   ( CE_PIXEL    ),
    .VGA_VS     ( VGA_VS     ),
    .HDMI_WIDTH ( HDMI_WIDTH    ),
    .HDMI_HEIGHT( HDMI_HEIGHT   ),
    .VGA_DE     ( VGA_DE     ),
    .VIDEO_ARX  ( crop_arx      ),
    .VIDEO_ARY  ( crop_ary      ),

    .VGA_DE_IN  ( raw_de        ),
    .ARX        ( raw_arx       ),
    .ARY        ( raw_ary       ),
    .CROP_SIZE  ( crop_size     ),
    .CROP_OFF   ( crop_off      ),
    .SCALE      ( crop_scale    )
);

assign VIDEO_ARX = (rotated_crop_active && original_aspect) ? {1'b1, 12'd960} :
                   rotated_original_aspect ? {1'b1, rotated_aspect_width} : crop_arx;
assign VIDEO_ARY = (rotated_crop_active && original_aspect) ? {1'b1, 12'd1080} :
                   rotated_original_aspect ? {1'b1, HDMI_HEIGHT} : crop_ary;


wire rot_clk;

`ifdef JTFRAME_VERTICAL
    wire [11:0] rot_fb_width, rot_fb_height;
    wire [31:0] rot_fb_base;
    screen_rotate u_rotate(
        .CLK_VIDEO      ( CLK_VIDEO     ),
        .CE_PIXEL       ( CE_PIXEL     ),

        .VGA_R          ( VGA_R       ),
        .VGA_G          ( VGA_G       ),
        .VGA_B          ( VGA_B       ),
        .VGA_HS         ( VGA_HS      ),
        .VGA_VS         ( VGA_VS      ),
        .VGA_DE         ( VGA_DE      ),

        .rotate_ccw     (  rotate[1]     ),
        .no_rotate      ( ~rotate[0]     ),
        .flip           ( framebuf_flip  ),
        .video_rotated  ( video_rotated  ),

        .FB_EN          ( FB_EN          ),
        .FB_FORMAT      ( FB_FORMAT      ),
        .FB_WIDTH       ( rot_fb_width   ),
        .FB_HEIGHT      ( rot_fb_height  ),
        .FB_BASE        ( rot_fb_base    ),
        .FB_STRIDE      ( FB_STRIDE      ),
        .FB_VBL         ( FB_VBL         ),
        .FB_LL          ( FB_LL          ),

        //.debug_bus      ( debug_bus      ),

        //muxed
        .DDRAM_BUSY     ( rot_busy       ),
        .DDRAM_BURSTCNT ( rot_burstcnt   ),
        .DDRAM_ADDR     ( rot_addr       ),
        .DDRAM_BE       ( rot_be         ),
        .DDRAM_WE       ( rot_we         ),
        .DDRAM_RD       ( rot_rd         ),
        // umuxed
        .DDRAM_CLK      ( rot_clk        ), // same as clk_rom
        .DDRAM_DIN      ( rot_din        )
    );
    fixeight_rotated_crop u_rotated_crop (
        .enable(crop_ok && crop_en && rotate[0] && FB_EN), .offset(crop_off),
        .width_in(rot_fb_width), .height_in(rot_fb_height),
        .base_in(rot_fb_base), .stride_in(FB_STRIDE),
        .active(rotated_crop_active), .width_out(FB_WIDTH),
        .height_out(FB_HEIGHT), .base_out(FB_BASE)
    );
`else
    assign rotated_crop_active = 1'b0;
    `ifndef JTFRAME_LF_BUFFER assign rot_din=64'd0; `endif
    assign rot_clk = clk_rom;
`endif

`ifdef JTFRAME_LF_BUFFER
    // line-frame buffer. This won't work with fast DDR load or vertical games
    jtframe_lfbuf_ddr u_lf_buf(
        .rst        ( rst           ),
        .clk        ( clk_rom       ),
        .pxl_cen    ( pxl1_cen      ),

        .hs         ( hs            ),
        .vs         ( vs            ),
        .lvbl       ( LVBL          ),
        .lhbl       ( LHBL          ),
        .vrender    ( game_vrender  ),
        .hdump      ( game_hdump    ),

        // interface with the game core
        .ln_addr    ( ln_addr       ),
        .ln_data    ( ln_data       ),
        .ln_done    ( ln_done       ),
        .ln_hs      ( ln_hs         ),
        .ln_dout    ( ln_dout       ),
        .ln_pxl     ( ln_pxl        ),
        .ln_v       ( ln_v          ),
        .ln_vs      ( ln_vs         ),
        .ln_lvbl    ( ln_lvbl       ),
        .ln_we      ( ln_we         ),

        .ddram_clk  ( DDRAM_CLK     ),
        .ddram_busy ( DDRAM_BUSY    ),
        .ddram_addr ( DDRAM_ADDR    ),
        .ddram_dout ( DDRAM_DOUT    ),
        .ddram_rd   ( DDRAM_RD      ),
        .ddram_din  ( DDRAM_DIN     ),
        .ddram_be   ( DDRAM_BE      ),
        .ddram_we   ( DDRAM_WE      ),
        .ddram_burstcnt  ( DDRAM_BURSTCNT    ),
        .ddram_dout_ready( DDRAM_DOUT_READY  ),
        .st_addr    ( st_addr       ),
        .st_dout    ( st_lpbuf      )
    );
`else
    wire        mr_ddr_clk;
    wire [ 7:0] mr_ddr_burstcnt;
    wire [28:0] mr_ddr_addr;
    wire        mr_ddr_rd;
    wire        mr_ddr_we;
    wire [ 7:0] mr_ddr_be;
    wire        mr_ddr_busy = ddr_ss.acquire ? 1'b1 : DDRAM_BUSY;

    jtframe_mr_ddrmux u_ddrmux(
        .rst            ( rst             ),
        .clk            ( clk_rom         ),
        .ioctl_rom      ( ioctl_rom       ),
        // Fast DDR load
        .ddrld_burstcnt ( ddrld_burstcnt  ),
        .ddrld_addr     ( ddrld_addr      ),
        .ddrld_rd       ( ddrld_rd        ),
        .ddrld_busy     ( ddrld_busy      ),
        // Rotation signals
        .rot_clk        ( rot_clk         ),
        .rot_burstcnt   ( rot_burstcnt    ),
        .rot_addr       ( rot_addr        ),
        .rot_rd         ( rot_rd          ),
        .rot_we         ( rot_we          ),
        .rot_be         ( rot_be          ),
        .rot_busy       ( rot_busy        ),
        // DDR Signals
        .ddr_clk        ( mr_ddr_clk      ),
        .ddr_busy       ( mr_ddr_busy     ),
        .ddr_burstcnt   ( mr_ddr_burstcnt ),
        .ddr_addr       ( mr_ddr_addr     ),
        .ddr_rd         ( mr_ddr_rd       ),
        .ddr_we         ( mr_ddr_we       ),
        .ddr_be         ( mr_ddr_be       )
    );

    assign DDRAM_CLK      = ddr_ss.acquire ? clk_rom : mr_ddr_clk;
    assign DDRAM_BURSTCNT = ddr_ss.acquire ? ddr_ss.burstcnt : mr_ddr_burstcnt;
    assign DDRAM_ADDR     = ddr_ss.acquire ? ddr_ss.addr[31:3] : mr_ddr_addr;
    assign DDRAM_RD       = ddr_ss.acquire ? ddr_ss.read : mr_ddr_rd;
    assign DDRAM_WE       = ddr_ss.acquire ? ddr_ss.write : mr_ddr_we;
    assign DDRAM_BE       = ddr_ss.acquire ? ddr_ss.byteenable : mr_ddr_be;
    assign DDRAM_DIN      = ddr_ss.acquire ? ddr_ss.wdata : rot_din;

    assign ddr_ss.rdata       = DDRAM_DOUT;
    assign ddr_ss.rdata_ready = DDRAM_DOUT_READY;
    assign ddr_ss.busy        = DDRAM_BUSY;
`endif


`ifndef JTFRAME_STEREO
    assign snd_right = snd_left;
`endif

endmodule
