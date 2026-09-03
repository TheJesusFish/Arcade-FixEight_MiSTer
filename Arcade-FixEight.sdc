derive_pll_clocks
derive_clock_uncertainty

create_generated_clock -name SDRAM_CLK -source \
    [get_pins {emu|pll|raizingpll_inst|altera_pll_i|general[5].gpll~PLL_OUTPUT_COUNTER|divclk}] \
    -divide_by 1 \
    [get_ports SDRAM_CLK]

# joy_db15 samples its serial inputs from counter bit 4: clk_sys / 32.
create_generated_clock -name DB15_JOY_CLK -source \
    [get_pins {emu|u_joymux|u_db15|JCLOCKS[0]|clk}] \
    -divide_by 32 \
    [get_pins {emu|u_joymux|u_db15|JCLOCKS[4]|q}]

set_multicycle_path -from [get_clocks {SDRAM_CLK}] -to [get_clocks {emu|pll|raizingpll_inst|altera_pll_i|general[4].gpll~PLL_OUTPUT_COUNTER|divclk}] -setup -end 2
set_multicycle_path -from [get_clocks {SDRAM_CLK}] -to [get_clocks {emu|pll|raizingpll_inst|altera_pll_i|general[4].gpll~PLL_OUTPUT_COUNTER|divclk}] -hold -end 2

# The restored shell uses the Raizing PLL instance. Keep SDRAM/game clocks
# related and cut unrelated framework, audio, video, and HPS clock domains.
set_clock_groups -exclusive \
    -group [get_clocks {emu|pll|raizingpll_inst|altera_pll_i|general[4].gpll~PLL_OUTPUT_COUNTER|divclk emu|pll|raizingpll_inst|altera_pll_i|general[5].gpll~PLL_OUTPUT_COUNTER|divclk SDRAM_CLK}] \
    -group [get_clocks {pll_hdmi|pll_hdmi_inst|altera_pll_i|cyclonev_pll|counter[0].output_counter|divclk}] \
    -group [get_clocks {pll_audio|pll_audio_inst|altera_pll_i|general[0].gpll~PLL_OUTPUT_COUNTER|divclk}] \
    -group [get_clocks {spi_sck}] \
    -group [get_clocks {hdmi_sck}] \
    -group [get_clocks {sysmem|fpga_interfaces|clocks_resets|h2f_user0_clk}] \
    -group [get_clocks {FPGA_CLK1_50}] \
    -group [get_clocks {FPGA_CLK2_50}] \
    -group [get_clocks {FPGA_CLK3_50}]

# SDRAM timing constraints for the inlined JTFrame/MiSTer hierarchy.
set_multicycle_path -setup -end -from [get_keepers {SDRAM_DQ[*]}] -to [get_keepers {emu:emu|jtframe_board:u_board|jtframe_board_sdram:u_sdram|jtframe_sdram64:u_sdram|dout[*]}] 2
set_multicycle_path -hold  -end -from [get_keepers {SDRAM_DQ[*]}] -to [get_keepers {emu:emu|jtframe_board:u_board|jtframe_board_sdram:u_sdram|jtframe_sdram64:u_sdram|dout[*]}] 2
set_multicycle_path -setup -end -from [get_keepers {emu:emu|jtframe_board:u_board|jtframe_board_sdram:u_sdram|jtframe_sdram64:u_sdram|dq_pad[*]}] -to [get_keepers {SDRAM_DQ[*]}] 2
set_multicycle_path -hold  -end -from [get_keepers {emu:emu|jtframe_board:u_board|jtframe_board_sdram:u_sdram|jtframe_sdram64:u_sdram|dq_pad[*]}] -to [get_keepers {SDRAM_DQ[*]}] 2
set_multicycle_path -setup -end -from [get_keepers {emu:emu|jtframe_board:u_board|jtframe_board_sdram:u_sdram|jtframe_sdram64:u_sdram|sdram_a[12]}] -to [get_keepers {SDRAM_DQMH}] 2
set_multicycle_path -hold  -end -from [get_keepers {emu:emu|jtframe_board:u_board|jtframe_board_sdram:u_sdram|jtframe_sdram64:u_sdram|sdram_a[12]}] -to [get_keepers {SDRAM_DQMH}] 2
set_multicycle_path -setup -end -from [get_keepers {emu:emu|jtframe_board:u_board|jtframe_board_sdram:u_sdram|jtframe_sdram64:u_sdram|sdram_a[11]}] -to [get_keepers {SDRAM_DQML}] 2
set_multicycle_path -hold  -end -from [get_keepers {emu:emu|jtframe_board:u_board|jtframe_board_sdram:u_sdram|jtframe_sdram64:u_sdram|sdram_a[11]}] -to [get_keepers {SDRAM_DQML}] 2

# The compact V25 advances only when its exact 16 MHz fractional enable is
# asserted. At least five 94.5 MHz clocks separate enables and every
# sequential element inside u_cpu is gated by that enable. Keep wrapper and
# device buses outside this exception so they remain single-cycle constrained.
set v25_cpu_keepers [get_keepers {emu:emu|fixeight_game:u_game|fixeight_core:u_core|fixeight_sound:u_sound|fixeight_v25_cpu:u_v25|fixeight_z8086:u_cpu|*}]
set_multicycle_path -setup -from $v25_cpu_keepers -to $v25_cpu_keepers 5
set_multicycle_path -hold  -from $v25_cpu_keepers -to $v25_cpu_keepers 4

# JTFrame framework exceptions.
set_false_path -to [get_keepers {audio_out:audio_out|cl1[*]}]
set_false_path -to [get_keepers {audio_out:audio_out|cr1[*]}]
set_false_path -from [get_keepers {emu:emu|jtframe_board:u_board|jtframe_reset:u_reset|rst_rom[0]}] -to [get_keepers {emu:emu|jtframe_board:u_board|jtframe_reset:u_reset|rst_rom_sync}]
set_false_path -to emu:emu|jtframe_board:u_board|jtframe_reset:u_reset|rst_req_sync[0]
set_false_path -from FB_EN
set_false_path -to deb_osd[0]
set_false_path -from emu:emu|jtframe_board:u_board|jtframe_led:u_led|led
set_false_path -to [get_keepers {*altera_std_synchronizer:*|din_s1}]
