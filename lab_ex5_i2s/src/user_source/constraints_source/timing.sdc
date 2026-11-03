# TD 6.2.168116 / EG4S20BG256. CDC repair, 2026-10-30.
# Keep PLL-derived frequencies/phases; do not recreate internal clocks.
create_clock -name clk -period 20 -waveform {0 10} [get_ports {clk}]
derive_clocks
rename_clock -name {sd_card_clk} -source [get_ports {clk}] -master_clock {clk} [get_pins {sys_pll_m0/pll_inst.clkc[0]}]
rename_clock -name {ext_mem_clk} -source [get_ports {clk}] -master_clock {clk} [get_pins {sys_pll_m0/pll_inst.clkc[1]}]
rename_clock -name {video_clk}   -source [get_ports {clk}] -master_clock {clk} [get_pins {video_pll_m0/pll_inst.clkc[0]}]
rename_clock -name {hdmi_5x_clk} -source [get_ports {clk}] -master_clock {clk} [get_pins {video_pll_m0/pll_inst.clkc[1]}]
rename_clock -name {audio_mclk}  -source [get_ports {clk}] -master_clock {clk} [get_pins {u_audio_pll/pll_inst.clkc[2]}]

# No clock-group cuts: video pixel/5x operate concurrently; SD/memory are
# outputs of the same system PLL. Keep their real inter-clock timing, and
# keep ext_mem_clk <-> sys_pll_m0/pll_inst.clkc[2] (125 MHz, 180 degrees).
# No cross-domain-wide false_path, multicycle, phase or IO-delay guesses.

# I2S_receiver.v:42-53: ONLY the three first sampling flops, with the
# corresponding registered transmitter output as source. Subsequent stages
# and all other audio paths remain timed. No overlap with FIFO Gray regs.
set_false_path -from [get_regs {u_hdmi_audio_tone_i2s_64fs/O_i2s_BCLK}] -to [get_regs {u_I2S_receiver/S_i2s_bclk_1d}]
set_false_path -from [get_regs {u_hdmi_audio_tone_i2s_64fs/O_i2s_LRCK}] -to [get_regs {u_I2S_receiver/S_i2s_lrck_1d}]
set_false_path -from [get_regs {u_hdmi_audio_tone_i2s_64fs/O_i2s_DOUT}] -to [get_regs {u_I2S_receiver/S_i2s_dout_1d}]

# Single-bit controls: ONLY source domain -> explicitly audited FIRST stage.
# The first stage feeds only the next synchronizer stage unless noted below.
# mode_pixel_sync[1], request_sync[1], wrfin_tgl_sync[1:2], fin_sync[1:2]
# and all functional consumers retain their destination-clock timing.
set_false_path -from [get_regs {u_media_mode_ctrl/active_mode}] -to [get_regs {mode_pixel_sync[0]}]
set_false_path -from [get_regs {video_mode_requested}] -to [get_regs {u_media_mode_ctrl/request_sync[0]}]
set_false_path -from [get_regs {frame_write_toggle_mem}] -to [get_regs {sd_card_bmp_m0/wrfin_tgl_sync[0] u_video_player/fin_sync[0]}]

# Registered read request is held until acknowledgement. Only d0 is cut;
# the unsynchronized return acknowledgement is NOT cut at functional logic.
set_false_path -from [get_regs {video_timing_data_m0/read_req}] -to [get_regs {frame_read_write_m0/frame_fifo_read_m0/read_req_d0}]
# Write request/abort are control levels, held by the SD-domain producer.
# Restrict to these first sampling flops; no address/length/data bus cuts.
set_false_path -from [get_clocks -of [get_pins {sys_pll_m0/pll_inst.clkc[0]}]] -to [get_regs {frame_read_write_m0/frame_fifo_write_m0/write_req_d0 frame_read_write_m0/frame_fifo_write_m0/abort_sync[0]}]

# Memory idle status is a single-domain decode sampled through 2/3 flops.
# Top and manager first-stage logical aliases may merge to the same flop.
set_false_path -from [get_clocks -of [get_pins {sys_pll_m0/pll_inst.clkc[1]}]] -to [get_regs {write_idle_sync[0] sd_card_bmp_m0/writer_idle_sync[0]}]
# inhibit = active_mode OR pending, both SD-domain controls; only its first
# sampler is exempt, not the arbitrary combinational CDCs elsewhere.
set_false_path -from [get_clocks -of [get_pins {sys_pll_m0/pll_inst.clkc[0]}]] -to [get_regs {u_image_transition_ctrl/inhibit_sync[0]}]

# Transition ownership req/ack added by the parallel RTL repair. These exact
# names must be checked on the NEW elaborated/routed netlist (absent in 19:12 DB).
set_false_path -from [get_regs {sd_card_bmp_m0/transition_req_toggle}] -to [get_regs {u_image_transition_ctrl/release_req_sync[0]}]
set_false_path -from [get_regs {u_image_transition_ctrl/transition_done_toggle}] -to [get_regs {sd_card_bmp_m0/transition_done_sync[0]}]
set_false_path -from [get_clocks -of [get_pins {sys_pll_m0/pll_inst.clkc[1]}]] -to [get_regs {sd_card_bmp_m0/transition_busy_sync[0]}]

# Descriptor is a bundled payload, NOT a false path. Bound its registered
# SD producer -> memory-domain first sample physically to min(10,8)-0.3ns.
# The producer holds the published descriptor while awaiting release ack.
# d0->d1->d2 and stable/commit logic are timed normally in ext_mem_clk.
set_max_delay -from [get_regs {sd_card_bmp_m0/display_valid sd_card_bmp_m0/disp_buf_idx[*]}] -to [get_regs {u_image_transition_ctrl/image_d0[*]}] -datapath_only 7.700

# WAV has NO cross-domain binary buffer pointers: wrp AND rdp use sd_clk.
# Only s_toggle and held s_l/s_r cross to video. Exempt only the incoming
# control sample; h_l/h_r bundled payload and all paths OUT of tog_sync[0]
# stay timed. The repaired RTL uses three stages and consumes [2] ^ [1],
# so the first sample is used only by the next synchronizer stage.
set_false_path -from [get_regs {u_wav_audio/s_toggle}] -to [get_regs {u_wav_audio/tog_sync[0]}]

# Gray pointer max delay is provided by the existing IP SDCs, loaded AFTER
# this user SDC by TD settings.cfg with read_sdc -ip:
#   rfifo_32_32_512 : ip/afifo_32_16_256.tcl
#   wfifo_32_32_512 : ip/afifo_16_32_256.tcl
# Each uses primary_addr_gray_reg[*] -> sync_r1[*], datapath_only,
# min(clkw period, clkr period)-0.3ns = 7.7ns for the current configuration.
# TD SWUG200 3.11.3: datapath_only also suppresses hold on THESE paths.
# Do NOT add false_path (including setup-only) over those Gray paths;
# do NOT replace this with set_clock_groups or reorder to try to override it.
# Other CDC / bundled buses remain analyzed pending protocol/endpoint review.
# Unspecified board IO delays remain a separate, open interface requirement.

# Frame writer ACK is a registered level held until producer request drops.
# Only the newly added first samplers are exempt; ack_sync[1] and consumers
# keep normal SD-clock timing. FIFO flags use their native domain in RTL.
set_false_path -from [get_regs {frame_read_write_m0/frame_fifo_write_m0/write_req_ack}] -to [get_regs {u_video_player/ack_sync[0] sd_card_bmp_m0/bmp_read_m0/ack_sync[0]}]

# fifo_aclr asynchronously asserts the TWO reset-release flops only. Both
# stages stay asserted through clear; write_ready uses stage 2 after two local
# edges. Exempt incoming reset assertion/recovery, retaining normal timing on
# the local shift chain, FIFO count/full and all producer enable paths.
set_false_path -from [get_regs {frame_read_write_m0/frame_fifo_write_m0/fifo_aclr}] -to [get_regs {frame_read_write_m0/write_reset_sync[*]}]

# Slowly changing diagnostic state / display visibility has its own pixel
# sampler. This bound applies ONLY to the first sampling stage; later stages
# and the pixel color selector keep normal video-clock timing. This bus is
# never used as a memory descriptor or as a frame ownership acknowledgement.
set_max_delay -from [get_clocks {sd_card_clk}] -to [get_regs {u_display_status/status_d0[*]}] -datapath_only 9.700
