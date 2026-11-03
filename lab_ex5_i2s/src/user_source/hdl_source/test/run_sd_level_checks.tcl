onerror {quit -code 1 -f}
onbreak {}
set test_dir [file dirname [file normalize [info script]]]
set rtl [file dirname $test_dir]
if {![file isdirectory video_sd_level_work]} {vlib video_sd_level_work}
vlog -work video_sd_level_work $rtl/video_player.v $rtl/SD/sd_card_sec_read_write.v $test_dir/tb_video_sd_level.v
foreach gap {0 32 99} {
    vsim -quiet -voptargs=+acc -onfinish stop video_sd_level_work.tb_video_sd_level +GAP=$gap
    run -all
    if {[catch {examine -radix unsigned sim:/tb_video_sd_level/test_done} passed] || $passed ne "1"} {quit -code 1 -f}
    quit -sim
}
quit -code 0 -f
