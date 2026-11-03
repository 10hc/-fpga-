onerror {quit -code 1 -f}
onbreak {quit -code 1 -f}
set test_dir [file dirname [file normalize [info script]]]
set rtl [file dirname $test_dir]
if {![file isdirectory video_checks_work]} {vlib video_checks_work}
vlog -work video_checks_work $rtl/video_player.v $test_dir/tb_video.v $test_dir/tb_video_backpressure.v
vsim -c -quiet -voptargs=+acc -onfinish stop video_checks_work.tb_video
run -all
if {[catch {examine -radix unsigned sim:/tb_video/test_passed} passed] || $passed ne "1"} {quit -code 1 -f}
quit -sim
foreach seed {1 20261030 730201} {
    vsim -c -quiet -voptargs=+acc -onfinish stop video_checks_work.tb_video_backpressure +SEED=$seed
    run -all
    if {[catch {examine -radix unsigned sim:/tb_video_backpressure/test_passed} passed] || $passed ne "1"} {quit -code 1 -f}
    quit -sim
}
quit -code 0 -f
