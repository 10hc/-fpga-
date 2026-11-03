onerror {quit -code 1 -f}
onbreak {}
# Invoke with: vsim -c -do "source D:/.../test/run_av_pcm_checks.tcl"
# source is required for info script on ModelSim 10.5b.
set test_dir [file dirname [file normalize [info script]]]
set project_dir [file dirname $test_dir]
set work_dir [file join [pwd] pcm_checks_work]
set log_dir [file join [pwd] logs]
file mkdir $work_dir $log_dir
transcript file [file join $log_dir av_pcm_checks.log]
if {![file exists [file join $work_dir _info]]} {vlib $work_dir}
vmap av_pcm_checks_work $work_dir
vlog -work av_pcm_checks_work [file join $project_dir src/user_source/hdl_source/av_pcm_player.v] [file join $test_dir tb_av_pcm_player.v]
set seeds {1}
# Optional full random rerun: set AV_PCM_SEEDS="1 20261031 730201".
if {[info exists ::env(AV_PCM_SEEDS)]} {set seeds $::env(AV_PCM_SEEDS)}
foreach seed $seeds {
    vsim -quiet -voptargs=+acc -onfinish stop av_pcm_checks_work.tb_av_pcm_player +SEED=$seed
    run -all
    if {[catch {examine -radix unsigned sim:/tb_av_pcm_player/testdone} done] || $done ne "1"} {quit -code 1 -f}
    if {[catch {examine -radix unsigned sim:/tb_av_pcm_player/test_passed} passed] || $passed ne "1"} {quit -code 1 -f}
    quit -sim
}
puts "AV_PCM_CHECKS_PASS all seeds"
quit -code 0 -f
