onerror {quit -code 1 -f}
onbreak {}
# Invoke with: vsim -c -do "source D:/.../test/run_av_video_checks.tcl"
set test_dir [file dirname [file normalize [info script]]]
set rtl [file dirname $test_dir]
set output_dir [file join [pwd] av_video_checks]
if {[info exists ::env(AV_VIDEO_OUTPUT)]} {set output_dir [file normalize $::env(AV_VIDEO_OUTPUT)]}
file mkdir $output_dir
cd $output_dir
transcript file [file join $output_dir av_video_transcript.log]
set lib [file join $output_dir av_video_work]
if {![file isdirectory $lib]} {vlib $lib}
vmap av_video_work $lib
if {[catch {vlog -work av_video_work $rtl/video_player.v $rtl/av_pcm_player.v $rtl/SD/sd_card_sec_read_write.v $test_dir/tb_av_video.v} compile_error]} {
    puts "AV_VIDEO_COMPILE_FAIL $compile_error"
    quit -code 1 -f
}
set cases {{20 0 0} {25 0 0} {20 2 0} {20 3 0} {25 4 0} {25 6 0} {12 7 0 35 12}}
for {set bad 0} {$bad < 12} {incr bad} {lappend cases [list 20 1 $bad]}
if {[info exists ::env(AV_VIDEO_CASES)]} {set cases $::env(AV_VIDEO_CASES)}
set report [open [file join $output_dir av_video_results.txt] w]
foreach item $cases {
    lassign $item fps scenario bad
    set gap 0
    set bits 12
    set real_rate 0
    if {[llength $item] >= 4} {set gap [lindex $item 3]}
    if {[llength $item] >= 5} {set bits [lindex $item 4]}
    if {$scenario == 7 || $scenario == 8} {set real_rate 1}
    puts "AV_VIDEO_RUN fps=$fps scenario=$scenario bad=$bad gap=$gap bits=$bits"
    vsim -quiet -voptargs=+acc -onfinish stop av_video_work.tb_av_video -gREAL_RATE=$real_rate -gPCM_BITS=$bits +FPS=$fps +SCENARIO=$scenario +BAD=$bad +GAP=$gap
    run -all
    set done 0
    set passed 0
    catch {set done [examine -radix unsigned sim:/tb_av_video/test_done]}
    catch {set passed [examine -radix unsigned sim:/tb_av_video/test_passed]}
    if {$done ne "1" || $passed ne "1"} {
        puts $report "FAIL fps=$fps scenario=$scenario bad=$bad gap=$gap bits=$bits done=$done passed=$passed"
        close $report
        quit -code 1 -f
    }
    puts $report "PASS fps=$fps scenario=$scenario bad=$bad gap=$gap bits=$bits"
    flush $report
    quit -sim
}
puts $report "ALL AV VIDEO CHECKS PASSED"
close $report
quit -code 0 -f
