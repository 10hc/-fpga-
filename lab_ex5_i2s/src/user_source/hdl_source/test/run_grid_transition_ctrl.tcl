# Run with: vsim -c -do "source <absolute-path>/run_grid_transition_ctrl.tcl"
# Products stay in the caller's directory; onfinish stop preserves result regs.
onerror {echo "GRID_CONTROL_SENTINEL FAIL simulator_error"; quit -code 1 -f}
onbreak {}
set test_dir [file dirname [file normalize [info script]]]
set rtl [file dirname $test_dir]
set task_dir [pwd]
set work_dir [file join $task_dir grid_control_work]
file mkdir $work_dir
cd $work_dir
transcript file [file join $task_dir grid_control.log]
transcript on
set library [file join $work_dir grid_control_lib]
if {![file isdirectory $library]} {vlib $library}
vlog -work $library [file join $rtl trans_ctrl.v] [file join $rtl top_tf_hdmi_audio.v] \
    [file join $test_dir tb_trans_ctrl.v] [file join $test_dir tb_grid_transition_ctrl.v]
foreach bench {tb_trans_ctrl tb_grid_transition_ctrl} {
    vsim -quiet -voptargs=+acc -onfinish stop -lib $library $bench
    run -all
    # Neither $finish nor the process exit status alone establishes success.
    if {[catch {examine -radix unsigned sim:/$bench/test_done} done] ||
        [catch {examine -radix unsigned sim:/$bench/test_passed} passed]} {
        echo "GRID_CONTROL_SENTINEL FAIL $bench missing_result"
        quit -code 1 -f
    }
    if {$done ne "1" || $passed ne "1"} {
        echo "GRID_CONTROL_SENTINEL FAIL $bench done=$done passed=$passed"
        quit -code 1 -f
    }
    echo "GRID_CONTROL_BENCH_PASS $bench done=$done passed=$passed"
    quit -sim
}
echo "GRID_CONTROL_SENTINEL PASS benches=2"
quit -code 0 -f
