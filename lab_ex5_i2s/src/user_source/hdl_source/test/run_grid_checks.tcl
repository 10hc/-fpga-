# Invoke with: vsim -c -do "source <absolute-path>/run_grid_checks.tcl"
# Run the ownership/effect tests separately with run_grid_transition_ctrl.tcl.
onerror {echo "GRID_CHECKS FAIL simulator_error"; quit -code 1 -f}
onbreak {}
set test_dir [file dirname [file normalize [info script]]]
set rtl [file dirname $test_dir]
set project_dir [file normalize [file join $rtl ../../td_project]]
set library [file join [pwd] grid_checks_work]
if {![file isdirectory $library]} {vlib $library}
vlog -work $library +incdir+$rtl/include+$project_dir \
    $rtl/SD/frame_fifo_read.v $rtl/include/sdr_as_ram.enc.v \
    $rtl/include/sdr_wrrd.enc.v $rtl/include/sdr_init_ref.enc.v \
    $test_dir/tb_grid_mapping.v $test_dir/tb_grid_sdram.v
foreach bench {tb_grid_mapping tb_grid_sdram} {
    vsim -quiet -voptargs=+acc -onfinish stop -lib $library $bench
    run -all
    if {[catch {examine -radix unsigned sim:/$bench/test_done} done] ||
        [catch {examine -radix unsigned sim:/$bench/test_passed} passed]} {
        echo "GRID_CHECKS FAIL $bench missing_result"
        quit -code 1 -f
    }
    if {$done ne "1" || $passed ne "1"} {
        echo "GRID_CHECKS FAIL $bench done=$done passed=$passed"
        quit -code 1 -f
    }
    echo "GRID_CHECKS_BENCH_PASS $bench"
    quit -sim
}
echo "GRID_CHECKS PASS benches=2"
quit -code 0 -f
