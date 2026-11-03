# Invoke with: vsim -c -do "source D:/.../test/run_bmp_checks.tcl"
onerror {quit -code 1 -f}
onbreak {quit -code 1 -f}
set testdir [file dirname [file normalize [info script]]]
set rtl [file dirname $testdir]
if {![file isdirectory bmp_checks_work]} {vlib bmp_checks_work}
vlog -work bmp_checks_work $rtl/bmp_scale.v $rtl/SD/bmp_read.v $rtl/SD/frame_read_write.v $rtl/SD/frame_fifo_write.v $rtl/SD/frame_fifo_read.v $rtl/ip/afifo_16_32_256.v $rtl/ip/afifo_32_16_256.v $testdir/tb_bmp_scale.v
vsim -c -quiet -onfinish stop bmp_checks_work.tb_bmp_scale
run -all
quit -sim
vlog -work bmp_checks_work $rtl/SD/sd_card_bmp.v $testdir/tb_image_commit.v
vsim -c -quiet -onfinish stop bmp_checks_work.tb_image_commit
run -all
quit -code 0 -f
