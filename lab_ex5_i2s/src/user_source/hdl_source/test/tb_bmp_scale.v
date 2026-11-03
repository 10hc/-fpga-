`timescale 1ns/1ps
// Independent integer-coordinate oracle, a sector SD model with byte gaps,
// and the actual asynchronous FIFO / frame writer at unrelated clock phases.
module tb_bmp_scale;
reg clk=0, mem_clk=0, rst=1;
always #5 clk=~clk;
always #4 mem_clk=~mem_clk;
reg scan_start=0, load_start=0, abort=0;
wire ready, scan_done, found, req, ack, pixel_en, load_done, load_error;
wire [2:0] found_count;
wire [23:0] pixel;
wire sd_req;
wire [31:0] sd_addr;
reg [7:0] sd_data=0;
reg sd_valid=0, sd_end=0;
wire write_ready, write_idle, write_finish;
reg [3:0] abort_hold=0;
wire writer_abort=abort || abort_hold!=0;
reg [1:0] buffer_index=0;
integer w=640, h=480, td=0, off=54, bad=0, short_at=-1, sectors=0;
integer card_state=0, byte_index=0, card_addr=0, sector_length=512;
integer out_count=0, mem_count=0, finishes=0, loads=0, errors=0, case_no=0;
integer cycle=0, stalls=0, line_stride, file_size, fit_w, fit_h, left, top;
reg [23:0] last_pixel;
reg last_stalled=0;
reg mem_busy=0;
wire mem_we;
wire [20:0] mem_addr;
wire [31:0] mem_data;

bmp_read dut(
    .clk(clk),.rst(rst),.ready(ready),.scan_start(scan_start),
    .scan_start_sector(32'd100),.scan_max_sector(32'd100),.scan_target_count(3'd1),
    .scan_done(scan_done),.scan_found_valid(found),.scan_found_sector(),.scan_found_total(found_count),
    .jump_en(1'b1),.jump_count(3'd1),.jump_sector0(32'd100),
    .jump_sector1(32'd0),.jump_sector2(32'd0),.jump_sector3(32'd0),
    .load_start(load_start),.load_sector(32'd100),.sd_init_done(1'b1),.state_code(),
    .bmp_width(16'd640),.bmp_height(16'd480),.write_req(req),.write_req_ack(ack),
    .sd_sec_read(sd_req),.sd_sec_read_addr(sd_addr),.sd_sec_read_data(sd_data),
    .sd_sec_read_data_valid(sd_valid),.sd_sec_read_end(sd_end),
    .bmp_data_ready(write_ready),.abort(abort),.load_done(load_done),.load_error(load_error),
    .bmp_data_wr_en(pixel_en),.bmp_data(pixel)
);
frame_read_write memory_path(
    .rst(rst),.mem_clk(mem_clk),.Sdr_init_done(!rst),.Sdr_init_ref_vld(1'b0),.Sdr_busy(mem_busy),
    .App_rd_en(),.App_rd_addr(),.Sdr_rd_en(1'b0),.Sdr_rd_dout(32'd0),
    .read_clk(clk),.read_req(1'b0),.read_req_ack(),.read_finish(),
    .read_addr_0(21'd0),.read_addr_1(21'd307200),.read_addr_2(21'd614400),.read_addr_3(21'd921600),
    .read_addr_index(2'd0),.read_len(21'd307200),.read_en(1'b0),.read_data(),
    .App_wr_en(mem_we),.App_wr_addr(mem_addr),.App_wr_din(mem_data),.App_wr_dm(),
    .write_clk(clk),.write_req(req),.write_req_ack(ack),.write_finish(write_finish),
    .write_abort(writer_abort),.write_idle(write_idle),.write_ready(write_ready),
    .write_addr_0(21'd0),.write_addr_1(21'd307200),.write_addr_2(21'd614400),.write_addr_3(21'd921600),
    .write_addr_index(buffer_index),.write_len(21'd307200),.write_en(pixel_en),.write_data({pixel,8'd0}),
    .trans_abort(1'b1),.trans_en(1'b0),.trans_effect(2'd0),.trans_offset(10'd0),
    .trans_old_base(21'd0),.trans_new_base(21'd0),.dbg_rd_state(),.dbg_rd_wrbusy(),.dbg_rd_req2(),.dbg_rd_init()
);

function [23:0] source_pixel;
    input integer x,y;
    reg [7:0] r,g,b;
    begin r=(x*13+y*7+case_no*19); g=(y*11+x/256); b=(x*3+y/256); source_pixel={r,g,b}; end
endfunction
function [23:0] expected_pixel;
    input integer n;
    integer x,y,sx,sy;
    begin
        x=n%640; y=479-n/640;
        if(x<left || x>=left+fit_w || y<top || y>=top+fit_h) expected_pixel=0;
        else begin sx=(x-left)*w/fit_w; sy=(y-top)*h/fit_h; expected_pixel=source_pixel(sx,sy); end
    end
endfunction
function [7:0] file_byte;
    input integer a;
    integer raw_y, y, x, component, field;
    reg [23:0] p;
    begin
        file_byte=0;
        case(a)
            0:file_byte="B"; 1:file_byte=bad==1 ? "X" : "M";
            2,3,4,5:begin field=bad==2 ? file_size-1 : file_size; file_byte=field>>((a-2)*8); end
            10,11,12,13:begin field=bad==12 ? 30 : off; file_byte=field>>((a-10)*8); end
            14,15,16,17:begin field=bad==3 ? 12 : 40; file_byte=field>>((a-14)*8); end
            18,19,20,21:begin field=bad==4 ? 65536+w : (bad==8 ? 0 : (bad==10 ? 2049 : w)); file_byte=field>>((a-18)*8); end
            22,23,24,25:begin field=bad==9 ? 0 : (bad==11 ? 32'h80000000 : (td ? -h : h)); file_byte=field>>((a-22)*8); end
            26:file_byte=bad==5 ? 2 : 1;
            28:file_byte=bad==6 ? 32 : 24;
            30:file_byte=bad==7 ? 1 : 0;
            default:if(a>=off && a<file_size) begin
                raw_y=(a-off)/line_stride; y=td ? raw_y : h-1-raw_y;
                x=((a-off)%line_stride)/3; component=((a-off)%line_stride)%3;
                p=source_pixel(x,y);
                if(x>=w) file_byte=8'ha5; // nonzero padding exposes alignment errors
                else file_byte=p>>(component*8);
            end
        endcase
    end
endfunction

always @(posedge clk) begin
    if(rst) begin card_state<=0; sd_valid<=0; sd_end<=0; abort_hold<=0; end
    else begin
        cycle=cycle+1; sd_valid<=0; sd_end<=0;
        if(load_error) begin errors=errors+1; abort_hold<=15; end
        else if(abort_hold!=0) abort_hold<=abort_hold-1;
        if(load_done) begin
            if(out_count!=307200) $fatal(1,"Incomplete success: %0d",out_count);
            loads=loads+1;
        end
        if(dut.u_scale.out_valid && !write_ready) stalls=stalls+1;
        if(last_stalled && !abort && !writer_abort &&
           (!dut.u_scale.out_valid || pixel!==last_pixel)) $fatal(1,"Unstable stalled output");
        last_stalled=dut.u_scale.out_valid && !write_ready && !writer_abort;
        last_pixel=pixel;
        if(pixel_en) begin
            if(memory_path.wr_fifo_full) $fatal(1,"FIFO overflow");
            if(pixel!==expected_pixel(out_count))
                $fatal(1,"Pixel mismatch case=%0d n=%0d got=%h expected=%h",case_no,out_count,pixel,expected_pixel(out_count));
            out_count=out_count+1;
            if(out_count>307200) $fatal(1,"Extra output");
        end
        case(card_state)
            0:if(sd_req) begin
                card_addr<=sd_addr; byte_index<=0;
                sector_length<=sectors==short_at ? 117 : 512;
                sectors=sectors+1; card_state<=1;
            end
            1:if(cycle%7!=0) begin
                sd_data<=file_byte((card_addr-100)*512+byte_index); sd_valid<=1;
                if(byte_index==sector_length-1) begin
                    if(case_no%2==0) begin sd_end<=1; card_state<=3; end
                    else card_state<=2;
                end
                else byte_index<=byte_index+1;
            end
            2:begin sd_end<=1; card_state<=3; end
            3:if(!sd_req) card_state<=0;
        endcase
    end
end
always @(posedge mem_clk) if(!rst) begin
    // Long refresh/contention windows force the image producer to stop.
    mem_busy <= cycle%5000 < 2500;
    if(mem_we) begin
        if(memory_path.write_buf.empty_flag) $fatal(1,"FIFO underflow");
        if(mem_data!=={expected_pixel(mem_count),8'd0}) $fatal(1,"SDRAM data mismatch n=%0d got=%h",mem_count,mem_data);
        if(mem_addr!==buffer_index*307200+(479-mem_count/640)*640+mem_count%640)
            $fatal(1,"SDRAM address mismatch");
        mem_count=mem_count+1;
    end
    if(write_finish) begin
        if(mem_count!=307200) $fatal(1,"Short memory frame %0d",mem_count);
        finishes=finishes+1;
    end
end

task setup_case;
    input integer iw,ih,itd,ioff;
    begin
        wait(ready && write_idle && card_state==0 && abort_hold==0);
        repeat(8) @(negedge clk);
        w=iw; h=ih; td=itd; off=ioff; bad=0; short_at=-1;
        line_stride=(w*3+3)/4*4; file_size=off+line_stride*h;
        if(w*480>=h*640) begin fit_w=640; fit_h=h*640/w; if(fit_h==0)fit_h=1; end
        else begin fit_h=480; fit_w=w*480/h; if(fit_w==0)fit_w=1; end
        left=(640-fit_w)/2; top=(480-fit_h)/2;
        out_count=0; mem_count=0; sectors=0; case_no=case_no+1; buffer_index=case_no%2;
    end
endtask
task load;
    begin @(negedge clk); load_start=1; @(negedge clk); load_start=0; end
endtask
task successful_case;
    input integer iw,ih,itd,ioff;
    integer old_loads,old_finishes,old_errors;
    begin
        setup_case(iw,ih,itd,ioff);
        old_loads=loads; old_finishes=finishes; old_errors=errors;
        @(negedge clk); scan_start=1; @(negedge clk); scan_start=0;
        wait(scan_done); @(negedge clk);
        if(found_count!=1) $fatal(1,"Dimension not scanned: %0dx%0d",w,h);
        wait(ready); load;
        wait(loads>old_loads || errors>old_errors);
        if(errors!=old_errors) $fatal(1,"Valid BMP rejected");
        wait(finishes>old_finishes); wait(ready && write_idle);
        $display("PASS BMP %0dx%0d top_down=%0d offset=%0d fit=%0dx%0d pixels=%0d sectors=%0d",w,h,td,off,fit_w,fit_h,out_count,sectors);
    end
endtask
integer i,old_errors,old_loads,old_finishes;
initial begin
    // The existing writer arbitrates on App_rd_busy, not on Sdr_busy.
    // Model long display-read bursts at that arbitration interface.
    force memory_path.App_rd_busy=mem_busy;
    #100; @(negedge clk); rst=0;
    if(!$test$plusargs("FAULT_ONLY")) begin
    successful_case(640,480,0,54);
    successful_case(320,240,0,54);
    successful_case(800,600,0,54);
    successful_case(800,800,0,54);
    successful_case(241,479,1,70);
    successful_case(1920,1080,1,1024);
    successful_case(641,481,0,54);
    successful_case(1,1,0,54);
    successful_case(2048,1,1,54);
    successful_case(1,2048,0,54);
    successful_case(639,479,1,509);
    end
    for(i=1;i<=12;i=i+1) begin
        setup_case(321,241,0,54); bad=i; old_errors=errors; old_loads=loads; old_finishes=finishes;
        load; wait(errors>old_errors); wait(ready && write_idle && abort_hold==0);
        if(out_count!=0 || loads!=old_loads || finishes!=old_finishes) $fatal(1,"Invalid header published");
        $display("PASS reject header %0d",i);
    end
    setup_case(640,480,0,54); short_at=0; old_errors=errors;
    load; wait(errors>old_errors); if(out_count!=0) $fatal(1,"Short header accepted");
    $display("PASS short header");
    setup_case(640,480,0,54); short_at=20; old_errors=errors; old_loads=loads; old_finishes=finishes;
    load; wait(errors>old_errors); wait(ready && write_idle && abort_hold==0);
    if(loads!=old_loads || finishes!=old_finishes) $fatal(1,"Short payload published");
    $display("PASS short payload, partial words=%0d",out_count);
    setup_case(800,600,1,54); old_errors=errors; old_loads=loads;
    load; wait(out_count>2000 && sd_req); @(negedge clk); abort=1;
    @(negedge clk); abort=0; wait(errors>old_errors); wait(ready && write_idle && abort_hold==0);
    if(loads!=old_loads) $fatal(1,"Cancelled frame published");
    $display("PASS cancel during SD receive");
    setup_case(1,1,0,54); old_errors=errors;
    load; wait(dut.u_scale.out_valid && !write_ready); @(negedge clk); abort=1;
    @(negedge clk); abort=0;
    wait(errors>old_errors); wait(ready && write_idle && abort_hold==0);
    $display("PASS cancel while output stalled");
    setup_case(320,240,0,54); old_errors=errors;
    load; wait(sd_req && dut.state==2); @(negedge clk); abort=1;
    @(negedge clk); abort=0; wait(errors>old_errors); wait(ready && write_idle && abort_hold==0);
    if(out_count!=0) $fatal(1,"Cancelled header produced pixels");
    $display("PASS cancel during header receive");
    // Cancellation must remain sticky after the SD transaction has drained,
    // including the cycle that publishes the registered validation result.
    for(i=5;i<=10;i=i+1) begin
        if(i!=9) begin
            setup_case(320,240,0,54); old_errors=errors; old_loads=loads;
            load; wait(dut.state==i); @(negedge clk); abort=1;
            @(negedge clk); abort=0;
            wait(errors>old_errors); wait(ready && write_idle && abort_hold==0);
            if(out_count!=0 || loads!=old_loads) $fatal(1,"Validation cancel published a frame");
            $display("PASS cancel validation stage %0d",i);
        end
    end
    successful_case(321,241,1,54);
    if(stalls==0) $fatal(1,"Backpressure was not exercised");
    $display("BMP_ADAPTIVE_ALL_PASSED cases=%0d stalls=%0d",case_no,stalls); $finish;
end
initial begin #1000000000; $fatal(1,"BMP test timeout case=%0d state=%0d scale=%0d memstate=%0d req=%b ack=%b idle=%b hold=%0d out=%0d errors=%0d old_errors=%0d loads=%0d card=%0d",case_no,dut.state,dut.u_scale.state,memory_path.frame_fifo_write_m0.state,req,ack,write_idle,abort_hold,out_count,errors,old_errors,loads,card_state); end
endmodule
