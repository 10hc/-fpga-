`timescale 1ns/1ps
// Real image owner, arbitration, BMP parser and scaler. Directory and physical
// SD are models; frame writer is a handshake model with delayed completion.
module tb_image_commit;
reg clk=0,rst=1,key_next=1,suspend=0,invalidate=0;
always #5 clk=~clk;
wire req,en,abort,storage_idle,valid,load_busy,wd_seen;
wire [31:0] data;
wire [1:0] write_idx,disp_idx;
reg ack=0,writer_idle=1,finish_toggle=0;
// This parser/writer regression has no transition reader. Model an immediate
// no-effect consumer acknowledgement; ownership/effect CDC is tested separately.
wire transition_req;
reg transition_done=0;
always @(posedge clk) if(rst)transition_done<=0;
else transition_done<=transition_req;
integer n=0,cycle=0,finish_wait=0,commits=0;
reg last_valid=0;
reg passed=0;
reg [1:0] last_idx=0;
wire downstream_ready=!writer_idle && cycle%13!=0;
sd_card_bmp #(.CLK_FREQ_HZ(1000)) img(
    .clk(clk),.rst(rst),.key_next(key_next),.key_auto(1'b1),
    .invalidate(invalidate),.storage_idle(storage_idle),.suspend(suspend),.single_buf(1'b0),
    .state_code(),.locate_done_latched(),.dbg(),.load_busy_o(load_busy),.wd_seen(wd_seen),.sd_init_done_o(),
    .locate_done(),.locate_error(),.fat_state_dbg(),.vid_file_valid(),.vid_file_sector(),.wav_file_valid(),.wav_file_sector(),
    .bmp_width(16'd640),.bmp_height(16'd480),.display_valid(valid),
    .write_finish_toggle(finish_toggle),.write_buf_idx(write_idx),.disp_buf_idx(disp_idx),
    .write_req(req),.write_req_ack(ack),.write_ready(downstream_ready),.write_idle(writer_idle),.write_abort(abort),
    .write_en(en),.write_data(data),.SD_nCS(),.SD_DCLK(),.SD_MOSI(),.SD_MISO(1'b1),
    .wav_sec_read(1'b0),.wav_sec_read_addr(32'd0),.wav_sec_read_data(),.wav_sec_read_data_valid(),.wav_sec_read_end(),
    .transition_req_toggle(transition_req),.transition_busy(1'b0),.transition_done_toggle(transition_done)
);
always @(posedge clk) if(!rst) begin
    cycle=cycle+1;
    if(img.load_start_pulse) n=0;
    if(req) begin ack<=1; writer_idle<=0; end else ack<=0;
    if(abort) begin writer_idle<=1; finish_wait=0; ack<=0; end
    if(en) begin
        if(!downstream_ready) $fatal(1,"Image ignored downstream ready");
        n=n+1;
        if(n==307200) finish_wait=37;
        if(n>307200) $fatal(1,"Extra image pixels");
    end
    if(finish_wait!=0) begin
        finish_wait=finish_wait-1;
        if(finish_wait==0) begin finish_toggle<=~finish_toggle; writer_idle<=1; end
    end
    if(img.source_done_seen && n!=307200) $fatal(1,"Parser idle mistaken for successful load");
    if(valid && (!last_valid || last_idx!=disp_idx)) begin
        if(n!=307200 || !writer_idle) $fatal(1,"Premature display commit");
        commits=commits+1;
    end
    last_valid=valid; last_idx=disp_idx;
end
task next_image;
    begin
        @(negedge clk); key_next=0; repeat(30)@(negedge clk);
        key_next=1; repeat(30)@(negedge clk);
    end
endtask
integer old_commits;
initial begin
    #100; @(negedge clk); rst=0;
    wait(commits==1 && storage_idle);
    if(disp_idx!=0) $fatal(1,"First image not in buffer zero");
    $display("PASS image commits only after source success and delayed SDRAM finish");
    img.sd_card_top_m0.bad_second=1;
    next_image; wait(wd_seen && !load_busy && storage_idle && !abort);
    if(commits!=1 || !valid || disp_idx!=0) $fatal(1,"Invalid header changed display");
    $display("PASS rejected image keeps displayed buffer and releases writer");
    img.sd_card_top_m0.bad_second=0;
    next_image; wait(commits==2 && storage_idle);
    if(disp_idx!=1) $fatal(1,"Next image buffer mismatch");
    $display("PASS retry after rejection loads scaled image into alternate buffer");
    next_image; wait(en && n>2000);
    @(negedge clk); force img.load_wd=32'd1000000000;
    repeat(2)@(negedge clk); release img.load_wd;
    wait(!load_busy && storage_idle && !abort);
    if(commits!=2 || !valid || disp_idx!=1) $fatal(1,"Watchdog published partial image");
    $display("PASS load watchdog drains SD, aborts writer, preserves display");
    next_image; wait(commits==3 && storage_idle);
    suspend=1; next_image; repeat(200)@(negedge clk);
    if(load_busy || req || commits!=3) $fatal(1,"Suspended image path started a load");
    @(negedge clk); invalidate=1; @(negedge clk); invalidate=0;
    repeat(5)@(negedge clk); if(valid) $fatal(1,"Invalidate did not clear image state");
    suspend=0; wait(commits==4 && storage_idle);
    $display("PASS suspend, invalidate and return to images");
    $display("IMAGE_COMMIT_ALL_PASSED commits=%0d",commits); passed=1; $finish;
end
initial begin #150000000; $fatal(1,"Image owner timeout load=%b ready=%b scale=%0d n=%0d commits=%0d",load_busy,img.bmp_ready,img.bmp_read_m0.u_scale.state,n,commits); end
endmodule

module fat32_scan #(parameter [2:0] MAX_BMP_COUNT=4)(
 input clk,rst,start,sd_init_done,
 output busy, output reg done, output error,
 output sd_req, output [3:0] state_dbg, output [31:0] sd_addr,
 input [7:0] sd_data, input sd_data_valid,sd_end,
 output [2:0] bmp_count, output [31:0] bmp_sector0,bmp_sector1,bmp_sector2,bmp_sector3,
 output wav_valid,output [31:0] wav_sector,output vid_valid,output [31:0] vid_sector
);
assign busy=0; assign error=0; assign sd_req=0; assign state_dbg=0; assign sd_addr=0;
assign bmp_count=2; assign bmp_sector0=100; assign bmp_sector1=1000; assign bmp_sector2=0; assign bmp_sector3=0;
assign wav_valid=0; assign wav_sector=0; assign vid_valid=0; assign vid_sector=0;
always @(posedge clk) if(rst)done<=0; else done<=start;
endmodule

module sd_card_top(
 input clk,rst,output SD_nCS,SD_DCLK,SD_MOSI,input SD_MISO,output sd_init_done,
 input sd_sec_read,input [31:0] sd_sec_read_addr,
 output reg [7:0] sd_sec_read_data,output reg sd_sec_read_data_valid,sd_sec_read_end,
 input sd_sec_write,input [31:0] sd_sec_write_addr,input [7:0] sd_sec_write_data,
 output sd_sec_write_data_req,sd_sec_write_end
);
assign SD_nCS=1; assign SD_DCLK=0; assign SD_MOSI=0; assign sd_init_done=!rst;
assign sd_sec_write_data_req=0; assign sd_sec_write_end=0;
reg bad_second=0;
integer state=0,addr=0,b=0;
function [7:0] file_byte;
 input integer address,byte_num;
 integer a,w,h,stride,len,field;
 begin
  w=address>=1000 ? 321 : 320; h=address>=1000 ? 241 : 240;
  a=(address-(address>=1000 ? 1000 : 100))*512+byte_num;
  stride=(w*3+3)/4*4; len=54+stride*h; file_byte=0;
  case(a)
   0:file_byte="B"; 1:file_byte=address>=1000 && bad_second ? "X" : "M";
   2,3,4,5:file_byte=len>>((a-2)*8);
   10:file_byte=54; 14:file_byte=40;
   18,19,20,21:file_byte=w>>((a-18)*8);
   22,23,24,25:begin field=address>=1000 ? -h : h; file_byte=field>>((a-22)*8); end
   26:file_byte=1; 28:file_byte=24;
   default:if(a>=54 && a<len)file_byte=8'h80;
  endcase
 end
endfunction
always @(posedge clk) begin
 if(rst)begin state<=0; sd_sec_read_data_valid<=0; sd_sec_read_end<=0; sd_sec_read_data<=0; end
 else begin
  sd_sec_read_data_valid<=0; sd_sec_read_end<=0;
  case(state)
   0:if(sd_sec_read)begin addr<=sd_sec_read_addr; b<=0; state<=1; end
   1:begin sd_sec_read_data<=file_byte(addr,b); sd_sec_read_data_valid<=1;
      if(b==511)state<=2; else b<=b+1; end
   2:begin sd_sec_read_end<=1; state<=3; end
   3:if(!sd_sec_read)state<=0;
  endcase
 end
end
endmodule
