`timescale 1ns/1ps
// Byte/frame scoreboard against the real wav_audio, not a replacement parser.
module tb_wav_riff;
reg sd_clk=0, vid_clk=0, rst=1, enable=0, file_valid=1;
always #5 sd_clk=~sd_clk;
always #20 vid_clk=~vid_clk;
reg [31:0] file_sector=100;
wire req, playing, active, aud_valid;
wire [31:0] addr;
wire [23:0] aud_left,aud_right;
wire [15:0] sum16;
reg [7:0] data=0;
reg valid=0, ending=0;
wav_audio #(.SD_CLK_HZ(4800000),.VID_CLK_HZ(1200000),
            .SCAN_START(98),.SCAN_MAX(102)) dut (
 .sd_clk(sd_clk),.sd_rst(rst),.vid_clk(vid_clk),.vid_rst(rst),.enable(enable),
 .file_valid(file_valid),.file_sector(file_sector),
 .sec_read(req),.sec_read_addr(addr),.sec_read_data(data),
 .sec_read_data_valid(valid),.sec_read_end(ending),
 .aud_valid(aud_valid),.aud_left(aud_left),.aud_right(aud_right),
 .playing(playing),.active(active),.sum16(sum16),.sum_ok());

reg [7:0] image[0:65535];
integer size, pcm_start, pcm_bytes, frames, target, writes, reads, requests;
integer cases=0, i, pos, n, junk, fmtlen, last_pcm_lba;
integer sdstate=0, off=0, tx_lba=0, limit=512, attempts=0;
integer fault_kind=0, fault_lba=100, gap=0, delay_count=0, stall_count=0;
integer paused_writes, paused_requests;
reg simultaneous=0, monitor=0, reject_case=0, saw_full=0;
reg test_done=0;
reg [31:0] expected;
wire [31:0] sampled_word = dut.sample_q;
integer checksum_expected;

function [31:0] frame_word;
 input integer frame;
 reg [15:0] left,right;
 begin
  left=16'h8101 + frame*37; right=16'h3f02 - frame*19;
  frame_word={right,left};
 end
endfunction
task le16;
 input integer at,value;
 begin image[at]=value;image[at+1]=value>>8;end
endtask
task le32;
 input integer at,value;
 begin image[at]=value;image[at+1]=value>>8;image[at+2]=value>>16;image[at+3]=value>>24;end
endtask
task fourcc;
 input integer at;
 input [31:0] id;
 begin image[at]=id[31:24];image[at+1]=id[23:16];image[at+2]=id[15:8];image[at+3]=id[7:0];end
endtask
task make_image;
 input integer bytes,junk_bytes,fmt_bytes;
 begin
  for(i=0;i<65536;i=i+1) image[i]=8'hee;
  fourcc(0,"RIFF"); fourcc(8,"WAVE");pos=12;
  if(junk_bytes>=0) begin
   fourcc(pos,"JUNK");le32(pos+4,junk_bytes);pos=pos+8+junk_bytes+(junk_bytes%2);
  end
  fourcc(pos,"fmt ");le32(pos+4,fmt_bytes);
  le16(pos+8,1);le16(pos+10,2);le32(pos+12,48000);le32(pos+16,192000);
  le16(pos+20,4);le16(pos+22,16);pos=pos+8+fmt_bytes+(fmt_bytes%2);
  fourcc(pos,"LIST");le32(pos+4,3);pos=pos+12; // odd chunk + pad
  fourcc(pos,"data");le32(pos+4,bytes);pcm_start=pos+8;
  pcm_bytes=bytes;frames=bytes/4;
  for(i=0;i<bytes;i=i+1) begin
   expected=frame_word(i/4);image[pcm_start+i]=expected>>((i%4)*8);
  end
  pos=pcm_start+bytes+(bytes%2);
  fourcc(pos,"LIST");le32(pos+4,8);pos=pos+16; // must never enter FIFO
  size=pos;le32(4,size-8);
  last_pcm_lba=100+(pcm_start+bytes-1)/512;
  checksum_expected=0;
  for(i=512;i<2560;i=i+1)checksum_expected=(checksum_expected+image[i])&65535;
 end
endtask

// Sector transaction model: held request, byte gaps, short/oversize reads,
// optional last-byte+end in the same cycle. A request is accepted once only.
always @(negedge sd_clk) begin
 if(rst) begin
  sdstate=0;off=0;valid=0;ending=0;attempts=0;requests=0;delay_count=0;
 end else begin
  valid=0;ending=0;
  case(sdstate)
   0: if(req) begin
    tx_lba=addr;off=0;requests=requests+1;limit=512;
    if(!reject_case && (addr<98 || addr>last_pcm_lba))
     $fatal(1,"Read past PCM range: LBA=%0d last=%0d",addr,last_pcm_lba);
    if(tx_lba==fault_lba && attempts==0 && fault_kind!=0) begin
     attempts=attempts+1;
     if(fault_kind==1 || fault_kind==3 || fault_kind==4)limit=100;
     if(fault_kind==2)limit=513;
    end
    sdstate=1;
   end
   1: begin
    if(gap!=0 && delay_count<gap)delay_count=delay_count+1;
    else begin
     delay_count=0;valid=1;
     if(tx_lba<100)data=8'hff;
     else if((tx_lba-100)*512+off<65536)data=image[(tx_lba-100)*512+off];
     else data=8'hdd;
     off=off+1;
     if(off==limit)begin
      if((fault_kind==3 || fault_kind==4) && tx_lba==fault_lba && limit==100)begin
       sdstate=fault_kind==3 ? 5 : 6;stall_count=0;
      end else if(simultaneous)begin ending=1;sdstate=3;end
      else sdstate=2;
     end
    end
   end
   2: begin ending=1;sdstate=3;end
   3: if(!req)sdstate=0;else sdstate=4;
   4: if(!req)sdstate=0;
   5: begin
    stall_count=stall_count+1;
    if(stall_count>2000020)begin ending=1;sdstate=3;end
   end
   6: if(!req)sdstate=0;
  endcase
 end
end

always @(posedge sd_clk) if(!rst && monitor)begin
 if(dut.buf_cnt==4095)saw_full=1;
 if(dut.sbuf_we)begin
  if(reject_case)$fatal(1,"Rejected WAV wrote PCM");
  expected=frame_word(writes%frames);
  if({dut.sector_q,dut.r_low,dut.l16}!==expected)
   $fatal(1,"PCM mismatch frame=%0d got=%h expected=%h start=%0d",writes,
          {dut.sector_q,dut.r_low,dut.l16},expected,pcm_start);
  writes=writes+1;
 end
 if(dut.sbuf_re)begin
  expected=frame_word(reads%frames);
  if(sampled_word!==expected)$fatal(1,"FIFO output mismatch frame=%0d got=%h expected=%h",reads,sampled_word,expected);
  reads=reads+1;
 end
end
always @(negedge vid_clk) if(!rst && aud_valid)begin
 if((^aud_left)===1'bx || (^aud_right)===1'bx)$fatal(1,"Unknown audio after reset/underflow");
end

task reset_case;
 begin
  @(negedge sd_clk);enable=0;rst=1;monitor=0;
  repeat(6)@(negedge sd_clk);
  writes=0;reads=0;saw_full=0;
  @(negedge sd_clk);rst=0;enable=1;monitor=1;
 end
endtask
task accept_case;
 input [255:0] name;
 input integer loops;
 begin
  reject_case=0;target=frames*loops;
  reset_case;
  wait(writes>=target);
  @(negedge sd_clk);enable=0;
  wait(!active);repeat(4)@(negedge sd_clk);
  if(!playing || writes<target || reads==0)$fatal(1,"Accepted case did not play");
  cases=cases+1;
  $display("PASS %0s start=%0d PCM=%0d frames_written=%0d read=%0d requests=%0d",name,pcm_start,pcm_bytes,writes,reads,requests);
 end
endtask
task invalid_case;
 input [255:0] name;
 begin
  reject_case=1;reset_case;
  wait(dut.st==5);repeat(30)@(negedge sd_clk);
  if(writes!=0 || playing || req)$fatal(1,"Invalid format accepted");
  enable=0;cases=cases+1;$display("PASS reject %0s",name);
 end
endtask

initial begin
 // Standard 44-byte header, including all 400 first-sector bytes.
 make_image(400,-1,16);
 // Remove the LIST preceding data to produce canonical 44-byte WAV.
 for(i=36;i<size-12;i=i+1)image[i]=image[i+12];
 pcm_start=44;size=size-12;le32(4,size-8);last_pcm_lba=100;
 accept_case("canonical_short_400",3);
 make_image(800,-1,16);
 for(i=36;i<size-12;i=i+1)image[i]=image[i+12];
 pcm_start=44;size=size-12;le32(4,size-8);last_pcm_lba=101;
 accept_case("canonical_cross_sector_800",3);
 make_image(800,-1,16);accept_case("cross_sector_800",3);
 make_image(4,-1,16);accept_case("single_frame",4);
 // data starts at offsets 0, 2, 510: frame/chunk boundaries cross sectors.
 make_image(800,448,16);accept_case("data_at_sector_zero",3);
 make_image(800,450,16);accept_case("PCM_frame_split_at_2",3);
 make_image(800,446,16);accept_case("data_header_split_PCM_at_510",3);
 make_image(400,477,16);accept_case("fmt_header_crosses_sector",3);
 make_image(800,480,16);accept_case("fmt_payload_crosses_sector",3);
 make_image(800,1301,41);accept_case("large_odd_JUNK_extended_fmt",3);
 make_image(456,-1,16);accept_case("data_ends_on_sector_boundary",3);
 simultaneous=1;make_image(800,446,18);accept_case("same_cycle_last_byte_end",3);
 simultaneous=0;gap=2;make_image(800,3,16);accept_case("gapped_SD_valid",3);gap=0;
 fault_kind=1;fault_lba=100;make_image(800,-1,16);accept_case("short_first_sector_retry",3);
 fault_lba=101;make_image(1600,446,16);accept_case("short_PCM_sector_retry",3);
 fault_kind=2;make_image(1600,446,16);accept_case("oversize_sector_retry",3);fault_kind=0;
 fault_kind=3;make_image(1600,446,16);accept_case("timeout_drain_then_end",3);
 fault_kind=4;make_image(1600,446,16);accept_case("timeout_missing_end_recovery",3);fault_kind=0;
 file_valid=0;make_image(800,5,16);accept_case("raw_scan_skips_garbage",3);file_valid=1;
 make_image(2400,446,16);reject_case=0;reset_case;
 wait(writes>=30);@(negedge sd_clk);enable=0;
 wait(!active);@(negedge sd_clk);paused_writes=writes;paused_requests=requests;
 repeat(500)@(negedge sd_clk);
 if(writes!=paused_writes || requests!=paused_requests)$fatal(1,"Paused WAV continued reading");
 enable=1;wait(writes>=frames*3);
 @(negedge sd_clk);enable=0;wait(!active);
 cases=cases+1;$display("PASS pause_resume_mid_sector frames=%0d",writes);
 // More than FIFO capacity; verify backpressure, wrap and PCM order.
 make_image(24000,3,16);accept_case("FIFO_full_and_pointer_wrap",2);
 if(!saw_full)$fatal(1,"FIFO pressure test never reached full");
 if(sum16!==checksum_expected[15:0])$fatal(1,"Legacy sector checksum mismatch %h/%h",sum16,checksum_expected[15:0]);
 make_image(400,-1,16);le16(20,3);invalid_case("non_PCM");
 make_image(400,-1,16);le16(22,1);invalid_case("mono");
 make_image(400,-1,16);le32(24,44100);invalid_case("wrong_rate");
 make_image(400,-1,16);le32(28,1234);invalid_case("wrong_byte_rate");
 make_image(400,-1,16);le16(32,2);invalid_case("wrong_block_align");
 make_image(400,-1,16);le16(34,24);invalid_case("24_bit");
 make_image(400,-1,16);le32(pcm_start-4,0);invalid_case("zero_data");
 make_image(400,-1,16);le32(pcm_start-4,399);invalid_case("incomplete_frame");
 make_image(400,-1,16);le32(pcm_start-4,65536);invalid_case("data_past_RIFF");
 make_image(400,-1,16);le32(16,15);invalid_case("short_fmt");
 make_image(400,3,16);le32(16,32'hffffffff);invalid_case("chunk_size_overflow");
 make_image(400,-1,16);le32(4,32'hffffffff);invalid_case("RIFF_size_overflow");
 make_image(400,-1,16);le32(4,3);invalid_case("RIFF_too_short");
 make_image(400,-1,16);le32(4,10);invalid_case("truncated_chunk_header");
 make_image(400,3,16);le32(4,15);invalid_case("missing_odd_chunk_pad");
 make_image(400,-1,16);fourcc(12,"data");invalid_case("data_before_fmt");
 make_image(400,-1,16);fourcc(0,"RF64");invalid_case("RF64");
 make_image(400,-1,16);fourcc(36,"JUNK");le32(40,468);le32(4,504);
 invalid_case("metadata_ends_at_sector_boundary_no_data");
 if(requests!=1)$fatal(1,"Read beyond RIFF without data");
 test_done=1;$display("WAV_RIFF_PASS cases=%0d",cases);$finish;
end
initial begin #200000000;$display("WAV test timeout st=%0d parser=%0d writes=%0d requests=%0d",dut.st,dut.parser,writes,requests);$fatal(1,"WAV test timeout");end
endmodule

module tb_wav_selftest;
reg clk=0,rst=1;
always #5 clk=~clk;
wire req,valid;
wire [23:0] left,right;
integer count=0;
reg test_done=0;
wav_audio #(.SELFTEST(2),.SD_CLK_HZ(4800000),.VID_CLK_HZ(4800000)) dut (
 .sd_clk(clk),.sd_rst(rst),.vid_clk(clk),.vid_rst(rst),.enable(1'b1),
 .file_valid(1'b0),.file_sector(32'd0),.sec_read(req),.sec_read_addr(),
 .sec_read_data(8'd0),.sec_read_data_valid(1'b0),.sec_read_end(1'b0),
 .aud_valid(valid),.aud_left(left),.aud_right(right),.playing(),.active(),.sum16(),.sum_ok());
always @(negedge clk)if(!rst)begin
 if(req)$fatal(1,"SELFTEST2 requested SD");
 if(valid)begin
  if((^left)===1'bx || (^right)===1'bx)$fatal(1,"SELFTEST2 unknown output");
  if(left!=right)$fatal(1,"SELFTEST2 channel mismatch");
  if(left!=0)count=count+1;
 end
end
initial begin
 repeat(5)@(negedge clk);rst=0;
 repeat(30000)@(negedge clk);
 if(count<100 || dut.wrp_cf==0 || dut.cf_ph==0)$fatal(1,"SELFTEST2 failed to advance");
 if(dut.s_l==0 || left!=dut.h_l)$fatal(1,"SELFTEST2 did not use buffered path");
 test_done=1;$display("WAV_SELFTEST2_PASS nonzero=%0d",count);$finish;
end
endmodule
