`timescale 1ns/1ps
// The REAL SD sector sequencer is level-sensitive in S_WAIT_READ_WRITE.
// Model only its command/byte layer; do not substitute an edge-triggered SD
// client model, which hides duplicate requests on READ_END -> WAIT.
module tb_video_sd_level;
reg clk=0,rst=1,enable=0;
always #5 clk=~clk;
wire req,wr_req,wr_en,display_valid,active,abort;
wire [31:0] lba,pixel;
wire [3:0] st;
wire [1:0] di,wi;
wire [7:0] sd_data;
wire sd_valid,sd_end,initialized;
wire cmd_req,block_req;
wire [47:0] cmd;
reg cmd_ack=0,block_valid=0,block_ack=0;
reg [7:0] block_data=0;
reg ack=0,finish_toggle=0;
reg ready=1;
integer command_delay=0,command_reads=0,blocks=0,words=0;
integer byte_index=0,byte_gap=0,byte_state=0;
integer gap_clocks=32;
integer sessions=0;
reg [31:0] current_lba=0;
reg test_done=0;
localparam [31:0] BASE=32'd47984;
video_player dut(.clk(clk),.rst(rst),.enable(enable),.file_valid(1'b1),.file_sector(BASE),
 .sec_read(req),.sec_read_addr(lba),.sec_read_data(sd_data),.sec_read_data_valid(sd_valid),
 .sec_read_end(sd_end),.wr_req(wr_req),.wr_req_ack(ack),.write_ready(ready),
 .wr_en(wr_en),.wr_data(pixel),.wr_finish(finish_toggle),.wr_abort(abort),
 .display_valid(display_valid),.disp_buf_idx(di),.write_buf_idx(wi),.active(active),.st_dbg(st));
sd_card_sec_read_write #(.SPI_LOW_SPEED_DIV(0)) sector (
 .clk(clk),.rst(rst),.sd_init_done(initialized),.sd_sec_read(req),.sd_sec_read_addr(lba),
 .sd_sec_read_data(sd_data),.sd_sec_read_data_valid(sd_valid),.sd_sec_read_end(sd_end),
 .sd_sec_write(1'b0),.sd_sec_write_addr(32'd0),.sd_sec_write_data(8'd0),
 .sd_sec_write_data_req(),.sd_sec_write_end(),.spi_clk_div(),
 .cmd_req(cmd_req),.cmd_req_ack(cmd_ack),.cmd_req_error(1'b0),.cmd(cmd),.cmd_r1(),.cmd_data_len(),
 .block_read_req(block_req),.block_read_valid(block_valid),.block_read_data(block_data),
 .block_read_req_ack(block_ack),.block_write_req(),.block_write_data(),
 .block_write_data_rd(1'b0),.block_write_req_ack(1'b0));
function [7:0] value;
 input [31:0] address; input integer idx;
 integer source_word;
 begin
  source_word=(address-BASE-1)*256+idx/2;
  value=idx[0] ? source_word[15:8] : source_word[7:0];
  if(address==BASE)case(idx)
   0:value="V";1:value="I";2:value="D";3:value="1";
   4:value=8'h40;5:value=1;6:value=8'hf0;7:value=0;8:value=25;10:value=200;
   default:value=0;
  endcase
 end
endfunction
function [31:0] expected_pixel;
 input integer n;
 integer src;reg [15:0] p;
 begin
  src=((n/640)/2)*320+((n%640)/2);p=src;
  expected_pixel={p[15:11],p[15:13],p[10:5],p[10:9],p[4:0],p[4:2],8'd0};
 end
endfunction
always @(posedge clk) begin
 if(rst)begin cmd_ack<=0;command_delay<=0;command_reads<=0;end
 else begin
  cmd_ack<=0;
  if(!cmd_req)command_delay<=0;
  else if(command_delay==3)begin cmd_ack<=1;command_delay<=4;end
  else if(command_delay<3)command_delay<=command_delay+1;
  if(cmd_ack && cmd_req && cmd[45:40]==17)begin
   if(cmd[39:8] !== (command_reads<2 ? BASE : BASE+command_reads-1))
    $fatal(1,"DUPLICATE_OR_SKIPPED_SECTOR read=%0d got=%0d expected=%0d video_state=%0d",command_reads,cmd[39:8],command_reads<2?BASE:BASE+command_reads-1,st);
   current_lba<=cmd[39:8];command_reads<=command_reads+1;
  end
 end
end
always @(posedge clk)begin
 block_valid<=0;block_ack<=0;
 if(rst)begin byte_state<=0;byte_index<=0;byte_gap<=0;blocks<=0;end
 else case(byte_state)
  0:if(block_req)begin byte_state<=1;byte_index<=0;byte_gap<=0;blocks<=blocks+1;end
  1:if(byte_gap==0)begin
    if(byte_index<512)begin block_data<=value(current_lba,byte_index);block_valid<=1;end
    if(byte_index==513)begin block_ack<=1;byte_state<=2;end
    else begin byte_index<=byte_index+1;byte_gap<=gap_clocks;end
   end else byte_gap<=byte_gap-1;
  2:if(!block_req)byte_state<=0;
 endcase
end
always @(posedge clk)begin
 if(rst)begin ack<=0;words<=0;finish_toggle<=0;end
 else begin
  ack<=wr_req;
  if(abort)$fatal(1,"VIDEO_HALTED words=%0d reads=%0d bytes=%0d",words,command_reads,dut.sector_bytes);
  if(wr_en)begin
   if(pixel!==expected_pixel(words))$fatal(1,"PIXEL_MISMATCH n=%0d got=%h expected=%h",words,pixel,expected_pixel(words));
   words<=words+1;
   if(words==307199)finish_toggle<=~finish_toggle;
  end
 end
end
initial begin
 if($value$plusargs("GAP=%d",gap_clocks))begin end
 repeat(8)@(negedge clk);rst=0;
 // Skip only power-on delay. All initialization and read states are real RTL.
 force sector.por_cnt=20'd1000000;
 wait(initialized);release sector.por_cnt;
 repeat(2)begin
  @(negedge clk);enable=1;
  wait(display_valid);
  if(words!=307200 || command_reads!=302)$fatal(1,"Wrong totals words=%0d reads=%0d",words,command_reads);
  sessions=sessions+1;
  $display("PASS VIDEO SESSION gap=%0d session=%0d pixels=%0d sector_commands=%0d",gap_clocks,sessions,words,command_reads);
  @(negedge clk);enable=0;
  wait(!active && !display_valid && !req && !block_req);
  repeat(12)@(negedge clk);
  words=0;command_reads=0;
 end
 test_done=1;$display("VIDEO_SD_LEVEL_PASS gap=%0d sessions=%0d exact_pixels=614400 sector_commands=604",gap_clocks,sessions);$finish;
end
initial begin #500000000;$fatal(1,"Timeout st=%0d words=%0d",st,words);end
endmodule
