// MJP1 sector source, single clock domain. Intended to run with jpeg_video_pipe
// at 50MHz; the existing board's SD port is 100MHz and needs a sector bridge.
// Only whole, exactly 512-byte reads enter the parser. A raw sector end drops
// the physical request immediately (the SD controller accepts request LEVELS).
// Frame start precedes JPEG data. Consumer supplies next_frame only after the
// decoder, scaler and frame writer have finished/aborted the current image.
module mjpeg_sector_source #(
 parameter [23:0] READ_TIMEOUT=24'hffffff
)(
 input clk,rst,enable,input [31:0] file_sector,
 output reg sec_read,output reg [31:0] sec_read_addr,
 input [7:0] sec_data,input sec_valid,sec_end,
 output reg frame_start, output wire byte_valid, input byte_ready,
 output wire [7:0] byte_data, output wire byte_last,
 input next_frame,
 output wire frame_wait, output wire active,
 output reg [31:0] frame_index,output reg [15:0] fps,
 output reg [31:0] frame_pts,output reg [3:0] error,
 output wire [3:0] state_dbg
);
 localparam IDLE=0,FILE_REQ=1,RX=2,FILE_CHECK=3,FRAME_REQ=4,
            FRAME_CHECK=5,DATA_REQ=6,READ_BYTE=7,SEND_BYTE=8,WAIT_FRAME=9,HALT=10;
 localparam K_FILE=0,K_FRAME=1,K_DATA=2;
 reg [3:0] state;
 reg [1:0] kind;
 reg [7:0] sector_mem[0:511];
 reg [7:0] q;
 reg [9:0] received;
 reg [8:0] send_idx;
 reg [159:0] header;
 reg [31:0] base,offset,total_sectors,total_frames,remaining;
 reg [23:0] timeout;
 wire [10:0] received_total={1'b0,received}+(sec_valid ? 11'd1:11'd0);
 wire [31:0] length=header[63:32];
 wire [31:0] rounded_sectors={9'd0,length[31:9]}+(|length[8:0]);
 wire [32:0] frame_end={1'b0,offset}+33'd1+{1'b0,rounded_sectors};
 wire [32:0] physical_end={1'b0,base}+{1'b0,total_sectors};
 assign active=state!=IDLE && state!=HALT;
 assign state_dbg=state;
 assign frame_wait=state==WAIT_FRAME;
 assign byte_valid=state==SEND_BYTE && enable;
 assign byte_data=q;
 assign byte_last=remaining==1;
 always @(posedge clk)begin
  if(state==RX && sec_valid && received<512)sector_mem[received[8:0]]<=sec_data;
  q<=sector_mem[send_idx];
 end
 always @(posedge clk or posedge rst)begin
  if(rst)begin
   state<=IDLE;kind<=K_FILE;sec_read<=0;sec_read_addr<=0;
   frame_start<=0;received<=0;send_idx<=0;header<=0;
   base<=0;offset<=0;total_sectors<=0;total_frames<=0;
   remaining<=0;frame_index<=0;fps<=0;frame_pts<=0;
   error<=0;timeout<=0;
  end else begin
   frame_start<=0;
   if(!enable && state!=RX)begin state<=IDLE;sec_read<=0;end
   else case(state)
    IDLE:if(enable)begin base<=file_sector;offset<=0;frame_index<=0;error<=0;state<=FILE_REQ;end
    FILE_REQ,FRAME_REQ,DATA_REQ:begin
     kind<=state==FILE_REQ?K_FILE:state==FRAME_REQ?K_FRAME:K_DATA;
     sec_read_addr<=base+offset;sec_read<=1;received<=0;
     header<=0;timeout<=0;state<=RX;
    end
    RX:begin
     if(sec_valid)begin
      if(received!=1023)received<=received+1'b1;
      if(received<20)header[received*8 +:8]<=sec_data;
     end
     if(sec_end)begin
      sec_read<=0;
      if(!enable)state<=IDLE;
      else if(received_total!=512)begin error<=3;state<=HALT;end
      else if(kind==K_FILE)state<=FILE_CHECK;
      else if(kind==K_FRAME)state<=FRAME_CHECK;
      else begin send_idx<=0;state<=READ_BYTE;end
     end else if(timeout==READ_TIMEOUT)begin sec_read<=0;error<=4;state<=HALT;end
     else timeout<=timeout+1'b1;
    end
    FILE_CHECK:begin
     if(header[31:0]!=32'h31504a4d || header[47:32]!=320 || header[63:48]!=240 ||
        header[79:64]==0 || header[79:64]>60 || header[95:80]!=1 ||
        header[127:96]==0 || header[159:128]<3 ||
        header[127:96]>((header[159:128]-1)>>1))begin error<=1;state<=HALT;end
     else if(({1'b0,base}+{1'b0,header[159:128]})>33'h100000000)begin error<=5;state<=HALT;end
     else begin
      fps<=header[79:64];total_frames<=header[127:96];total_sectors<=header[159:128];
      offset<=1;state<=FRAME_REQ;
     end
    end
    FRAME_CHECK:begin
     if(physical_end>33'h100000000 || frame_end>{1'b0,total_sectors})begin error<=5;state<=HALT;end
     else if(header[31:0]!=32'h3147504a || length==0 || length>131072 ||
             header[95:64]!=frame_index ||
             (frame_index==total_frames-1 && frame_end!={1'b0,total_sectors}) ||
             (frame_index<total_frames-1 && frame_end+2>{1'b0,total_sectors}))begin error<=2;state<=HALT;end
     else begin
      frame_pts<=header[127:96];remaining<=length;
      frame_start<=1;offset<=offset+1'b1;state<=DATA_REQ;
     end
    end
    READ_BYTE:state<=SEND_BYTE;
    SEND_BYTE:if(byte_ready)begin
     remaining<=remaining-1'b1;
     if(remaining==1)begin offset<=offset+1'b1;state<=WAIT_FRAME;end
     else if(send_idx==511)begin offset<=offset+1'b1;state<=DATA_REQ;end
     else begin send_idx<=send_idx+1'b1;state<=READ_BYTE;end
    end
    WAIT_FRAME:if(next_frame)begin
     if(frame_index==total_frames-1)begin frame_index<=0;offset<=1;end
     else frame_index<=frame_index+1'b1;
     state<=FRAME_REQ;
    end
    HALT:sec_read<=0;
    default:begin error<=15;state<=HALT;sec_read<=0;end
   endcase
  end
 end
endmodule
