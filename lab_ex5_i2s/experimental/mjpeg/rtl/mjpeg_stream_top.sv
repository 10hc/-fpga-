// Experimental single-domain integration target (50MHz). Not the board top.
// The board adapter must bridge its 100MHz SD and frame-writer interfaces.
// next_frame is supplied only after done and frame-write completion; it may
// also skip a failed frame once any speculative writes have been aborted.
module mjpeg_stream_top(
 input wire clk,rst,enable,input wire [31:0] file_sector,
 output wire sec_read,output wire [31:0] sec_read_addr,
 input wire [7:0] sec_data,input wire sec_valid,sec_end,
 output wire out_valid,input wire out_ready,output wire [31:0] out_data,
 input wire next_frame,output wire frame_start,frame_done,frame_wait,busy,
 output wire [31:0] frame_index,frame_pts,output wire [15:0] fps,
 output wire [3:0] source_error,output wire [15:0] jpeg_error,
 output wire [18:0] words_sent
);
 wire byte_valid,byte_ready,byte_last,source_active,pipe_busy;
 wire [7:0] byte_data;
 wire [3:0] unused_state;
 mjpeg_sector_source source(
  .clk(clk),.rst(rst),.enable(enable),.file_sector(file_sector),
  .sec_read(sec_read),.sec_read_addr(sec_read_addr),.sec_data(sec_data),
  .sec_valid(sec_valid),.sec_end(sec_end),.frame_start(frame_start),
  .byte_valid(byte_valid),.byte_ready(byte_ready),.byte_data(byte_data),.byte_last(byte_last),
  .next_frame(next_frame),.frame_wait(frame_wait),.active(source_active),
  .frame_index(frame_index),.fps(fps),.frame_pts(frame_pts),.error(source_error),.state_dbg(unused_state));
 jpeg_video_pipe pipe(
  .clk(clk),.rst(rst || !enable || source_error!=0),.start(frame_start),
  .in_valid(byte_valid),.in_data(byte_data),.in_last(byte_last),.in_ready(byte_ready),
  .out_valid(out_valid),.out_ready(out_ready),.out_data(out_data),
  .busy(pipe_busy),.done(frame_done),.error(jpeg_error),.words_sent(words_sent));
 assign busy=source_active || pipe_busy;
endmodule
