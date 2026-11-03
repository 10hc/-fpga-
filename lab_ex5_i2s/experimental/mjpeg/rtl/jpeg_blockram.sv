// jpeg_blockram.sv - 64 x 16 coefficient block buffer.
// A block usually has only a few non-zero coefficients (T.81 F.2.2.2 EOB), and the decoder
// writes only those.  So the buffer must read as all-zero at the start of every block: it is
// zeroed (64 clocks, one entry per clock) while the IDCT runs its second pass, which does not
// read this buffer and takes ~150 clocks, and once after reset.  Blocks whose IDCT is skipped
// are not written at all, so the buffer stays clean for the next one.
module jpeg_blockram (
  input  logic        clk,
  input  logic        rst,
  input  logic        zero_start,     // pulse: start zeroing (IDCT pass 2 has begun)
  input  logic        we,
  input  logic [5:0]  waddr,
  input  logic signed [15:0] wdata,
  input  logic [5:0]  raddr,
  output logic signed [15:0] rdata    // registered, 1-cycle latency
);
  logic       zeroing;
  logic [5:0] zaddr;
  always_ff @(posedge clk) begin
    if (rst || zero_start) begin
      zeroing <= 1'b1; zaddr <= '0;
    end else if (zeroing) begin
      zaddr <= zaddr + 6'd1;
      if (zaddr == 6'd63) zeroing <= 1'b0;
    end
  end
  jpeg_sdp_ram #(.WIDTH(16), .DEPTH_LOG2(6)) u_mem (
    .clk(clk), .we(we | zeroing), .waddr(zeroing ? zaddr : waddr), .wdata(zeroing ? 16'sd0 : wdata),
    .raddr(raddr), .rdata(rdata));
endmodule
