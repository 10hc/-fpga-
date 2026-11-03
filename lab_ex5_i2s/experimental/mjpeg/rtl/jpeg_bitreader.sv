// jpeg_bitreader.sv - turns the parser's un-stuffed byte tokens into a bit stream (T.81 F.2.2.5).
//
// Bits are delivered MSB first, one per clock, from the head of a small token FIFO.
// A marker token at the head means the entropy-coded segment (or restart interval) is
// exhausted: further bit requests are answered with zeros (like libjpeg's "premature end
// of data" padding) and flagged on err_pad.  On restart_req the residual bits of the
// current byte are dropped and the RSTn marker token is consumed (F.2.2.5 / B.2.4.4).
module jpeg_bitreader (
  input  logic       clk,
  input  logic       rst,
  input  logic       clear,          // drop everything (new frame)
  // tokens from the parser
  input  logic       tok_valid,
  input  logic [8:0] tok_data,
  output logic       tok_ready,
  // bit interface
  output logic       bit_valid,
  output logic       bit_data,
  input  logic       bit_take,
  // restart handling
  input  logic       restart_req,    // held high until restart_done
  output logic       restart_done,   // one-cycle pulse
  output logic       err_pad,        // one-cycle pulse: bits requested beyond a marker
  output logic       at_end          // a non-RST marker (normally EOI) is at the head: segment over
);
  localparam int DEPTH = 4;
  localparam logic [2:0] DEPTH3 = 3'd4;

  logic [8:0] q [0:DEPTH-1];
  logic [2:0] cnt;                    // number of valid entries, q[0] is the head
  logic [2:0] pos;                    // next bit of q[0] to deliver (0 = MSB)

  logic head_valid, head_is_marker, head_is_rst;
  logic push, pop;

  assign head_valid     = (cnt != 3'd0);
  assign head_is_marker = head_valid & q[0][8];
  assign head_is_rst    = head_is_marker & (q[0][7:3] == 5'b11010);
  assign at_end         = head_is_marker & ~head_is_rst;
  assign tok_ready      = (cnt != DEPTH3) | pop;      // a pop frees a slot in the same cycle
  assign push           = tok_valid & tok_ready;

  // bit output: data byte at the head -> its bits; marker at the head -> zero padding
  assign bit_valid = head_valid & ~restart_req;
  assign bit_data  = head_is_marker ? 1'b0 : q[0][3'd7 - pos];

  always_comb begin
    pop          = 1'b0;
    restart_done = 1'b0;
    err_pad      = 1'b0;
    if (restart_req) begin
      if (head_valid) begin
        if (!head_is_marker)      pop = 1'b1;                 // drop padding / stray data bytes
        else if (head_is_rst)     begin pop = 1'b1; restart_done = 1'b1; end
        else                      begin restart_done = 1'b1; err_pad = 1'b1; end   // EOI where RSTn expected
      end
    end else if (bit_take & head_valid) begin
      if (head_is_marker)         err_pad = 1'b1;
      else if (pos == 3'd7)       pop = 1'b1;
    end
  end

  integer i;
  always_ff @(posedge clk) begin
    if (rst | clear) begin
      cnt <= '0; pos <= '0;
    end else begin
      // bit position within the head byte
      if (restart_req)                                   pos <= '0;
      else if (bit_take & head_valid & ~head_is_marker)  pos <= pos + 3'd1;   // wraps 7 -> 0 with the pop

      // FIFO: shift on pop, append on push
      if (pop) begin
        for (i = 0; i < DEPTH-1; i = i + 1) q[i] <= q[i+1];
      end
      if (push) begin
        if (pop) q[cnt - 3'd1] <= tok_data;
        else     q[cnt]        <= tok_data;
      end
      cnt <= cnt + {2'd0, push} - {2'd0, pop};
    end
  end
endmodule
