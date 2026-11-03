// jpeg_sdp_ram.sv - simple dual-port RAM (one write port, one registered read port).
// Written in the canonical style that Quartus II infers into M4K blocks and that the
// simulator handles directly.  OUT_REG = 1 adds the block RAM's output register (read data
// two clocks after the address) to take the RAM access out of the logic paths behind it.
module jpeg_sdp_ram #(
  parameter int WIDTH = 8,
  parameter int DEPTH_LOG2 = 6,
  parameter int DEPTH = 1 << DEPTH_LOG2,     // may be smaller than 2**DEPTH_LOG2 (non-power-of-two RAMs)
  parameter bit OUT_REG = 1'b0
) (
  input  logic                  clk,
  input  logic                  we,
  input  logic [DEPTH_LOG2-1:0] waddr,
  input  logic [WIDTH-1:0]      wdata,
  input  logic [DEPTH_LOG2-1:0] raddr,
  output logic [WIDTH-1:0]      rdata
);
  (* ramstyle = "M4K" *) logic [WIDTH-1:0] mem [0:DEPTH-1];   // Quartus: never fall back to logic cells
  logic [WIDTH-1:0] q;
  always_ff @(posedge clk) begin
    if (we) mem[waddr] <= wdata;
    q <= mem[raddr];
  end
  generate
    if (OUT_REG) begin : g_oreg
      always_ff @(posedge clk) rdata <= q;
    end else begin : g_q
      assign rdata = q;
    end
  endgenerate
endmodule
