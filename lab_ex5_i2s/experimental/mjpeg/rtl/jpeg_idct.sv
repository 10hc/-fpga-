// jpeg_idct.sv - 8x8 inverse DCT, a bit-exact port of libjpeg's jpeg_idct_islow (jidctint.c,
// the Loeffler/Ligtenberg/Moschytz algorithm, CONST_BITS=13, PASS1_BITS=2, RANGE_BITS=2).
//
// Pass 1 transforms the 8 columns of the dequantised block into a 16-bit workspace,
// pass 2 transforms the 8 rows of the workspace into range-limited 8-bit samples
// (the +128 level shift of T.81 A.3.1 is folded into pass 2 exactly as libjpeg does).
// One 8-point 1-D transform is computed combinationally (12 multiplications); each
// 8-point vector costs 8 read + 2 compute + 8 write cycles, so a block takes ~300 cycles.
// Based in part on the work of the Independent JPEG Group: the arithmetic reproduces libjpeg /
// libjpeg-turbo so that the output is bit-identical to them (see NOTICE.md).
module jpeg_idct (
  input  logic        clk,
  input  logic        rst,
  input  logic        start,            // pulse: the block RAM holds a complete block
  output logic        done,             // pulse: all 64 samples have been written
  output logic        busy,
  output logic        pass2_start,      // pulse: pass 1 finished, the block RAM is no longer read
  // dequantised coefficient block, natural order (registered read, 1-cycle latency)
  output logic [5:0]  blk_raddr,
  input  logic signed [15:0] blk_rdata,
  // 8-bit samples out (row-major index r*8+c)
  output logic        smp_we,
  output logic [5:0]  smp_waddr,
  output logic [7:0]  smp_wdata
);
  localparam int CONST_BITS = 13;
  localparam int PASS1_BITS = 2;

  typedef enum logic [2:0] { IDLE, GATHER, COMPUTE, COMPUTE2, WRITE } state_t;
  state_t state;
  logic       pass2;                   // 0 = columns (pass 1), 1 = rows (pass 2)
  logic [2:0] vec;                     // column (pass 1) or row (pass 2) being processed
  logic [3:0] step;

  logic signed [15:0] x [0:7];         // gathered inputs
  logic signed [31:0] y [0:7];         // 1-D results (before descale)

  // workspace RAM (pass 1 -> pass 2), 64 x 16, registered read
  logic        ws_we;
  logic [5:0]  ws_waddr, ws_raddr;
  logic signed [15:0] ws_wdata, ws_rdata;
  jpeg_sdp_ram #(.WIDTH(16), .DEPTH_LOG2(6)) u_ws (
    .clk(clk), .we(ws_we), .waddr(ws_waddr), .wdata(ws_wdata), .raddr(ws_raddr), .rdata(ws_rdata));

  // read addressing: column vec, element step (pass 1) / row vec, element step (pass 2)
  logic [2:0] elem;
  assign elem      = step[2:0];
  assign blk_raddr = {elem, vec};      // 8*r + c with r = elem, c = vec
  assign ws_raddr  = {vec, elem};      // 8*r + c with r = vec,  c = elem

  // ---------------------------------------------------------------- 1-D transform (combinational)
  // All arithmetic mirrors the INT32 arithmetic of jidctint.c.  Multiplier operands are kept
  // at <= 18 bits so that each product maps onto one Cyclone II 18x18 DSP block.
  logic signed [17:0] xe [0:7];                     // sign-extended inputs
  logic signed [17:0] s26, s02, s13, s0213, s03, s12;
  (* multstyle = "dsp" *) logic signed [33:0] p_z1, p_t2, p_t3, p_zz1, p_zz2, p_zz3, p_w1, p_t0, p_t3o, p_w2, p_t1, p_t2o;
  logic signed [31:0] z2p, z3p, tmp0, tmp1, tmp2, tmp3, tmp10, tmp11, tmp12, tmp13;
  logic signed [31:0] zz1, zz2m, zz3m, w1, w2, t0m, t1m, t2m, t3m;
  // pipeline registers between the multipliers (stage 1, packed into the DSP output
  // registers) and the adder tree (stage 2)
  logic signed [31:0] r_z1, r_t2, r_t3, r_zz1, r_zz2, r_zz3, r_w1, r_t0, r_t3o, r_w2, r_t1, r_t2o, r_tmp0, r_tmp1;
  localparam logic signed [15:0] C_0_541 = 16'sd4433,  C_0_765 = 16'sd6270,  C_1_847 = 16'sd15137;
  localparam logic signed [15:0] C_1_175 = 16'sd9633,  C_0_298 = 16'sd2446,  C_1_501 = 16'sd12299;
  localparam logic signed [15:0] C_2_053 = 16'sd16819, C_3_072 = 16'sd25172;
  localparam logic signed [15:0] CN_1_961 = -16'sd16069, CN_0_390 = -16'sd3196;
  localparam logic signed [15:0] CN_0_899 = -16'sd7373,  CN_2_562 = -16'sd20995;

  integer j;
  always_comb begin
    for (j = 0; j < 8; j = j + 1) xe[j] = x[j];   // sign extension
    // ---- stage 1: even-part DC terms and the twelve products
    if (!pass2)
      z2p = (z2w(x[0]) <<< CONST_BITS) + 32'sd1024;                    // + ONE << (CONST_BITS-PASS1_BITS-1)
    else
      z2p = (z2w(x[0]) + 32'sd16384 + 32'sd16) <<< CONST_BITS;         // + (RANGE_CENTER<<(PASS1_BITS+3)) + (ONE<<(PASS1_BITS+2))
    z3p   = z2w(x[4]) <<< CONST_BITS;
    tmp0  = z2p + z3p;
    tmp1  = z2p - z3p;
    s26   = xe[2] + xe[6];
    p_z1  = s26   * C_0_541;                                            // FIX_0_541196100
    p_t2  = xe[2] * C_0_765;                                            // FIX_0_765366865
    p_t3  = xe[6] * C_1_847;                                            // FIX_1_847759065
    // odd part (t0..t3 = x7, x5, x3, x1)
    s02   = xe[7] + xe[3];
    s13   = xe[5] + xe[1];
    s0213 = s02 + s13;
    s03   = xe[7] + xe[1];
    s12   = xe[5] + xe[3];
    p_zz1 = s0213 * C_1_175;                                            //  FIX_1_175875602
    p_zz2 = s02   * CN_1_961;                                           // -FIX_1_961570560
    p_zz3 = s13   * CN_0_390;                                           // -FIX_0_390180644
    p_w1  = s03   * CN_0_899;                                           // -FIX_0_899976223
    p_t0  = xe[7] * C_0_298;                                            //  FIX_0_298631336
    p_t3o = xe[1] * C_1_501;                                            //  FIX_1_501321110
    p_w2  = s12   * CN_2_562;                                           // -FIX_2_562915447
    p_t1  = xe[5] * C_2_053;                                            //  FIX_2_053119869
    p_t2o = xe[3] * C_3_072;                                            //  FIX_3_072711026
    // ---- stage 2: adder tree on the registered products
    tmp2  = r_z1 + r_t2;
    tmp3  = r_z1 - r_t3;
    tmp10 = r_tmp0 + tmp2;  tmp13 = r_tmp0 - tmp2;
    tmp11 = r_tmp1 + tmp3;  tmp12 = r_tmp1 - tmp3;
    zz1   = r_zz1;
    zz2m  = r_zz2 + zz1;
    zz3m  = r_zz3 + zz1;
    w1    = r_w1;
    w2    = r_w2;
    t0m   = r_t0  + w1 + zz2m;
    t3m   = r_t3o + w1 + zz3m;
    t1m   = r_t1  + w2 + zz3m;
    t2m   = r_t2o + w2 + zz2m;
  end

  function automatic logic signed [31:0] z2w(input logic signed [15:0] v);
    z2w = v;                                      // sign extend
  endfunction

  // descale + saturate (pass 1) / descale + range limit (pass 2)
  function automatic logic signed [15:0] sat16(input logic signed [31:0] v);
    if (v > 32'sd32767)       sat16 = 16'sd32767;
    else if (v < -32'sd32768) sat16 = 16'sh8000;
    else                      sat16 = v[15:0];
  endfunction
  function automatic logic [7:0] range_limit(input logic signed [31:0] v);
    logic [9:0] m;
    m = v[9:0];                                   // (v) & RANGE_MASK
    if (m < 10'd384)      range_limit = 8'd0;
    else if (m < 10'd640) range_limit = m[7:0] - 8'd128;   // m - 384 (for 384 <= m < 640)
    else                  range_limit = 8'd255;
  endfunction

  logic signed [31:0] ysel;
  assign ysel = y[step[2:0]];
  assign ws_we    = (state == WRITE) & ~pass2;
  assign ws_waddr = {elem, vec};
  assign ws_wdata = sat16(ysel >>> (CONST_BITS - PASS1_BITS));
  assign smp_we    = (state == WRITE) & pass2;
  assign smp_waddr = {vec, elem};
  assign smp_wdata = range_limit(ysel >>> (CONST_BITS + PASS1_BITS + 3));

  assign busy = (state != IDLE);

  integer i;
  always_ff @(posedge clk) begin
    done <= 1'b0; pass2_start <= 1'b0;
    if (rst) begin
      state <= IDLE; pass2 <= 1'b0; vec <= '0; step <= '0;
      for (i = 0; i < 8; i = i + 1) begin x[i] <= '0; y[i] <= '0; end
    end else begin
      case (state)
        IDLE: if (start) begin pass2 <= 1'b0; vec <= '0; step <= '0; state <= GATHER; end
        GATHER: begin
          // address `step` is presented this cycle; data for `step-1` arrives now
          if (step != 4'd0) x[step[2:0] - 3'd1] <= pass2 ? ws_rdata : blk_rdata;
          if (step == 4'd8) begin state <= COMPUTE; step <= '0; end
          else step <= step + 4'd1;
        end
        COMPUTE: begin
          r_z1 <= p_z1[31:0]; r_t2 <= p_t2[31:0]; r_t3 <= p_t3[31:0];
          r_zz1 <= p_zz1[31:0]; r_zz2 <= p_zz2[31:0]; r_zz3 <= p_zz3[31:0];
          r_w1 <= p_w1[31:0]; r_t0 <= p_t0[31:0]; r_t3o <= p_t3o[31:0];
          r_w2 <= p_w2[31:0]; r_t1 <= p_t1[31:0]; r_t2o <= p_t2o[31:0];
          r_tmp0 <= tmp0; r_tmp1 <= tmp1;
          state <= COMPUTE2;
        end
        COMPUTE2: begin
          y[0] <= tmp10 + t3m;  y[7] <= tmp10 - t3m;
          y[1] <= tmp11 + t2m;  y[6] <= tmp11 - t2m;
          y[2] <= tmp12 + t1m;  y[5] <= tmp12 - t1m;
          y[3] <= tmp13 + t0m;  y[4] <= tmp13 - t0m;
          step <= '0; state <= WRITE;
        end
        WRITE: begin
          if (step == 4'd7) begin
            step <= '0;
            if (vec == 3'd7) begin
              vec <= '0;
              if (pass2) begin state <= IDLE; done <= 1'b1; end
              else begin pass2 <= 1'b1; pass2_start <= 1'b1; state <= GATHER; end
            end else begin
              vec <= vec + 3'd1; state <= GATHER;
            end
          end else step <= step + 4'd1;
        end
        default: state <= IDLE;
      endcase
    end
  end
endmodule
