// jpeg_coefdec.sv - decodes one 8x8 block of quantised DCT coefficients (T.81 F.2.2).
//
//   DECODE  (F.2.2.3, Figure F.16): bit-serial canonical Huffman decode using the
//           MAXCODE / (VALPTR-MINCODE) tables written by jpeg_parser, one bit per clock.
//   DC      (F.2.2.1): DIFF = EXTEND(RECEIVE(T), T); PRED += DIFF; ZZ(0) = PRED
//   AC      (F.2.2.2): run/size symbols RS, ZRL (F0) and EOB (00) handling
//   Dequantisation (F.2.2 / A.3.2 order): coefficient k (zig-zag) times Qk, written into the
//   block RAM at its natural position via the zig-zag table (Figure A.6).
//
// The dequantised value is saturated to 16 bits (libjpeg keeps a 32-bit product; real
// images from 8-bit sources never exceed ~13 bits, so results stay bit-exact for them).
module jpeg_coefdec
(
  input  logic        clk,
  input  logic        rst,
  // control
  input  logic        start,        // pulse: decode one block with the parameters below
  input  logic [1:0]  comp,         // frame component index (selects the DC predictor)
  input  logic        dc_tbl,       // Td
  input  logic        ac_tbl,       // Ta
  input  logic [1:0]  tq,           // Tq
  input  logic        pred_clear,   // pulse: reset the DC predictors (scan start / restart)
  output logic        done,         // pulse: block complete
  output logic        err_huff,     // pulse: no code matched in 16 bits or run past k=63
  // bit reader
  input  logic        bit_valid,
  input  logic        bit_data,
  output logic        bit_take,
  // table RAM read ports (1-cycle latency)
  output logic [5:0]  hc_raddr,     // {tc, th, L-1} -> {valid, MAXCODE+1, delta}
  input  logic [24:0] hc_rdata,
  output logic [8:0]  hv_raddr,     // HUFFVAL, folded: AC {th, index}, DC {th, 4'hF, index[3:0]}
  input  logic [7:0]  hv_rdata,
  output logic [7:0]  dqt_raddr,    // {tq, k} -> Qk
  input  logic [7:0]  dqt_rdata,
  // block RAM write port (natural order); the RAM reads as zero at block start (jpeg_blockram)
  output logic        blk_we,
  output logic [5:0]  blk_waddr,
  output logic signed [15:0] blk_wdata
);
  import jpeg_pkg::*;

  typedef enum logic [2:0] { IDLE, DEC, SYM, RECV, EXT, WRITE, FIN } state_t;
  state_t state;

  logic        phase_ac;            // 0 = decoding the DC coefficient, 1 = AC coefficients
  logic [5:0]  k;                   // zig-zag index of the coefficient being decoded
  logic [15:0] code;                // code bits gathered so far
  logic [4:0]  len;                 // number of bits in code
  logic [15:0] v;                   // RECEIVE value
  logic [3:0]  s;                   // SSSS: bits to receive
  logic [3:0]  nb;                  // bits received so far
  logic [1:0]  comp_r;
  logic        dc_tbl_r, ac_tbl_r;
  logic [1:0]  tq_r;
  logic [15:0] pred [0:3];          // DC predictors

  // Huffman table currently in use
  logic        tc, th;
  assign tc = phase_ac;
  assign th = phase_ac ? ac_tbl_r : dc_tbl_r;

  // decode step (combinational)
  logic        take;
  logic [15:0] code_n;
  logic [4:0]  len_n;
  logic        hc_valid;
  logic [15:0] hc_maxcode;
  logic [7:0]  hc_delta;
  logic        match;

  assign take       = (state == DEC) & bit_valid;
  assign bit_take   = take | ((state == RECV) & bit_valid);
  assign code_n     = {code[14:0], bit_data};
  assign len_n      = len + 5'd1;
  assign hc_valid   = hc_rdata[24];
  assign hc_maxcode = hc_rdata[23:8];
  assign hc_delta   = hc_rdata[7:0];
  assign match      = take & hc_valid & (code_n < hc_maxcode);   // hc_maxcode = MAXCODE+1 (see jpeg_parser)

  // RAM addresses: the code-table entry for the *next* code length must be on the output
  // when the next bit is taken, so address with the next-state length (L-1 = len_n when a
  // bit is being taken now, len otherwise).  At block start the DC table, length 1.
  logic [3:0] len_addr;
  always_comb begin
    len_addr = take ? len_n[3:0] : len[3:0];
    if (state == IDLE)      hc_raddr = {1'b0, dc_tbl, 4'd0};      // next: DC code, length 1
    else if (state == DEC)  hc_raddr = {tc, th, len_addr};
    else                    hc_raddr = {1'b1, ac_tbl_r, 4'd0};    // next: AC code, length 1
  end
  logic [7:0] hv_idx;
  assign hv_idx    = code_n[7:0] + hc_delta;
  assign hv_raddr  = {th, tc ? hv_idx : {4'hF, hv_idx[3:0]}};
  assign dqt_raddr = {tq_r, k};

  // EXTEND (F.2.2.1 Figure F.12)
  logic signed [15:0] v_ext;
  always_comb begin
    if (s == 4'd0)         v_ext = 16'sd0;
    else if (v[s - 4'd1])  v_ext = v;
    else                   v_ext = v - (16'd1 << s) + 16'd1;
  end

  // dequantise: EXT registers the (predicted) coefficient, WRITE multiplies by Qk and
  // registers the block write (address/data) - two short pipeline steps for timing.
  logic signed [15:0] coef, coef_r;
  (* multstyle = "dsp" *) logic signed [24:0] prod;
  logic signed [15:0] prod_sat;
  assign coef = phase_ac ? v_ext : (pred[comp_r] + v_ext);
  assign prod = coef_r * $signed({1'b0, dqt_rdata});
  always_comb begin
    if (prod > 25'sd32767)       prod_sat = 16'sd32767;
    else if (prod < -25'sd32768) prod_sat = 16'sh8000;
    else                         prod_sat = prod[15:0];
  end

  // zig-zag -> natural position as a registered ROM lookup (T.81 Figure A.6); k does not change
  // between SYM and WRITE, so the value is ready in WRITE.  Maps onto one small block RAM.
  (* romstyle = "M4K" *) logic [5:0] zz_nat;
  always_ff @(posedge clk) zz_nat <= zigzag_to_natural(k);

  logic [6:0] k_run;                // k + RRRR (7 bits so an overrun past 63 is visible)
  assign k_run = {1'b0, k} + {3'd0, hv_rdata[7:4]};

  always_ff @(posedge clk) begin
    done <= 1'b0; err_huff <= 1'b0; blk_we <= 1'b0;
    if (rst) begin
      state <= IDLE; phase_ac <= 1'b0; k <= '0; code <= '0; len <= '0; v <= '0; s <= '0; nb <= '0;
      comp_r <= '0; dc_tbl_r <= 1'b0; ac_tbl_r <= 1'b0; tq_r <= '0; blk_waddr <= '0; blk_wdata <= '0; coef_r <= '0;
      pred[0] <= '0; pred[1] <= '0; pred[2] <= '0; pred[3] <= '0;
    end else begin
      if (pred_clear) begin
        pred[0] <= '0; pred[1] <= '0; pred[2] <= '0; pred[3] <= '0;
      end
      case (state)
        IDLE: if (start) begin
          comp_r <= comp; dc_tbl_r <= dc_tbl; ac_tbl_r <= ac_tbl; tq_r <= tq;
          phase_ac <= 1'b0; k <= '0; code <= '0; len <= '0;
          state <= DEC;
        end
        DEC: if (take) begin
          code <= code_n; len <= len_n;
          if (match) begin
            state <= SYM;
          end else if (len_n == 5'd16) begin
            err_huff <= 1'b1; state <= FIN;            // no code of length <= 16 matched
          end
        end
        SYM: begin
          code <= '0; len <= '0; nb <= '0; v <= '0;
          s <= hv_rdata[3:0];
          if (!phase_ac) begin
            if (hv_rdata[3:0] == 4'd0) state <= EXT;     // DIFF = 0
            else state <= RECV;
          end else begin
            if (hv_rdata[3:0] == 4'd0) begin
              if (hv_rdata[7:4] == 4'hF) begin           // ZRL: sixteen zero coefficients
                if (k > 6'd47) begin err_huff <= 1'b1; state <= FIN; end
                else begin k <= k + 6'd16; state <= DEC; end
              end else state <= FIN;                     // EOB
            end else begin
              if (k_run > 7'd63) begin err_huff <= 1'b1; state <= FIN; end
              else begin k <= k_run[5:0]; state <= RECV; end
            end
          end
        end
        RECV: if (bit_valid) begin                       // RECEIVE (F.2.2.4): SSSS bits, MSB first
          v <= {v[14:0], bit_data};
          nb <= nb + 4'd1;
          if (nb + 4'd1 == s) state <= EXT;
        end
        EXT: begin                                       // EXTEND + DC prediction
          coef_r <= coef; state <= WRITE;
        end
        WRITE: begin
          blk_we <= 1'b1; blk_waddr <= zz_nat; blk_wdata <= prod_sat;
          if (!phase_ac) begin
            pred[comp_r] <= coef_r;
            phase_ac <= 1'b1; k <= 6'd1; state <= DEC;
          end else begin
            if (k == 6'd63) state <= FIN;
            else begin k <= k + 6'd1; state <= DEC; end
          end
        end
        FIN: begin
          done <= 1'b1; state <= IDLE;
        end
        default: state <= IDLE;
      endcase
    end
  end
endmodule
