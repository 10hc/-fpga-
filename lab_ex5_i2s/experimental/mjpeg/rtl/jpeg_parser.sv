// jpeg_parser.sv - streaming marker-segment parser for baseline JPEG (T.81 Annex B).
//
// Consumes the file one byte per clock (valid/ready), decodes DQT / DHT / SOF0 / DRI / SOS
// headers into table RAM writes and registers, skips every other segment by its length
// (APPn, COM, ... - including any JPEG thumbnail embedded inside an EXIF APP1, which a
// naive byte-pair search would mistake for the real image), and then forwards the
// entropy-coded segment as 9-bit tokens with the 0xFF00 byte stuffing removed
// (T.81 B.1.1.5 / F.1.2.3) and RSTn / EOI markers turned into marker tokens.
//
// The Huffman decoding tables are generated on the fly from BITS exactly as in
// T.81 Annex C (Generate_size_table / Generate_code_table) and F.2.2.3 (Figure F.15/F.16):
// for each code length L, MAXCODE(L), and VALPTR(L)-MINCODE(L) are stored.
//
// LUT_EN = 1 (fast decoder) also builds, per table, a 256-entry lookahead table indexed by the
// next 8 bits of the entropy-coded data: entry = {1, L-1, HUFFVAL} when those bits start with a
// code of length L <= 8, else 0 (longer code: the decoder falls back to MAXCODE).  Canonical
// codes (Annex C) of length <= 8 cover the index range [0, F) without gaps, code after code, so
// the table is written in index order while HUFFVAL arrives: 2^(8-L) entries per symbol, then
// [F, 256) is marked invalid.  The input is stalled meanwhile (at most ~256 clocks per table).
//
// Validation (CHECKS = 1, before the data path relies on a header or table; CHECKS = 0 keeps only
// the checks of the unsupported features, the Tq/Th range and the end-of-input handling, for the
// smallest devices): a table counts as defined only
// once it has been read completely and legally - DQT: Pq = 0, Tq <= 3, 64 non-zero Qk (B.2.4.1);
// DHT: Tc, Th <= 1, at most 240 AC / 16 DC symbols, a code tree that is neither over-subscribed
// nor uses an all-ones code (Annex C, the check of libjpeg's jpeg_make_d_derived_tbl), DC
// symbols <= 11 (F.1.2.1), AC symbols of size 1..10 or 0x00 / 0xF0 (F.1.2.2.1).  Segment lengths: below 2
// is an error for every marker (ERR_DQT / ERR_DHT / ERR_FRAME / ERR_SCAN by marker, else ERR_SYNC); SOF0
// and SOS need their fields (length 2: ERR_FRAME / ERR_SCAN), DRI is exactly 4 bytes (B.2.4.4, else
// ERR_SYNC); DQT, DHT, APPn and COM may be empty (length 2, as libjpeg).  The component identifiers
// of a frame are distinct (B.2.2, else ERR_FRAME).  Bytes before an SOI (between images, or before the
// first) belong to no image and are skipped without an error: err holds an image's errors from its
// frame_done until the next SOI clears it (before 2026-10-12 such bytes set ERR_SYNC on the image before,
// after its frame_done: found by tb/jpeg_checker.sv with data after the EOI).
// Colour space (cs_rgb, libjpeg 9e's rule): RGB images are passed through without colour conversion;
// libjpeg 9's big-gamut component IDs (BG_YCC 1,0x22,0x23; BG_RGB 'r','g','b') and its LSE colour
// transform marker (0xFFF8) are not supported (ERR_SOF_TYPE).  Tables persist across images (B.5); redefining one invalidates it
// until it is complete again.  SOF0: length 8 + 3 Nf, Tqi <= 3; SOS: length 6 + 2 Ns, Csj = Cj
// (the frame's components in frame order, B.2.3), and every table the scan uses defined.  Inside the scan RSTn must count 0..7
// modulo 8 (B.2.4.4, F.1.2.3).  (A zero X or Y - also: no SOF before the SOS - is flagged by the
// decoder, which checks the frame size anyway.)  Errors are reported on err_set; the decoder
// skips an image with header errors.
//
// End of input: in_last marks the last byte of a file.  If it arrives before the file's EOI,
// ERR_TRUNC is raised and the parser completes the image itself: inside the scan it appends an
// EOI token (the decoder decodes the missing data from zero bits and flags ERR_MARKER if bits
// were missing), in the headers it starts an (error) scan and appends the
// EOI token, so the decoder still ends the frame with frame_done.  Before an SOI (e.g. bytes
// after the EOI) in_last has no effect.
module jpeg_parser #(
  parameter bit LUT_EN = 1'b0,
  parameter bit CHECKS = 1'b1      // 0: no table/length/sequence validation (smaller; see the header)
) (
  input  logic        clk,
  input  logic        rst,
  // byte stream in
  input  logic        in_valid,
  input  logic [7:0]  in_data,
  input  logic        in_last,     // last byte of the file (0 if the caller does not know)
  output logic        in_ready,
  output logic        idle,        // waiting for SOI (nothing of a file consumed yet)
  // quantisation table writes: addr = {Tq[1:0], k[5:0]}, value k in zig-zag order (B.2.4.1)
  output logic        dqt_we,
  output logic [7:0]  dqt_waddr,
  output logic [7:0]  dqt_wdata,
  // Huffman code-table writes: addr = {Tc, Th, L-1} (L = 1..16)
  //   data = {valid, MAXCODE[15:0] + 1, (VALPTR - MINCODE) mod 256}
  output logic        hc_we,
  output logic [5:0]  hc_waddr,
  output logic [24:0] hc_wdata,
  // HUFFVAL writes, folded into 512 entries: AC table Th at {Th, idx[7:0]} (idx < 240),
  // DC table Th at {Th, 4'hF, idx[3:0]} (idx < 16).  Baseline tables hold at most 162 AC and
  // 12 DC symbols; larger tables are flagged ERR_DHT_ID.
  output logic        hv_we,
  output logic [8:0]  hv_waddr,
  output logic [7:0]  hv_wdata,
  // lookahead-table writes (LUT_EN): addr = {Tc, Th, next 8 bits}, data = {valid, L-1, HUFFVAL}
  output logic        lut_we,
  output logic [9:0]  lut_waddr,
  output logic [11:0] lut_wdata,
  // frame (SOF0) parameters (B.2.2)
  output logic [15:0] img_w,
  output logic [15:0] img_h,
  output logic [1:0]  nf,          // number of components Nf (1..3)
  output logic        cs_rgb,      // the 3 components are R, G, B, not Y, Cb, Cr (no colour conversion), by
                                   // libjpeg 9e's rule (jdapimin.c): component IDs 'R','G','B'; or IDs that are
                                   // none of 1,2,3 / 1,0x22,0x23 / 'r','g','b', no JFIF APP0, and an Adobe APP14
                                   // with transform 0.  Valid with scan_start.
  output logic [11:0] comp_h,      // Hi, 3 bits per component i (i = 0..3)
  output logic [11:0] comp_v,      // Vi
  output logic [7:0]  comp_tq,     // Tqi, 2 bits per component
  // scan (SOS) parameters (B.2.3), valid when scan_start pulses
  output logic [1:0]  ns,          // Ns
  output logic [7:0]  scan_ci,     // frame-component index of scan component j (2 bits each): always j,
                                   // the scan must name the frame's components in frame order (B.2.3)
  output logic [3:0]  scan_td,     // Tdj
  output logic [3:0]  scan_ta,     // Taj
  output logic [15:0] ri,          // restart interval (B.2.4.4), 0 = no restarts
  output logic        scan_start,  // one-cycle pulse: entropy-coded data follows
  // entropy-coded data tokens (see jpeg_pkg)
  output logic        tok_valid,
  output logic [8:0]  tok_data,
  input  logic        tok_ready,
  output logic        eoi,         // one-cycle pulse on EOI
  output logic        soi,         // one-cycle pulse: SOI accepted, a new image starts
  output logic [12:0] err_set      // one-cycle pulses, one per ERR_* code
);
  import jpeg_pkg::*;

  typedef enum logic [5:0] {
    S_SOI_FF, S_SOI_D8, S_MK_FF, S_MK, S_DISPATCH, S_LEN_HI, S_LEN_LO, S_SKIP,
    S_DQT_PQTQ, S_DQT_DATA,
    S_DHT_TCTH, S_DHT_BITS, S_DHT_VALS, S_DHT_INV,
    S_SOF_P, S_SOF_Y1, S_SOF_Y2, S_SOF_X1, S_SOF_X2, S_SOF_NF, S_SOF_CI, S_SOF_HV, S_SOF_TQ,
    S_DRI_HI, S_DRI_LO,
    S_SOS_NS, S_SOS_CS, S_SOS_TDTA, S_SOS_SS, S_SOS_SE, S_SOS_AHAL,
    S_ENT, S_ENT_FF,
    S_TR_SOS, S_TR_EOI           // in_last before EOI: start an error scan / append an EOI token
  } state_t;

  state_t      state;
  logic [7:0]  mk;          // current marker code
  logic [15:0] len;         // segment length field
  logic [15:0] rem;         // payload bytes still to consume in this segment, plus 2 (loaded with
                            // the length field Lx itself: no subtractor); the last byte has rem == 3
  logic [6:0]  k;           // generic small counter (DQT index, DHT length L, comp index ...)
  logic [1:0]  tq;          // current DQT table id
  logic [1:0]  tbl;         // current DHT table {Tc, Th}
  logic [16:0] code;        // Annex C running code
  logic [8:0]  cnt;         // Annex C running symbol count (VALPTR)
  logic [8:0]  ntot;        // number of HUFFVAL bytes in the current table
  logic [7:0]  ci_reg [0:2];// component identifiers C1..C3 (Nf > 3 is an error anyway)
  assign scan_ci = 8'b00_10_01_00;  // scan component j = frame component j
  logic [1:0]  nf_all;      // Nf-1 as parsed (for looping), even when unsupported
  logic [1:0]  sj;          // scan component counter
  // validation state
  logic [3:0]  q_def;       // DQT table Tq defined (complete and legal)
  logic [3:0]  h_def;       // DHT table {Tc, Th} defined
  logic        tbad;        // the table being read is invalid (zero Qk / illegal Huffman table)
  logic        sym_bad;     // this HUFFVAL byte is outside its table's domain: DC SSSS <= 11 (F.1.2.1);
                            // AC SSSS 1..10, or 0x00 (EOB) / 0xF0 (ZRL) (F.1.2.2.1)
  assign sym_bad = tbl[1] ? (in_data[3:0] > 4'd10 || (in_data[3:0] == 4'd0 && in_data != 8'h00 && in_data != 8'hF0))
                          : (in_data > 8'd11);
  logic        len2, nopay; // S_LEN_LO: the segment length is exactly 2; at most 2 (no payload)
  assign len2  = ({len[15:8], in_data} == 16'd2);
  assign nopay = (len[15:8] == 8'd0) && (in_data <= 8'd2);
  logic [2:0]  rst_exp;     // expected RSTn number
  logic        scanned;     // this image has had a scan (its frame is under way or done)
  // colour space: JFIF APP0 / Adobe APP14 seen in this image (libjpeg's examine_app0 / examine_app14),
  // read while their segments are skipped: identifier at payload bytes 0-4, Adobe transform at byte 11
  logic        saw_jfif, adobe0, am;   // JFIF APP0 seen; the last Adobe APP14 had transform 0; identifier matches
  logic [7:0]  app_id;      // identifier byte expected at payload byte k (k counts the skipped bytes, up
                            // to 15): "JFIF\0" (APP0), "Adobe" (APP14)
  always_comb case (k[2:0])
    3'd0:    app_id = (mk == 8'hE0) ? 8'h4A : 8'h41;
    3'd1:    app_id = (mk == 8'hE0) ? 8'h46 : 8'h64;
    3'd2:    app_id = (mk == 8'hE0) ? 8'h49 : 8'h6F;
    3'd3:    app_id = (mk == 8'hE0) ? 8'h46 : 8'h62;
    default: app_id = (mk == 8'hE0) ? 8'h00 : 8'h65;
  endcase
  // component ids: 1,2,3 / BG_YCC 1,0x22,0x23 and 'R','G','B' / BG_RGB 'r','g','b' differ in bit 5 only
  logic [6:0]  ci0_m, ci1_m, ci2_m;     // the ids without bit 5
  logic        ycc_ids, rgb_ids, id_123, id_bgycc, id_rgb, id_bgrgb;
  assign ci0_m    = {ci_reg[0][7:6], ci_reg[0][4:0]};
  assign ci1_m    = {ci_reg[1][7:6], ci_reg[1][4:0]};
  assign ci2_m    = {ci_reg[2][7:6], ci_reg[2][4:0]};
  assign ycc_ids  = (ci0_m == 7'h01) && !ci_reg[0][5] && (ci1_m == 7'h02) && (ci2_m == 7'h03) && (ci_reg[1][5] == ci_reg[2][5]);
  assign rgb_ids  = (ci0_m == 7'h32) && (ci1_m == 7'h27) && (ci2_m == 7'h22)
                    && (ci_reg[0][5] == ci_reg[1][5]) && (ci_reg[1][5] == ci_reg[2][5]);
  assign id_123   = ycc_ids && !ci_reg[1][5];
  assign id_bgycc = ycc_ids &&  ci_reg[1][5];
  assign id_rgb   = rgb_ids && !ci_reg[0][5];
  assign id_bgrgb = rgb_ids &&  ci_reg[0][5];
  assign cs_rgb   = (nf == 2'd3) && (id_rgb || (!ycc_ids && !rgb_ids && !saw_jfif && adobe0));
  // Annex C: code after the BITS(L) codes of length L.  libjpeg rejects a table when this reaches
  // 2^L at some length L (over-subscribed tree, or an all-ones code); the code value doubles
  // from each length to the next, so that happens exactly when it reaches 2^16 at some L <= 16.
  logic [17:0] csum;
  assign csum = {1'b0, code} + {10'd0, in_data};

  logic        consume;     // a byte is accepted this cycle
  logic        ent_state;

  // lookahead-table fill (LUT_EN): BITS(1..8) of the current table, the code length of the
  // HUFFVAL byte being placed, codes left at that length, entry counter inside its range,
  // next table index to write (9 bits: 256 = table full)
  logic [35:0] bitsv;       // BITS(1..8): BITS(L) in L bits at offset L(L-1)/2 (a legal tree has
                            // BITS(L) < 2^L; illegal ones are rejected with CHECKS=1)
  logic [3:0]  fl_len;      // 1..8, 9 = codes longer than 8 bits (no entries)
  logic [7:0]  fl_rem;
  logic [6:0]  fl_p;
  logic [6:0]  fl_lim;      // 2^(8-fl_len) - 1: last entry index of a code of length fl_len
  logic [8:0]  lp;
  logic        inv_to_mk;   // after S_DHT_INV: the DHT segment has ended
  logic        fl_short, fl_step;
  logic        fl_last;     // register: fl_p == fl_lim (keeps the compare out of in_ready)
  function automatic logic [7:0] bits_of(input logic [35:0] b, input logic [3:0] l);   // BITS(l), l = 1..8
    case (l)
      4'd1:    bits_of = {7'd0, b[0]};
      4'd2:    bits_of = {6'd0, b[2:1]};
      4'd3:    bits_of = {5'd0, b[5:3]};
      4'd4:    bits_of = {4'd0, b[9:6]};
      4'd5:    bits_of = {3'd0, b[14:10]};
      4'd6:    bits_of = {2'd0, b[20:15]};
      4'd7:    bits_of = {1'd0, b[27:21]};
      default: bits_of = b[35:28];
    endcase
  endfunction
  assign fl_short = LUT_EN && (fl_len <= 4'd8);
  assign fl_step  = fl_short && (fl_rem == 8'd0);                   // move on to the next code length

  assign ent_state = (state == S_ENT) || (state == S_ENT_FF);
  assign idle      = (state == S_SOI_FF);
  always_comb begin
    if (ent_state)                                   in_ready = tok_ready;
    else if (state == S_DISPATCH || state == S_DHT_INV || state == S_TR_SOS || state == S_TR_EOI) in_ready = 1'b0;
    else if (state == S_DHT_VALS && fl_short)        in_ready = !fl_step && fl_last;
    else                                             in_ready = 1'b1;
  end
  assign consume   = in_valid & in_ready;

  // ---------------------------------------------------------------- token output
  always_comb begin
    tok_valid = 1'b0;
    tok_data  = {1'b0, in_data};
    if (state == S_TR_EOI) begin
      // appended end marker (truncated file): any marker code except RSTn (11010xxx) ends the
      // scan, so only bits 8 and 3 are forced (keeps the input-to-token path short)
      tok_valid = 1'b1; tok_data = {1'b1, in_data[7:4], 1'b1, in_data[2:0]};
    end else if (in_valid) begin
      if (state == S_ENT && in_data != 8'hFF)
        tok_valid = 1'b1;                                   // plain data byte
      else if (state == S_ENT_FF) begin
        if (in_data == 8'h00) begin
          tok_valid = 1'b1; tok_data = {1'b0, 8'hFF};       // stuffed FF00 -> FF
        end else if (in_data != 8'hFF) begin
          tok_valid = 1'b1; tok_data = {1'b1, in_data};     // RSTn / EOI / other marker
        end
      end
    end
  end

  // ---------------------------------------------------------------- table writes (registered)
  always_ff @(posedge clk) begin
    dqt_we <= 1'b0; hc_we <= 1'b0; hv_we <= 1'b0; lut_we <= 1'b0;
    if (LUT_EN && state == S_DHT_VALS && fl_short && !fl_step && in_valid) begin
      lut_we <= 1'b1; lut_waddr <= {tbl, lp[7:0]}; lut_wdata <= {1'b1, fl_len[2:0] - 3'd1, in_data};
    end
    if (LUT_EN && state == S_DHT_INV && !lp[8]) begin
      lut_we <= 1'b1; lut_waddr <= {tbl, lp[7:0]}; lut_wdata <= 12'd0;
    end
    if (consume && state == S_DQT_DATA) begin
      dqt_we <= 1'b1; dqt_waddr <= {tq, k[5:0]}; dqt_wdata <= in_data;
    end
    if (consume && state == S_DHT_BITS) begin
      // T.81 F.2.2.3 Figure F.15: MAXCODE(L) = code + BITS(L) - 1 (or -1 when BITS(L)=0),
      // VALPTR(L) = running symbol count, MINCODE(L) = code.  We store MAXCODE(L)+1 = code +
      // BITS(L) (the decoders test code < MAXCODE+1; < 2^16 in a legal table) and VALPTR-MINCODE
      // mod 256.
      hc_we    <= 1'b1;
      hc_waddr <= {tbl, k[3:0]};
      hc_wdata <= {(in_data != 8'h00), csum[15:0], cnt[7:0] - code[7:0]};
    end
    if (consume && state == S_DHT_VALS) begin
      hv_we <= 1'b1; hv_waddr <= {tbl[0], tbl[1] ? cnt[7:0] : {4'hF, cnt[3:0]}}; hv_wdata <= in_data;
    end
  end

  // ---------------------------------------------------------------- main FSM
  always_ff @(posedge clk) begin
    scan_start <= 1'b0;
    eoi        <= 1'b0;
    soi        <= 1'b0;
    err_set    <= '0;
    if (rst) begin
      // control state only: the data registers are always written before they are used
      // (fewer reset loads also let the fitter pack the logic more densely)
      state <= S_SOI_FF; ri <= '0; img_w <= '0; img_h <= '0; fl_len <= 4'd9;
      q_def <= '0; h_def <= '0; scanned <= 1'b0;
    end else begin
      case (state)
        // ---- SOI
        S_SOI_FF: if (consume) begin
          if (in_data == 8'hFF) state <= S_SOI_D8;                // other bytes: no image yet, skipped
        end
        S_SOI_D8: if (consume) begin
          // SOI: image-local state starts afresh (B.2.4.4: no restart interval until a DRI;
          // no frame until an SOF).  Tables persist, as abbreviated streams allow (B.5).
          if (in_data == 8'hD8) begin
            state <= S_MK_FF; soi <= 1'b1; ri <= '0; img_w <= '0; img_h <= '0; scanned <= 1'b0;
            saw_jfif <= 1'b0; adobe0 <= 1'b0;
          end
          else if (in_data != 8'hFF) state <= S_SOI_FF;
        end
        // ---- marker
        S_MK_FF: if (consume) begin
          if (in_data == 8'hFF) state <= S_MK;
          else err_set[ERR_SYNC] <= 1'b1;
        end
        S_MK: if (consume) begin
          if (in_data != 8'hFF) begin mk <= in_data; state <= S_DISPATCH; end   // FF fill bytes allowed
        end
        S_DISPATCH: begin
          if (mk == 8'hD8) state <= S_MK_FF;                            // SOI again: ignore
          else if (mk == 8'hD9) begin eoi <= 1'b1; state <= S_SOI_FF; end
          else if (mk == 8'h01 || (mk[7:3] == 5'b11010)) state <= S_MK_FF; // TEM, RSTn: no length
          else state <= S_LEN_HI;
        end
        S_LEN_HI: if (consume) begin len[15:8] <= in_data; state <= S_LEN_LO; end
        S_LEN_LO: if (consume) begin
          len[7:0] <= in_data;
          rem <= {len[15:8], in_data};
          k <= '0; sj <= '0; am <= 1'b1;
          // no payload (length 2 or, impossible, less): see the header (lengths) for the errors
          case (mk)
            8'hDB: if (nopay) begin state <= S_MK_FF; if (CHECKS && !len2) err_set[ERR_DQT] <= 1'b1; end
                   else state <= S_DQT_PQTQ;
            8'hC4: if (nopay) begin state <= S_MK_FF; if (CHECKS && !len2) err_set[ERR_DHT] <= 1'b1; end
                   else state <= S_DHT_TCTH;
            8'hC0: if (nopay) begin state <= S_MK_FF; if (CHECKS) err_set[ERR_FRAME] <= 1'b1; end
                   else state <= S_SOF_P;
            8'hC1, 8'hC2, 8'hC3, 8'hC5, 8'hC6, 8'hC7, 8'hC9, 8'hCA, 8'hCB, 8'hCD, 8'hCE, 8'hCF, 8'hF8:
                   begin state <= nopay ? S_MK_FF : S_SKIP; err_set[ERR_SOF_TYPE] <= 1'b1; end   // F8: LSE (libjpeg 9)
            8'hDD: if (CHECKS && {len[15:8], in_data} != 16'd4) begin                   // Lr = 4 (B.2.4.4)
                     err_set[ERR_SYNC] <= 1'b1; state <= nopay ? S_MK_FF : S_SKIP;
                   end else state <= nopay ? S_MK_FF : S_DRI_HI;
            8'hDA: if (nopay) begin state <= S_MK_FF; if (CHECKS) err_set[ERR_SCAN] <= 1'b1; end
                   else state <= S_SOS_NS;
            default: if (nopay) begin state <= S_MK_FF; if (CHECKS && !len2) err_set[ERR_SYNC] <= 1'b1; end
                     else state <= S_SKIP;
          endcase
        end
        S_SKIP: if (consume) begin
          rem <= rem - 16'd1;
          if (rem == 16'd3) state <= S_MK_FF;
          if (k != 7'd15) k <= k + 7'd1;
          if (k <= 7'd4 && in_data != app_id) am <= 1'b0;
          if (mk == 8'hE0 && k == 7'd13 && am) saw_jfif <= 1'b1;             // JFIF: 14 data bytes at least
          if (mk == 8'hEE && k == 7'd11 && am) adobe0 <= (in_data == 8'd0);   // the last Adobe APP14 counts
        end
        // ---- DQT (B.2.4.1): PqTq, 64 x Qk (zig-zag order); several tables may follow
        S_DQT_PQTQ: if (consume) begin
          rem <= rem - 16'd1; tq <= in_data[1:0]; k <= '0; tbad <= 1'b0;
          if (in_data[7:4] != 4'd0 || in_data[3:2] != 2'd0 || rem == 16'd3) begin   // 16-bit, Tq > 3, or no data
            err_set[ERR_DQT] <= 1'b1; state <= (rem == 16'd3) ? S_MK_FF : S_SKIP;
          end else begin
            q_def[in_data[1:0]] <= 1'b0; state <= S_DQT_DATA;        // being redefined
          end
        end
        S_DQT_DATA: if (consume) begin
          rem <= rem - 16'd1; k <= k + 7'd1;
          if (CHECKS && in_data == 8'd0) tbad <= 1'b1;
          if (k == 7'd63) begin                                     // table complete
            q_def[tq] <= !(tbad || in_data == 8'd0);
            if (CHECKS && (tbad || in_data == 8'd0)) err_set[ERR_DQT] <= 1'b1;  // Qk = 0
            state <= (rem == 16'd3) ? S_MK_FF : S_DQT_PQTQ;
          end else if (rem == 16'd3) begin
            err_set[ERR_DQT] <= CHECKS; state <= S_MK_FF;           // segment ends inside the table
          end
        end
        // ---- DHT (B.2.4.2): TcTh, BITS(1..16), HUFFVAL; several tables may follow
        S_DHT_TCTH: if (consume) begin
          rem <= rem - 16'd1; tbl <= {in_data[4], in_data[0]}; k <= 7'd0; code <= '0; cnt <= '0;  // k = L-1
          lp <= '0; tbad <= 1'b0;
          if (in_data[7:5] != 3'd0 || in_data[3:1] != 3'd0 || rem == 16'd3) begin   // Tc/Th > 1, or no data
            err_set[ERR_DHT] <= 1'b1; state <= (rem == 16'd3) ? S_MK_FF : S_SKIP;
          end else begin
            h_def[{in_data[4], in_data[0]}] <= 1'b0; state <= S_DHT_BITS;          // being redefined
          end
        end
        S_DHT_BITS: if (consume) begin
          rem <= rem - 16'd1; k <= k + 7'd1;
          code <= {csum[15:0], 1'b0};                          // Annex C: code = (code + BITS(L)) << 1
          cnt  <= cnt + {1'b0, in_data};
          if (CHECKS && csum[17:16] != 2'd0) begin tbad <= 1'b1; err_set[ERR_DHT] <= 1'b1; end   // see csum
          if (k < 7'd8)
            case (k[2:0])
              3'd0: bitsv[0]     <= in_data[0];
              3'd1: bitsv[2:1]   <= in_data[1:0];
              3'd2: bitsv[5:3]   <= in_data[2:0];
              3'd3: bitsv[9:6]   <= in_data[3:0];
              3'd4: bitsv[14:10] <= in_data[4:0];
              3'd5: bitsv[20:15] <= in_data[5:0];
              3'd6: bitsv[27:21] <= in_data[6:0];
              default: bitsv[35:28] <= in_data;
            endcase
          if (k == 7'd15) begin
            ntot <= cnt + {1'b0, in_data}; cnt <= '0;
            fl_len <= 4'd1; fl_rem <= bits_of(bitsv, 4'd1); fl_p <= '0; fl_lim <= 7'd127; fl_last <= 1'b0;
            if (tbl[1] ? (cnt + {1'b0, in_data} > 9'd240) : (cnt + {1'b0, in_data} > 9'd16)) begin
              tbad <= 1'b1; err_set[ERR_DHT] <= 1'b1;               // table too large for the folded RAM
            end
            if (cnt + {1'b0, in_data} != 9'd0) begin
              if (rem == 16'd3) begin err_set[ERR_DHT] <= CHECKS; state <= S_MK_FF; end   // no HUFFVAL
              else state <= S_DHT_VALS;
            end else begin inv_to_mk <= (rem == 16'd3); state <= LUT_EN ? S_DHT_INV : ((rem == 16'd3) ? S_MK_FF : S_DHT_TCTH); end
          end else if (rem == 16'd3) begin
            err_set[ERR_DHT] <= CHECKS; state <= S_MK_FF;           // segment ends inside BITS
          end
        end
        S_DHT_VALS: begin
          if (fl_step) begin                                   // no (more) codes of this length
            fl_len <= fl_len + 4'd1; fl_lim <= fl_lim >> 1;
            fl_last <= (fl_lim[6:1] == 6'd0);                  // (fl_p is 0 here) new limit 0: one entry per code
            fl_rem <= (fl_len == 4'd8) ? 8'd0 : bits_of(bitsv, fl_len + 4'd1);   // BITS(L+1)
          end else if (fl_short && in_valid) begin             // one table entry per clock
            lp <= lp + 9'd1;
            fl_p <= fl_last ? 7'd0 : fl_p + 7'd1;
            fl_last <= fl_last ? (fl_lim == 7'd0) : (fl_p + 7'd1 == fl_lim);
            if (fl_last) fl_rem <= fl_rem - 8'd1;
          end
          if (consume) begin
            rem <= rem - 16'd1; cnt <= cnt + 9'd1;
            if (CHECKS && sym_bad) begin tbad <= 1'b1; err_set[ERR_DHT] <= 1'b1; end
            if (cnt + 9'd1 == ntot)                                 // table complete
              h_def[tbl] <= !(tbad || (CHECKS ? sym_bad : (!tbl[1] && in_data > 8'd11)));
            else if (CHECKS && rem == 16'd3) err_set[ERR_DHT] <= 1'b1;   // segment ends inside HUFFVAL
            if (rem == 16'd3 || cnt + 9'd1 == ntot) begin
              inv_to_mk <= (rem == 16'd3);
              if (LUT_EN) state <= S_DHT_INV;
              else        state <= (rem == 16'd3) ? S_MK_FF : S_DHT_TCTH;
            end
          end
        end
        S_DHT_INV: begin                                       // invalidate [F, 256)
          lp <= lp + 9'd1;
          if (lp[8] || lp[7:0] == 8'hFF) state <= inv_to_mk ? S_MK_FF : S_DHT_TCTH;
        end
        // ---- SOF0 (B.2.2): P, Y, X, Nf, then Ci, HiVi, Tqi per component
        S_SOF_P: if (consume) begin
          rem <= rem - 16'd1; state <= (rem == 16'd3) ? S_MK_FF : S_SOF_Y1;
          if (in_data != 8'd8) err_set[ERR_PRECISION] <= 1'b1;
          if (CHECKS && rem == 16'd3) err_set[ERR_FRAME] <= 1'b1;                 // Lf too small
        end
        S_SOF_Y1: if (consume) begin rem <= rem - 16'd1; img_h[15:8] <= in_data; state <= (rem == 16'd3) ? S_MK_FF : S_SOF_Y2; if (CHECKS && rem == 16'd3) err_set[ERR_FRAME] <= 1'b1; end
        S_SOF_Y2: if (consume) begin rem <= rem - 16'd1; img_h[7:0]  <= in_data; state <= (rem == 16'd3) ? S_MK_FF : S_SOF_X1; if (CHECKS && rem == 16'd3) err_set[ERR_FRAME] <= 1'b1; end
        S_SOF_X1: if (consume) begin rem <= rem - 16'd1; img_w[15:8] <= in_data; state <= (rem == 16'd3) ? S_MK_FF : S_SOF_X2; if (CHECKS && rem == 16'd3) err_set[ERR_FRAME] <= 1'b1; end
        S_SOF_X2: if (consume) begin rem <= rem - 16'd1; img_w[7:0]  <= in_data; state <= (rem == 16'd3) ? S_MK_FF : S_SOF_NF; if (CHECKS && rem == 16'd3) err_set[ERR_FRAME] <= 1'b1; end
        S_SOF_NF: if (consume) begin
          rem <= rem - 16'd1; k <= '0;
          nf     <= (in_data == 8'd1) ? 2'd1 : 2'd3;
          nf_all <= (in_data >= 8'd4) ? 2'd3 : in_data[1:0] - 2'd1;
          if (in_data != 8'd1 && in_data != 8'd3) err_set[ERR_NCOMP] <= 1'b1;
          if (CHECKS && rem == 16'd3) err_set[ERR_FRAME] <= 1'b1;
          state <= (rem == 16'd3 || in_data == 8'd0) ? S_MK_FF : S_SOF_CI;
        end
        S_SOF_CI: if (consume) begin
          rem <= rem - 16'd1; if (k[1:0] != 2'd3) ci_reg[k[1:0]] <= in_data; state <= (rem == 16'd3) ? S_MK_FF : S_SOF_HV;
          if (CHECKS && rem == 16'd3) err_set[ERR_FRAME] <= 1'b1;
          if (CHECKS && ((k[1:0] != 2'd0 && in_data == ci_reg[0]) || (k[1:0] == 2'd2 && in_data == ci_reg[1])))
            err_set[ERR_FRAME] <= 1'b1;                               // Ci of a frame are distinct (B.2.2)
        end
        S_SOF_HV: if (consume) begin
          rem <= rem - 16'd1;
          // only "factor 2" matters to the decoder (1 and 2 are supported, anything else is ERR_SAMPLING)
          comp_h[3*k[1:0] +: 3] <= {1'b0, in_data[7:4] == 4'd2, 1'b0};
          comp_v[3*k[1:0] +: 3] <= {1'b0, in_data[3:0] == 4'd2, 1'b0};
          if (!(in_data[7:4] == 4'd1 || in_data[7:4] == 4'd2) || !(in_data[3:0] == 4'd1 || in_data[3:0] == 4'd2))
            err_set[ERR_SAMPLING] <= 1'b1;
          state <= (rem == 16'd3) ? S_MK_FF : S_SOF_TQ;
          if (CHECKS && rem == 16'd3) err_set[ERR_FRAME] <= 1'b1;
        end
        S_SOF_TQ: if (consume) begin
          rem <= rem - 16'd1; comp_tq[2*k[1:0] +: 2] <= in_data[1:0]; k <= k + 7'd1;
          if (CHECKS && in_data[7:2] != 6'd0) err_set[ERR_FRAME] <= 1'b1;
          if (CHECKS && ((rem == 16'd3) != (k[1:0] == nf_all))) err_set[ERR_FRAME] <= 1'b1;   // Lf = 8 + 3 Nf exactly
          if (rem == 16'd3 || k[1:0] == nf_all) state <= S_MK_FF; else state <= S_SOF_CI;
        end
        // ---- DRI (B.2.4.4)
        S_DRI_HI: if (consume) begin rem <= rem - 16'd1; ri[15:8] <= in_data; state <= (rem == 16'd3) ? S_MK_FF : S_DRI_LO; end
        S_DRI_LO: if (consume) begin rem <= rem - 16'd1; ri[7:0]  <= in_data; state <= S_MK_FF; end
        // ---- SOS (B.2.3): Ns, then Csj, TdjTaj per scan component, then Ss, Se, AhAl
        S_SOS_NS: if (consume) begin
          rem <= rem - 16'd1; sj <= '0;
          ns <= (in_data == 8'd1) ? 2'd1 : 2'd3;
          if (in_data != {6'd0, nf}) err_set[ERR_SCAN] <= 1'b1;    // only one interleaved scan of all components
          if (CHECKS && rem == 16'd3) err_set[ERR_SCAN] <= 1'b1;               // Ls too small (Ls = 6 + 2 Ns)
          state <= (rem == 16'd3) ? S_MK_FF : S_SOS_CS;
        end
        S_SOS_CS: if (consume) begin
          rem <= rem - 16'd1;
          // B.2.3: "the ordering in the scan header shall follow the ordering in the frame header";
          // with the one interleaved scan of all components supported here, Csj is the frame's Cj
          if (in_data != ((sj == 2'd2) ? ci_reg[2] : sj[0] ? ci_reg[1] : ci_reg[0])) err_set[ERR_SCAN] <= 1'b1;
          if (CHECKS && rem == 16'd3) err_set[ERR_SCAN] <= 1'b1;
          state <= (rem == 16'd3) ? S_MK_FF : S_SOS_TDTA;
        end
        S_SOS_TDTA: if (consume) begin
          rem <= rem - 16'd1; scan_td[sj] <= in_data[4]; scan_ta[sj] <= in_data[0]; sj <= sj + 2'd1;
          if (in_data[7:5] != 3'd0 || in_data[3:1] != 3'd0) err_set[ERR_DHT_ID] <= 1'b1;
          // the tables this component uses must be defined (Huffman DC Td, AC Ta; its Tqi)
          if (CHECKS && (!h_def[{1'b0, in_data[4]}] || !h_def[{1'b1, in_data[0]}])) err_set[ERR_DHT] <= 1'b1;
          if (CHECKS && !q_def[comp_tq[2*sj +: 2]]) err_set[ERR_DQT] <= 1'b1;
          if (rem == 16'd3) begin state <= S_MK_FF; err_set[ERR_SCAN] <= CHECKS; end
          else if (sj == ns - 2'd1) state <= S_SOS_SS; else state <= S_SOS_CS;
        end
        S_SOS_SS: if (consume) begin rem <= rem - 16'd1; if (in_data != 8'd0  || (CHECKS && rem == 16'd3)) err_set[ERR_SCAN] <= 1'b1; state <= (rem == 16'd3) ? S_MK_FF : S_SOS_SE; end
        S_SOS_SE: if (consume) begin rem <= rem - 16'd1; if (in_data != 8'd63 || (CHECKS && rem == 16'd3)) err_set[ERR_SCAN] <= 1'b1; state <= (rem == 16'd3) ? S_MK_FF : S_SOS_AHAL; end
        S_SOS_AHAL: if (consume) begin
          rem <= rem - 16'd1; if (in_data != 8'd0 || (CHECKS && rem != 16'd3)) err_set[ERR_SCAN] <= 1'b1;   // Ls exact
          scan_start <= 1'b1; state <= S_ENT; rst_exp <= '0; scanned <= 1'b1;   // (a wrong Ls was flagged at Ns)
          if (nf == 2'd3 && (id_bgycc || id_bgrgb)) err_set[ERR_SOF_TYPE] <= 1'b1;   // libjpeg 9 big gamut
        end
        // ---- entropy-coded segment (F.1.2.3 byte stuffing, B.1.1.2 fill bytes)
        S_ENT: if (consume) begin
          if (in_data == 8'hFF) state <= S_ENT_FF;
        end
        S_ENT_FF: if (consume) begin
          if (in_data == 8'h00) state <= S_ENT;                  // stuffed zero: FF was data
          else if (in_data == 8'hFF) state <= S_ENT_FF;          // fill byte
          else if (in_data[7:3] == 5'b11010) begin               // RSTn: token emitted, scan continues
            state <= S_ENT; rst_exp <= rst_exp + 3'd1;
            if (CHECKS && in_data[2:0] != rst_exp) err_set[ERR_MARKER] <= 1'b1;   // (without DRI: see the decoder's err_pad)
          end
          else begin mk <= in_data; state <= S_DISPATCH; end     // EOI (or another marker): scan ends
        end
        // ---- truncated file (in_last before EOI)
        S_TR_SOS: begin scan_start <= 1'b1; state <= S_TR_EOI; end   // an error scan: the decoder skips it
        S_TR_EOI: if (tok_ready) begin mk <= 8'hD9; state <= S_DISPATCH; end
        default: state <= S_SOI_FF;
      endcase
      // in_last: the file ends with this byte.  Complete an unfinished image (see the header):
      // inside the entropy-coded data the decoder still waits for the marker that ends the scan
      // (append an EOI token); after a scan (the frame has its end marker already) only the eoi
      // pulse is missing; in headers before any scan an error scan lets the decoder end a frame.
      if (consume && in_last) begin
        if (state == S_ENT || (state == S_ENT_FF && (in_data == 8'h00 || in_data == 8'hFF || in_data[7:3] == 5'b11010))) begin
          err_set[ERR_TRUNC] <= 1'b1; state <= S_TR_EOI;
        end else if (state != S_SOI_FF && state != S_SOI_D8 && !((state == S_MK || state == S_ENT_FF) && in_data == 8'hD9)) begin
          err_set[ERR_TRUNC] <= 1'b1;
          if (scanned || state == S_ENT_FF) begin mk <= 8'hD9; state <= S_DISPATCH; end
          else state <= S_TR_SOS;
        end
      end
    end
  end
endmodule
