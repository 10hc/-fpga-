// jpeg_raster.sv - raster-order output stage (jpeg_decoder with RASTER_OUT = 1).
//
// The decoder writes every IDCT block of an MCU row straight into a row buffer holding the
// component planes of one MCU row (planar, row pitch pw_c = MCUs_per_row * 8 * Hc samples).
// When the MCU row is complete this module reads it back line by line in image order,
// upsamples the chroma - sample replication, or libjpeg-turbo's triangle filter when FANCY=1
// (jdsample.c h2v1/h1v2/h2v2_fancy_upsample, the default of Pillow and OpenCV) - and formats
// each pixel as RGB, YCbCr or Y.
//
// The vertical filter needs the component row above the first line of an MCU row (it belongs
// to the previous MCU row) and the row below its last line (next MCU row).  The decoder handles
// that with one row buffer plus a line buffer (in the same RAM, after the planes):
//   1. CMD_LINES  lines 0..mh-2 of MCU row n        (line 0 takes its "row above" from the
//                                                    line buffer = last rows of MCU row n-1)
//   2. CMD_COPY   last plane row of each component -> line buffer
//   3. (decoder decodes MCU row n+1 into the planes)
//   4. CMD_DEFER  line mh-1 of MCU row n: near rows from the line buffer, row below = row 0 of
//                 the planes (MCU row n+1)
//   then CMD_LINES for row n+1, and so on.  The last MCU row is emitted completely, with the
//   bottom edge replicated at the true component height like jdmainct.c does.
//
// Per output pixel each component needs at most one new column (1 or 2 RAM reads); per
// component a window of column sums cs = 3*near + far (vertical filter) or 4*near is kept and
// every filter reduces to     out = (3*cs[i] + cs[neighbour] + bias) >> 4
// (edges replicated at the true component width; see model/jpeg_golden.py upsample_plane).
// Throughput: (reads + 2) clocks per pixel, e.g. 4:2:0 with the filter ~5, luma only 3.
// Based in part on the work of the Independent JPEG Group: the arithmetic reproduces libjpeg /
// libjpeg-turbo so that the output is bit-identical to them (see NOTICE.md).
module jpeg_raster #(
  parameter int AW       = 14,        // row-buffer address width
  parameter bit FANCY    = 1'b0,
  parameter bit CC_TURBO = 1'b0,
  parameter bit RGB_OUT  = 1'b1       // 0: no colour converter, FMT_RGB behaves like FMT_YCBCR
) (
  input  logic            clk,
  input  logic            rst,
  // command from the decoder
  input  logic            start,
  input  logic [1:0]      mode,       // CMD_LINES / CMD_DEFER / CMD_COPY
  input  logic [4:0]      nlines,     // CMD_LINES: local lines 0 .. nlines-1
  input  logic [15:0]     y0,         // CMD_LINES: image row of the first line (CMD_DEFER: last row + 1)
  input  logic [12:0]     my,         // MCU row index of the planes' contents
  input  logic            lastrow,    // CMD_LINES: this is the last MCU row of the image
  output logic            done,       // pulse
  output logic            idle,       // no command running and the output pipeline empty
  // frame geometry (stable during a frame); component c at [c*AW +: AW]
  input  logic [15:0]     img_w,
  input  logic [15:0]     img_h,
  input  logic [1:0]      nproc,      // components read: 1 (grey image or FMT_Y) or 3
  input  logic            gray,
  input  logic [1:0]      fmt,
  input  logic [2:0]      uh,         // component c is subsampled horizontally (Hc < Hmax)
  input  logic [2:0]      uv,         // ... vertically (Vc < Vmax)
  input  logic [2:0]      hf,         // horizontal triangle filter active for component c
  input  logic [2:0]      vf,         // vertical triangle filter active for component c
  input  logic [3*AW-1:0] pw,         // plane row pitch
  input  logic [3*AW-1:0] pend,       // end of plane c (= base of plane c+1); plane 0 starts at 0
  input  logic [3*AW-1:0] lb,         // line-buffer row of component c
  // row buffer
  output logic [AW-1:0]   raddr,
  input  logic [7:0]      rdata,      // registered read, 1-cycle latency
  output logic            cp_we,
  output logic [AW-1:0]   cp_waddr,
  output logic [7:0]      cp_wdata,
  // pixels
  output logic            px_valid,
  input  logic            px_ready,
  output logic [15:0]     px_x,
  output logic [15:0]     px_y,
  output logic [7:0]      px_c0,
  output logic [7:0]      px_c1,
  output logic [7:0]      px_c2,
  output logic            px_sof,
  output logic            px_eol
);
  import jpeg_pkg::*;

  typedef enum logic [2:0] { R_IDLE, R_LINE, R_FETCH, R_HAND, R_COPY, R_COPY_END } rstate_t;
  rstate_t state;

  logic [1:0]    mode_r;
  logic [12:0]   my_r;
  logic [4:0]    lines_left;
  logic [3:0]    l;                    // local line index inside the MCU row
  logic [15:0]   x, y;
  logic [15:0]   wm1;                  // image width - 1
  logic          vline;                // vertical phase of the line (1 = lower output row of a pair)
  logic          lastrow_r;
  logic [2:0]    lastcr;               // last row of a vertically subsampled component inside the
                                       // last MCU row: ((H-1)/2) mod 8 = bits 3..1 of H-1
  logic [AW-1:0] nrow [0:2];           // near-row base of component c for the current line
  logic [1:0]    fmode [0:2];          // far row: 0 = near (edge), 1 = near-pw, 2 = near+pw, 3 = alt
  logic [9:0]    csl [0:2], csc [0:2], csr [0:2];   // column-sum window per component
  logic [7:0]    ntmp [0:2];           // near sample waiting for its far sample
  logic [14:0]   colh;                 // column of the horizontally filtered components

  logic [5:0]    mask;                 // reads still to issue: bit 2c = near row, 2c+1 = far row of c
  logic          infl;                 // a read was issued last cycle ...
  logic [2:0]    infl_item;            // ... for item {c, far}

  // copy engine: last plane row of each component -> contiguous line buffer
  logic [1:0]    cc;
  logic [AW:0]   ccnt;
  logic [AW-1:0] csrc, cdst;
  logic          wpend;
  logic [AW-1:0] wpend_addr;

  // output pipeline: V (value) stage -> O (pixel register) stage
  logic          v_valid, v_conv, v_sof, v_eol;
  logic [15:0]   v_x, v_y;
  logic [7:0]    v_a, v_b, v_c;
  logic          o_take, v_free;
  assign o_take = v_valid & (~px_valid | px_ready);
  assign v_free = ~v_valid | o_take;

  // ---------------------------------------------------------------- per-component geometry
  function automatic logic [AW-1:0] gA(input logic [3*AW-1:0] v, input integer c);
    gA = v[c*AW +: AW];
  endfunction
  logic [3*AW-1:0] base_pk;            // plane c starts where plane c-1 ends; plane 0 at 0
  assign base_pk = {pend[2*AW-1:0], {AW{1'b0}}};

  // ---------------------------------------------------------------- next-pixel setup
  // xs = pixel about to be processed: 0 at a line start (R_LINE), x+1 when advancing (R_HAND).
  // Every horizontally subsampled component has the same true width ceil(W/2), so one clamp
  // serves them all: right neighbour column = min(xs/2 + 1, ceil(W/2) - 1) = min(.., (W-1)/2).
  logic [15:0]   xs, cxp1;
  logic [14:0]   colh_n;
  logic [5:0]    mask_n;
  logic [2:0]    fetch_n, shift_n;
  always_comb begin : g_next
    integer k;
    xs     = (state == R_HAND) ? x + 16'd1 : 16'd0;
    cxp1   = {1'b0, xs[15:1]} + 16'd1;
    colh_n = (xs == 16'd0) ? 15'd0 : ((cxp1 > {1'b0, wm1[15:1]}) ? wm1[15:1] : cxp1[14:0]);
    for (k = 0; k < 3; k = k + 1) begin
      if (hf[k])      fetch_n[k] = (xs == 16'd0) | xs[0];      // new right column on odd pixels
      else if (uh[k]) fetch_n[k] = ~xs[0];                     // replication: new column on even pixels
      else            fetch_n[k] = 1'b1;
      if (k >= nproc) fetch_n[k] = 1'b0;
      shift_n[k] = hf[k] & (xs != 16'd0) & ~xs[0];             // even pixel: slide the window
    end
    mask_n = {fetch_n[2] & vf[2], fetch_n[2], fetch_n[1] & vf[1], fetch_n[1], fetch_n[0] & vf[0], fetch_n[0]};
  end

  // ---------------------------------------------------------------- read issue
  logic [2:0]    item;                 // lowest pending read
  always_comb begin
    if      (mask[0]) item = 3'd0;
    else if (mask[1]) item = 3'd1;
    else if (mask[2]) item = 3'd2;
    else if (mask[3]) item = 3'd3;
    else if (mask[4]) item = 3'd4;
    else              item = 3'd5;
  end
  logic [1:0]    fi;                   // component of the read being issued
  logic [AW-1:0] f_nrow, f_pw, f_alt, f_row, f_col;
  logic [31:0]   f_colt;
  assign fi    = item[2:1];
  assign f_nrow = nrow[fi];
  assign f_pw   = gA(pw, fi);
  assign f_alt  = (mode_r == CMD_DEFER) ? gA(base_pk, fi) : gA(lb, fi);   // row 0 of the next MCU row / line buffer
  always_comb begin
    if (!item[0])               f_row = f_nrow;
    else case (fmode[fi])
      2'd1:    f_row = f_nrow - f_pw;
      2'd2:    f_row = f_nrow + f_pw;
      2'd3:    f_row = f_alt;
      default: f_row = f_nrow;
    endcase
    if (uh[fi]) f_colt = hf[fi] ? {17'd0, colh} : {17'd0, x[15:1]};
    else        f_colt = {16'd0, x};
    f_col = f_colt[AW-1:0];
  end
  assign raddr = (state == R_COPY) ? csrc : (f_row + f_col);

  // ---------------------------------------------------------------- captured read -> column sum
  logic [1:0]    ic;
  logic [9:0]    cs_new;
  logic          cs_store;
  assign ic       = infl_item[2:1];
  assign cs_new   = infl_item[0] ? ({ntmp[ic], 1'b0} + {2'b00, ntmp[ic]} + {2'b00, rdata})   // 3*near + far
                                 : {rdata, 2'b00};                                            // 4*near
  assign cs_store = infl & (infl_item[0] | ~vf[ic]);

  // ---------------------------------------------------------------- filter output per component
  function automatic logic [7:0] upval(input logic [9:0] c, input logic [9:0] n, input logic [3:0] bias);
    logic [11:0] t;
    t = {1'b0, c, 1'b0} + {2'b00, c} + {2'b00, n} + {8'd0, bias};
    upval = t[11:4];
  endfunction
  logic [7:0] val [0:2];
  logic [9:0] nb;
  logic [3:0] bias;
  always_comb begin : g_filt
    integer k;
    for (k = 0; k < 3; k = k + 1) begin
      nb   = hf[k] ? (x[0] ? csr[k] : csl[k]) : csc[k];
      if (hf[k]) bias = vf[k] ? (x[0] ? 4'd7 : 4'd8) : (x[0] ? 4'd8 : 4'd4);   // h2v2 / h2v1
      else       bias = vf[k] ? (vline ? 4'd8 : 4'd4) : 4'd0;                    // h1v2 / none
      val[k] = upval(csc[k], nb, bias);
    end
  end

  // ---------------------------------------------------------------- colour conversion (O stage input)
  logic [7:0] cv_r, cv_g, cv_b;
  jpeg_ycc2rgb #(.CC_TURBO(CC_TURBO)) u_cc (.y(v_a), .cb(v_b), .cr(v_c), .r(cv_r), .g(cv_g), .b(cv_b));

  assign cp_we    = wpend & ((state == R_COPY) | (state == R_COPY_END));
  assign cp_waddr = wpend_addr;
  assign cp_wdata = rdata;
  assign idle     = (state == R_IDLE) & ~v_valid & ~px_valid;

  logic last_px, copy_next;
  logic [1:0] cc_n;
  assign last_px   = (x == wm1);
  assign cc_n      = cc + 2'd1;
  assign copy_next = ({1'b0, cc} + 3'd1) < {1'b0, nproc};

  // row the line buffer copy starts from: last row of plane c = end of plane c - pitch
  logic [AW-1:0] copy_src0, copy_src_n;
  assign copy_src0  = gA(pend, 0) - gA(pw, 0);
  assign copy_src_n = gA(pend, cc_n) - gA(pw, cc_n);

  integer c;
  always_ff @(posedge clk) begin
    done <= 1'b0;
    if (rst) begin
      state <= R_IDLE; mode_r <= CMD_LINES; my_r <= '0; lines_left <= '0; l <= '0; x <= '0; y <= '0; wm1 <= '0;
      vline <= 1'b0; lastrow_r <= 1'b0; lastcr <= '0; mask <= '0; infl <= 1'b0; infl_item <= '0; colh <= '0;
      cc <= '0; ccnt <= '0; csrc <= '0; cdst <= '0; wpend <= 1'b0; wpend_addr <= '0;
      v_valid <= 1'b0; v_conv <= 1'b0; v_sof <= 1'b0; v_eol <= 1'b0; v_x <= '0; v_y <= '0;
      v_a <= '0; v_b <= '0; v_c <= '0;
      px_valid <= 1'b0; px_x <= '0; px_y <= '0; px_c0 <= '0; px_c1 <= '0; px_c2 <= '0;
      px_sof <= 1'b0; px_eol <= 1'b0;
      for (c = 0; c < 3; c = c + 1) begin
        nrow[c] <= '0; fmode[c] <= '0; csl[c] <= '0; csc[c] <= '0; csr[c] <= '0; ntmp[c] <= '0;
      end
    end else begin
      // ---- O stage
      if (px_valid & px_ready) px_valid <= 1'b0;
      if (o_take) begin
        px_valid <= 1'b1; px_x <= v_x; px_y <= v_y; px_sof <= v_sof; px_eol <= v_eol;
        px_c0 <= (RGB_OUT && v_conv) ? cv_r : v_a;
        px_c1 <= (RGB_OUT && v_conv) ? cv_g : v_b;
        px_c2 <= (RGB_OUT && v_conv) ? cv_b : v_c;
        v_valid <= 1'b0;
      end

      // ---- captured read (R_FETCH): column sums
      if (state == R_FETCH && infl) begin
        if (!infl_item[0]) ntmp[ic] <= rdata;
        if (cs_store) begin
          if (hf[ic]) begin
            if (x == 16'd0) begin csl[ic] <= cs_new; csc[ic] <= cs_new; end
            else csr[ic] <= cs_new;
          end else csc[ic] <= cs_new;
        end
      end

      case (state)
        R_IDLE: if (start) begin
          mode_r <= mode; my_r <= my; l <= '0; wm1 <= img_w - 16'd1;
          if (mode == CMD_LINES)      y <= y0;
          else if (mode == CMD_DEFER) y <= y + 16'd1;   // continues after the last line emitted
          if (mode == CMD_COPY) begin
            cc <= 2'd0; csrc <= copy_src0; cdst <= gA(lb, 0); ccnt <= {1'b0, gA(pw, 0)}; wpend <= 1'b0;
            state <= R_COPY;
          end else begin
            lines_left <= (mode == CMD_DEFER) ? 5'd1 : nlines;
            lastrow_r <= lastrow;
            lastcr    <= (img_h[3:0] - 4'd1) >> 1;
            for (c = 0; c < 3; c = c + 1)
              nrow[c] <= (mode == CMD_DEFER) ? gA(lb, c) : gA(base_pk, c);
            state <= R_LINE;
          end
        end

        // ---- line setup: where each component's far row is, then pixel 0
        R_LINE: begin
          vline <= (mode_r == CMD_DEFER) ? 1'b1 : l[0];
          for (c = 0; c < 3; c = c + 1) begin
            if (mode_r == CMD_DEFER)
              fmode[c] <= 2'd3;                                       // row 0 of the next MCU row
            else if (!l[0])                                            // upper row of a pair: row above
              fmode[c] <= (l[3:1] != 3'd0) ? 2'd1 : ((my_r == 13'd0) ? 2'd0 : 2'd3);   // (line buffer)
            else                                                       // lower row: row below, clamped
              fmode[c] <= (lastrow_r && (l[3:1] >= lastcr)) ? 2'd0 : 2'd2;   // no row below the last one
          end
          x <= 16'd0; colh <= colh_n; mask <= mask_n; infl <= 1'b0;
          state <= R_FETCH;                                            // pixel 0 always reads
        end

        // ---- one read per cycle; the last read's data is captured on the way to R_HAND
        R_FETCH: begin
          if (mask != 6'd0) begin
            mask[item] <= 1'b0; infl <= 1'b1; infl_item <= item;
          end else begin
            infl <= 1'b0;
            state <= R_HAND;
          end
        end

        // ---- hand the pixel to the output pipeline, set up the next one
        R_HAND: if (v_free) begin
          v_valid <= 1'b1; v_x <= x; v_y <= y;
          v_sof <= (x == 16'd0) && (y == 16'd0); v_eol <= last_px;
          if (gray || fmt == FMT_Y || nproc == 2'd1) begin
            v_a <= val[0];
            v_b <= (RGB_OUT && gray && fmt == FMT_RGB) ? val[0] : 8'd128;
            v_c <= (RGB_OUT && gray && fmt == FMT_RGB) ? val[0] : 8'd128;
            v_conv <= 1'b0;
          end else begin
            v_a <= val[0]; v_b <= val[1]; v_c <= val[2];
            v_conv <= RGB_OUT && (fmt == FMT_RGB);
          end
          if (last_px) begin
            if (lines_left == 5'd1) begin
              done <= 1'b1; state <= R_IDLE;
            end else begin
              lines_left <= lines_left - 5'd1;
              for (c = 0; c < 3; c = c + 1)                                    // component row advances?
                if (!uv[c] || l[0]) nrow[c] <= nrow[c] + gA(pw, c);
              l <= l + 4'd1; y <= y + 16'd1;
              state <= R_LINE;
            end
          end else begin
            x <= xs; colh <= colh_n; mask <= mask_n; infl <= 1'b0;
            for (c = 0; c < 3; c = c + 1)
              if (shift_n[c]) begin csl[c] <= csc[c]; csc[c] <= csr[c]; end
            state <= (mask_n != 6'd0) ? R_FETCH : R_HAND;
          end
        end

        // ---- copy the last plane row of each component into the (contiguous) line buffer
        R_COPY: begin
          if (ccnt != '0) begin
            csrc <= csrc + 1'b1; cdst <= cdst + 1'b1; ccnt <= ccnt - 1'b1;
            wpend <= 1'b1; wpend_addr <= cdst;
          end else begin
            wpend <= 1'b0;
            if (copy_next) begin
              cc <= cc_n; csrc <= copy_src_n; ccnt <= {1'b0, gA(pw, cc_n)};
            end else state <= R_COPY_END;
          end
        end
        R_COPY_END: begin done <= 1'b1; wpend <= 1'b0; state <= R_IDLE; end
        default: state <= R_IDLE;
      endcase
    end
  end
endmodule
