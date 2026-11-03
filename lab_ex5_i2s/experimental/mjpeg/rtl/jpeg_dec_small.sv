// jpeg_dec_small.sv - compact core of jpeg_decoder (FAST = 0): the area-first datapath that fits
// a Cyclone II EP2C5 in every build option.
//
//   bytes in  -> jpeg_parser -> table RAMs + token FIFO
//             -> jpeg_bitreader -> jpeg_coefdec -> block RAM -> jpeg_idct
//             -> RASTER_OUT = 0: MCU sample buffer -> jpeg_pixgen  (pixels in MCU order)
//                RASTER_OUT = 1: MCU-row buffer    -> jpeg_raster  (pixels in raster order)
//
// Blocks of an MCU are decoded and transformed one after another (T.81 A.2.3 order: for each
// scan component, for v, for h).  Nothing is overlapped, which keeps the control simple and
// the design small: ~13.5 clocks per pixel for 4:2:0 colour.  See jpeg_decoder for the ports
// and parameters.
module jpeg_dec_small #(
  parameter bit RASTER_OUT     = 1'b0,
  parameter int ROWBUF_BYTES   = 16384,
  parameter bit FANCY_UPSAMPLE = 1'b0,
  parameter bit CC_TURBO       = 1'b0,
  parameter bit RGB_OUT        = 1'b1,
  parameter bit CHECKS         = 1'b1
) (
  input  logic        clk,
  input  logic        rst,
  // JPEG byte stream
  input  logic        in_valid,
  input  logic [7:0]  in_data,
  input  logic        in_last,
  output logic        in_ready,
  // output format for the next frame (FMT_RGB / FMT_YCBCR / FMT_Y)
  input  logic [1:0]  out_fmt,
  // decoded pixels
  output logic        px_valid,
  input  logic        px_ready,
  output logic [15:0] px_x,
  output logic [15:0] px_y,
  output logic [7:0]  px_c0,         // R | Y
  output logic [7:0]  px_c1,         // G | Cb   (128 for FMT_Y)
  output logic [7:0]  px_c2,         // B | Cr   (128 for FMT_Y)
  output logic        px_sof,        // first pixel of the frame (x = 0, y = 0)
  output logic        px_eol,        // last pixel of an image row (x = width-1)
  // status
  output logic [15:0] img_w,
  output logic [15:0] img_h,
  output logic        frame_start,   // pulse: header parsed, img_w/img_h valid, pixels follow
  output logic        frame_done,    // pulse: last pixel has been accepted
  output logic [12:0] err            // sticky error flags (see jpeg_pkg), cleared by rst
);
  import jpeg_pkg::*;

  localparam bit FANCY    = FANCY_UPSAMPLE & RASTER_OUT;       // the filter needs the row buffer
  localparam int RB_AW    = (ROWBUF_BYTES > 2) ? $clog2(ROWBUF_BYTES) : 1;
  localparam int GW       = RB_AW + 4;                         // row-buffer geometry width: with
                                                               // mcux <= MAX_MCUX all sizes fit
  localparam int MAX_MCUX = ROWBUF_BYTES / 64;                 // every MCU column needs >= 64 bytes

  // ---------------------------------------------------------------- parser
  logic        p_in_valid, p_in_ready, p_idle, p_soi;
  logic        dqt_we, hc_we, hv_we;
  logic [7:0]  dqt_waddr, dqt_wdata, hv_wdata;
  logic [5:0]  hc_waddr;
  logic [24:0] hc_wdata;
  logic [8:0]  hv_waddr;
  logic [1:0]  nf, ns;
  logic        cs_rgb;           // RGB image (parser): no colour conversion
  logic [11:0] comp_h, comp_v;
  logic [7:0]  comp_tq, scan_ci;
  logic [3:0]  scan_td, scan_ta;
  logic [15:0] ri;
  logic        scan_start, eoi;
  logic        tok_valid, tok_ready;
  logic [8:0]  tok_data;
  logic [12:0] err_set;
  logic        decoding;
  logic        skip_img, eoi_seen;   // unusable image: consume its scans until its EOI

  // while a frame is being decoded, hold back bytes of a following file
  assign p_in_valid = in_valid & ~(decoding & p_idle);
  assign in_ready   = p_in_ready & ~(decoding & p_idle);

  jpeg_parser #(.CHECKS(CHECKS)) u_parser (
    .clk(clk), .rst(rst),
    .in_valid(p_in_valid), .in_data(in_data), .in_last(in_last), .in_ready(p_in_ready), .idle(p_idle),
    .dqt_we(dqt_we), .dqt_waddr(dqt_waddr), .dqt_wdata(dqt_wdata),
    .hc_we(hc_we), .hc_waddr(hc_waddr), .hc_wdata(hc_wdata),
    .hv_we(hv_we), .hv_waddr(hv_waddr), .hv_wdata(hv_wdata),
    .lut_we(), .lut_waddr(), .lut_wdata(),
    .img_w(img_w), .img_h(img_h), .nf(nf), .cs_rgb(cs_rgb), .comp_h(comp_h), .comp_v(comp_v), .comp_tq(comp_tq),
    .ns(ns), .scan_ci(scan_ci), .scan_td(scan_td), .scan_ta(scan_ta), .ri(ri), .scan_start(scan_start),
    .tok_valid(tok_valid), .tok_data(tok_data), .tok_ready(tok_ready),
    .eoi(eoi), .soi(p_soi), .err_set(err_set));

  // ---------------------------------------------------------------- table RAMs
  logic [7:0]  dqt_raddr, dqt_rdata, hv_rdata;
  logic [5:0]  hc_raddr;
  logic [24:0] hc_rdata;
  logic [8:0]  hv_raddr;
  jpeg_sdp_ram #(.WIDTH(8),  .DEPTH_LOG2(8))  u_dqt (.clk(clk), .we(dqt_we), .waddr(dqt_waddr), .wdata(dqt_wdata), .raddr(dqt_raddr), .rdata(dqt_rdata));
  jpeg_sdp_ram #(.WIDTH(25), .DEPTH_LOG2(6))  u_hc  (.clk(clk), .we(hc_we),  .waddr(hc_waddr),  .wdata(hc_wdata),  .raddr(hc_raddr),  .rdata(hc_rdata));
  jpeg_sdp_ram #(.WIDTH(8),  .DEPTH_LOG2(9))  u_hv  (.clk(clk), .we(hv_we),  .waddr(hv_waddr),  .wdata(hv_wdata),  .raddr(hv_raddr),  .rdata(hv_rdata));

  // ---------------------------------------------------------------- bit reader
  logic bit_valid, bit_data, bit_take, restart_req, restart_done, err_pad, br_clear, br_at_end;
  jpeg_bitreader u_br (
    .clk(clk), .rst(rst), .clear(br_clear),
    .tok_valid(tok_valid), .tok_data(tok_data), .tok_ready(tok_ready),
    .bit_valid(bit_valid), .bit_data(bit_data), .bit_take(bit_take),
    .restart_req(restart_req), .restart_done(restart_done), .err_pad(err_pad), .at_end(br_at_end));

  // ---------------------------------------------------------------- block decoder + block RAM
  logic        cd_start, cd_done, cd_err, pred_clear;
  logic [1:0]  cd_comp, cd_tq;
  logic        cd_dc, cd_ac;
  logic        blk_we, id_pass2;
  logic [5:0]  blk_waddr, blk_raddr;
  logic signed [15:0] blk_wdata, blk_rdata;
  jpeg_coefdec u_cd (
    .clk(clk), .rst(rst),
    .start(cd_start), .comp(cd_comp), .dc_tbl(cd_dc), .ac_tbl(cd_ac), .tq(cd_tq), .pred_clear(pred_clear),
    .done(cd_done), .err_huff(cd_err),
    .bit_valid(bit_valid), .bit_data(bit_data), .bit_take(bit_take),
    .hc_raddr(hc_raddr), .hc_rdata(hc_rdata), .hv_raddr(hv_raddr), .hv_rdata(hv_rdata),
    .dqt_raddr(dqt_raddr), .dqt_rdata(dqt_rdata),
    .blk_we(blk_we), .blk_waddr(blk_waddr), .blk_wdata(blk_wdata));
  logic skip_idct;                       // block not transformed (assigned with the control below)
  jpeg_blockram u_blk (
    .clk(clk), .rst(rst), .zero_start(id_pass2), .we(blk_we & ~skip_idct), .waddr(blk_waddr), .wdata(blk_wdata),
    .raddr(blk_raddr), .rdata(blk_rdata));    // skipped blocks never dirty the buffer

  // ---------------------------------------------------------------- IDCT
  logic        id_start, id_done, id_busy, smp_we;
  logic [5:0]  smp_waddr;
  logic [7:0]  smp_wdata;
  jpeg_idct u_idct (
    .clk(clk), .rst(rst), .start(id_start), .done(id_done), .busy(id_busy), .pass2_start(id_pass2),
    .blk_raddr(blk_raddr), .blk_rdata(blk_rdata),
    .smp_we(smp_we), .smp_waddr(smp_waddr), .smp_wdata(smp_wdata));

  // ---------------------------------------------------------------- frame state shared with the output stages
  logic [15:0] mcu_x0, mcu_y0;
  logic        gray, hmax2, vmax2;
  logic [2:0]  h2, v2;
  logic [11:0] blk_base;
  logic [1:0]  fmt_r;
  logic [3:0]  slot;
  logic        pg_start, pg_done;          // MCU mode
  logic        out_idle;                   // output stage drained
  // raster mode
  logic        rs_start, rs_done;
  logic [1:0]  rs_mode;
  logic [4:0]  rs_nlines;
  logic [15:0] rs_y0;
  logic [12:0] mx, my;
  logic [1:0]  nstore;                     // components kept: 1 (grey / FMT_Y) or 3
  logic [2:0]  uh_r, uv_r, hf_r, vf_r;
  logic [GW-1:0] pend_g [0:2], lb_g [0:2], total;  // end of plane c, line-buffer row of c, bytes used
  logic [RB_AW-1:0] pw_a [0:2];              // plane row pitch mcux*8*Hc (combinational)
  logic [RB_AW-1:0] rb_row, blk_rb;          // IDCT write pointer into the row buffer

  logic more_y;                         // another MCU row follows (declared before its use in the raster
                                       // stage's port list: a forward reference there is an implicit net)
  generate
    if (!RASTER_OUT) begin : g_mcu
      // ---- MCU sample buffer (up to 10 blocks) + pixel generator
      logic [7:0] buf_rdata;
      logic [9:0] buf_raddr;
      jpeg_sdp_ram #(.WIDTH(8), .DEPTH_LOG2(10)) u_mcu (
        .clk(clk), .we(smp_we), .waddr({slot, smp_waddr}), .wdata(smp_wdata), .raddr(buf_raddr), .rdata(buf_rdata));
      jpeg_pixgen #(.CC_TURBO(CC_TURBO), .RGB_OUT(RGB_OUT)) u_pg (
        .clk(clk), .rst(rst), .start(pg_start), .done(pg_done),
        .img_w(img_w), .img_h(img_h), .mcu_x0(mcu_x0), .mcu_y0(mcu_y0),
        .gray(gray), .fmt(fmt_r), .hmax2(hmax2), .vmax2(vmax2), .h2(h2), .v2(v2), .blk_base(blk_base),
        .buf_raddr(buf_raddr), .buf_rdata(buf_rdata),
        .px_valid(px_valid), .px_ready(px_ready), .px_x(px_x), .px_y(px_y),
        .px_c0(px_c0), .px_c1(px_c1), .px_c2(px_c2), .px_sof(px_sof), .px_eol(px_eol));
      assign out_idle = ~px_valid;
      assign rs_done  = 1'b0;
    end else begin : g_raster
      // ---- MCU-row buffer (component planes + line buffer) + raster output stage
      logic [RB_AW-1:0] rb_raddr, rb_waddr, cp_waddr;
      logic [7:0]       rb_rdata, rb_wdata, cp_wdata;
      logic             rb_we, cp_we;
      assign rb_we    = smp_we | cp_we;
      assign rb_waddr = cp_we ? cp_waddr : {rb_row[RB_AW-1:3], smp_waddr[2:0]};
      assign rb_wdata = cp_we ? cp_wdata : smp_wdata;
      jpeg_sdp_ram #(.WIDTH(8), .DEPTH_LOG2(RB_AW), .DEPTH(ROWBUF_BYTES)) u_rowbuf (
        .clk(clk), .we(rb_we), .waddr(rb_waddr), .wdata(rb_wdata), .raddr(rb_raddr), .rdata(rb_rdata));
      jpeg_raster #(.AW(RB_AW), .FANCY(FANCY), .CC_TURBO(CC_TURBO), .RGB_OUT(RGB_OUT)) u_rs (
        .clk(clk), .rst(rst),
        .start(rs_start), .mode(rs_mode), .nlines(rs_nlines), .y0(rs_y0), .my(my), .lastrow(~more_y),
        .done(rs_done), .idle(out_idle),
        .img_w(img_w), .img_h(img_h), .nproc(nstore), .gray(gray), .fmt(fmt_r),
        .uh(uh_r), .uv(uv_r), .hf(hf_r), .vf(vf_r),
        .pw  ({pw_a[2], pw_a[1], pw_a[0]}),
        .pend({pend_g[2][RB_AW-1:0], pend_g[1][RB_AW-1:0], pend_g[0][RB_AW-1:0]}),
        .lb  ({lb_g[2][RB_AW-1:0],   lb_g[1][RB_AW-1:0],   lb_g[0][RB_AW-1:0]}),
        .raddr(rb_raddr), .rdata(rb_rdata), .cp_we(cp_we), .cp_waddr(cp_waddr), .cp_wdata(cp_wdata),
        .px_valid(px_valid), .px_ready(px_ready), .px_x(px_x), .px_y(px_y),
        .px_c0(px_c0), .px_c1(px_c1), .px_c2(px_c2), .px_sof(px_sof), .px_eol(px_eol));
      assign pg_done = 1'b0;
    end
  endgenerate

  // ---------------------------------------------------------------- frame / MCU / block sequencing
  typedef enum logic [4:0] { S_IDLE, S_SETUP, S_GEO1, S_GEO2, S_GEO3, S_GEO4, S_GEO5, S_GEO6,
                             S_BLK_START, S_BLK_WAIT, S_IDCT_WAIT, S_NEXT_BLK,
                             S_PIX_START, S_PIX_WAIT, S_NEXT_MCU,
                             S_ROW_DEF, S_ROW_DEF_W, S_ROW_EMIT, S_ROW_EMIT_W, S_ROW_COPY, S_ROW_COPY_W,
                             S_ROW_NEXT, S_RESTART, S_DRAIN, S_SKIP, S_DONE } state_t;
  state_t state;

  // per scan component j: frame component index, Hj==2, Vj==2, first block slot
  logic [1:0]  j;                    // scan component
  logic        vv, hh;               // block position inside the component's MCU area
  logic [15:0] mcu_w, mcu_h;         // MCU size in pixels
  logic [15:0] rst_cnt;              // MCUs left until the next RSTn marker
  logic        rst_due;              // an RSTn marker follows the MCU just finished
  logic        hmax2_c, vmax2_c;
  logic        suppress;             // raster mode: image too wide, decode without output
  logic        defer, defer_pending; // raster mode: last line of each MCU row waits for the next row
  logic        w_err;
  logic [13:0] mcux;                   // MCUs per row (up to 8192 for 8-pixel MCUs)
  logic [12:0] mcux_m1, mcuy_m1;       // index of the last MCU column / row: (W-1) >> log2(MCU width)
  logic [GW-1:0] mcux_g;
  assign mcux_g = mcux;                // zero-extended to the geometry width

  // component geometry helpers (packed: 3 bits per component in comp_h/comp_v)
  function automatic logic is2(input logic [11:0] hv, input logic [1:0] c);
    is2 = (hv[3*c +: 3] == 3'd2);
  endfunction
  function automatic logic [1:0] ci_of(input logic [7:0] sc, input logic [1:0] jj);
    ci_of = sc[2*jj +: 2];
  endfunction

  // max over the scan components
  assign hmax2_c = (nf == 2'd1) ? 1'b0 : (is2(comp_h, ci_of(scan_ci, 2'd0)) | is2(comp_h, ci_of(scan_ci, 2'd1)) | is2(comp_h, ci_of(scan_ci, 2'd2)));
  assign vmax2_c = (nf == 2'd1) ? 1'b0 : (is2(comp_v, ci_of(scan_ci, 2'd0)) | is2(comp_v, ci_of(scan_ci, 2'd1)) | is2(comp_v, ci_of(scan_ci, 2'd2)));

  // current block parameters
  logic [1:0] cur_ci;
  logic       cur_h2, cur_v2;
  logic [3:0] cur_base;
  assign cur_ci   = ci_of(scan_ci, j);
  assign cur_h2   = h2[j];
  assign cur_v2   = v2[j];
  assign cur_base = blk_base[4*j +: 4];
  assign cd_comp  = cur_ci;
  assign cd_dc    = scan_td[j];
  assign cd_ac    = scan_ta[j];
  assign cd_tq    = comp_tq[2*cur_ci +: 2];
  assign slot     = cur_base + (vv ? (cur_h2 ? 4'd2 : 4'd1) : 4'd0) + {3'd0, hh};

  logic last_blk_of_comp, last_comp, more_x;
  logic [15:0] wm1_c, hm1_c;
  localparam logic [12:0] hdr_err_mask = (13'd1 << ERR_SOF_TYPE) | (13'd1 << ERR_PRECISION) | (13'd1 << ERR_DQT)
                                       | (13'd1 << ERR_DHT) | (13'd1 << ERR_NCOMP) | (13'd1 << ERR_SAMPLING)
                                       | (13'd1 << ERR_SCAN) | (13'd1 << ERR_FRAME) | (13'd1 << ERR_TRUNC);
  assign wm1_c = img_w - 16'd1;
  assign hm1_c = img_h - 16'd1;
  assign last_blk_of_comp = (hh == cur_h2) && (vv == cur_v2);
  assign last_comp        = (j == ns - 2'd1);
  // FMT_Y: chroma blocks only need entropy decoding; raster overflow: nothing is transformed
  assign skip_idct        = ((fmt_r == FMT_Y) && (j != 2'd0)) || (RASTER_OUT && suppress);
  assign more_x           = (mx != mcux_m1);           // another MCU follows in this row
  assign more_y           = (my != mcuy_m1);           // another MCU row follows

  logic [3:0] nblk0, nblk1;           // blocks per MCU of scan components 0 and 1
  assign nblk0 = (is2(comp_h, ci_of(scan_ci, 2'd0)) ? 4'd2 : 4'd1) << (is2(comp_v, ci_of(scan_ci, 2'd0)) ? 1 : 0);
  assign nblk1 = (is2(comp_h, ci_of(scan_ci, 2'd1)) ? 4'd2 : 4'd1) << (is2(comp_v, ci_of(scan_ci, 2'd1)) ? 1 : 0);

  // row-buffer geometry, all derived from the MCU count by shifts:
  //   pitch of plane c  pw_c = mcux*8*Hc,   plane size ps_c = pw_c*8*Vc,
  //   layout [plane 0 | plane 1 | plane 2 | line buffer: one row of each component]
  logic [GW-1:0] ps_a [0:2];
  logic [31:0]   pwt, pst;
  always_comb begin : g_plane
    integer gi;                         // loop index over the three components
    for (gi = 0; gi < 3; gi = gi + 1) begin
      pwt = {18'd0, mcux} << (3 + h2[gi]);
      pst = {18'd0, mcux} << (6 + h2[gi] + v2[gi]);
      pw_a[gi] = pwt[RB_AW-1:0];
      ps_a[gi] = pst[GW-1:0];
    end
  end
  // row-buffer address of the current block: plane base + block row + MCU column (all
  // multiples of 8, so the IDCT's column index can be concatenated instead of added)
  logic [31:0] blk_rb_n, base_j;
  always_comb begin
    base_j   = (j == 2'd0) ? 32'd0 : (j == 2'd1) ? {{(32-GW){1'b0}}, pend_g[0]} : {{(32-GW){1'b0}}, pend_g[1]};
    blk_rb_n = base_j
             + (vv ? {{(32-RB_AW){1'b0}}, pw_a[j]} << 3 : 32'd0)
             + ((cur_h2 ? {18'd0, mx, hh} : {19'd0, mx}) << 3);
  end

  always_ff @(posedge clk) begin : g_seq
    integer gi;
    cd_start <= 1'b0; id_start <= 1'b0; pg_start <= 1'b0; pred_clear <= 1'b0; rs_start <= 1'b0;
    frame_start <= 1'b0; frame_done <= 1'b0; br_clear <= 1'b0; w_err <= 1'b0;
    if (rst) begin
      state <= S_IDLE; decoding <= 1'b0; restart_req <= 1'b0; err <= '0; skip_img <= 1'b0; eoi_seen <= 1'b0;
      j <= '0; vv <= 1'b0; hh <= 1'b0; mcu_x0 <= '0; mcu_y0 <= '0; mcu_w <= '0; mcu_h <= '0; rst_cnt <= '0; rst_due <= 1'b0;
      gray <= 1'b0; hmax2 <= 1'b0; vmax2 <= 1'b0; h2 <= '0; v2 <= '0; blk_base <= '0; fmt_r <= FMT_RGB;
      mx <= '0; my <= '0; mcux <= '0; mcux_m1 <= '0; mcuy_m1 <= '0; nstore <= 2'd3; uh_r <= '0; uv_r <= '0; hf_r <= '0; vf_r <= '0;
      total <= '0; suppress <= 1'b0; defer <= 1'b0; defer_pending <= 1'b0;
      rs_mode <= CMD_LINES; rs_nlines <= '0; rs_y0 <= '0; rb_row <= '0; blk_rb <= '0;
      for (gi = 0; gi < 3; gi = gi + 1) begin pend_g[gi] <= '0; lb_g[gi] <= '0; end
    end else begin
      // errors of the current image: cleared when the next image's SOI is accepted (the parser
      // is held at that SOI until frame_done, so err describes the last frame until then)
      err <= (p_soi ? 13'd0 : err) | err_set | ({12'd0, cd_err} << ERR_HUFF) | ({12'd0, err_pad & (state != S_DRAIN) & (state != S_SKIP)} << ERR_MARKER)
                 | ({12'd0, w_err} << ERR_WIDTH);

      // IDCT write pointer: rows of a block are written in order, 8 samples each
      if (RASTER_OUT && smp_we && smp_waddr[2:0] == 3'd7) rb_row <= rb_row + pw_a[j];

      if (eoi) eoi_seen <= 1'b1;
      case (state)
        S_IDLE: if (scan_start) begin
          decoding <= 1'b1; frame_start <= 1'b1; eoi_seen <= 1'b0;
          // unusable headers (no/unsupported SOF, bad tables or scan): no pixels, consume the scans
          if (img_w == 16'd0 || img_h == 16'd0) err[ERR_FRAME] <= 1'b1;   // X or Y = 0 (DNL), or no SOF
          if (cs_rgb && out_fmt != FMT_RGB) err[ERR_SOF_TYPE] <= 1'b1;     // an RGB image: RGB output only
          if (img_w == 16'd0 || img_h == 16'd0 || (hdr_err_mask & (err_set | err)) != 13'd0 || (cs_rgb && out_fmt != FMT_RGB)) begin
            skip_img <= 1'b1; state <= S_DRAIN;
          end else
            state <= S_SETUP;
        end
        S_SETUP: begin
          gray  <= (nf == 2'd1);
          fmt_r <= cs_rgb ? FMT_YCBCR : out_fmt;    // RGB image: no colour conversion
          nstore <= ((nf == 2'd1) || (out_fmt == FMT_Y)) ? 2'd1 : 2'd3;
          hmax2 <= hmax2_c; vmax2 <= vmax2_c;
          mcu_w <= hmax2_c ? 16'd16 : 16'd8;
          mcu_h <= vmax2_c ? 16'd16 : 16'd8;
          mcux  <= hmax2_c ? ({2'b00, img_w[15:4]} + {13'd0, |img_w[3:0]}) : ({1'b0, img_w[15:3]} + {13'd0, |img_w[2:0]});
          mcux_m1 <= hmax2_c ? {1'b0, wm1_c[15:4]} : wm1_c[15:3];
          mcuy_m1 <= vmax2_c ? {1'b0, hm1_c[15:4]} : hm1_c[15:3];
          if (nf == 2'd1) begin
            h2 <= 3'b000; v2 <= 3'b000; blk_base <= 12'd0;
          end else begin
            h2 <= {is2(comp_h, ci_of(scan_ci, 2'd2)), is2(comp_h, ci_of(scan_ci, 2'd1)), is2(comp_h, ci_of(scan_ci, 2'd0))};
            v2 <= {is2(comp_v, ci_of(scan_ci, 2'd2)), is2(comp_v, ci_of(scan_ci, 2'd1)), is2(comp_v, ci_of(scan_ci, 2'd0))};
            blk_base <= {nblk0 + nblk1, nblk0, 4'd0};
          end
          mcu_x0 <= '0; mcu_y0 <= '0; mx <= '0; my <= '0; j <= '0; vv <= 1'b0; hh <= 1'b0;
          rst_cnt <= ri; pred_clear <= 1'b1; suppress <= 1'b0; defer_pending <= 1'b0;
          state <= RASTER_OUT ? S_GEO1 : S_BLK_START;
        end
        // ---- raster mode: row-buffer layout  [plane 0 | plane 1 | plane 2 | line buffer]
        S_GEO1: begin
          for (gi = 0; gi < 3; gi = gi + 1) begin
            uh_r[gi] <= hmax2 & ~h2[gi];
            uv_r[gi] <= vmax2 & ~v2[gi];
            // libjpeg-turbo: the horizontal / 2x2 filters need a component wider than 2 samples
            // (ceil(W/2) > 2, i.e. W > 4); 1x2 always filters
            hf_r[gi] <= FANCY & (hmax2 & ~h2[gi]) & (img_w > 16'd4);
            vf_r[gi] <= FANCY & (vmax2 & ~v2[gi]) & (~(hmax2 & ~h2[gi]) | (img_w > 16'd4));
          end
          pend_g[0] <= ps_a[0];
          pend_g[1] <= ps_a[0] + ps_a[1];
          state <= S_GEO2;
        end
        S_GEO2: begin
          pend_g[2] <= pend_g[1] + ps_a[2];
          defer <= (nstore == 2'd1) ? vf_r[0] : |vf_r;
          state <= S_GEO3;
        end
        S_GEO3: begin
          lb_g[0] <= (nstore == 2'd1) ? pend_g[0] : pend_g[2];
          state <= S_GEO4;
        end
        S_GEO4: begin
          lb_g[1] <= lb_g[0] + {{(GW-RB_AW){1'b0}}, pw_a[0]};
          lb_g[2] <= lb_g[0] + {{(GW-RB_AW){1'b0}}, pw_a[0]} + {{(GW-RB_AW){1'b0}}, pw_a[1]};
          state <= S_GEO5;
        end
        S_GEO5: begin
          total <= !defer ? lb_g[0] : (nstore == 2'd1) ? lb_g[1] : lb_g[2] + {{(GW-RB_AW){1'b0}}, pw_a[2]};
          state <= S_GEO6;
        end
        S_GEO6: begin
          if ({2'd0, mcux} > MAX_MCUX || total > ROWBUF_BYTES) begin
            suppress <= 1'b1; w_err <= 1'b1;
          end
          state <= S_BLK_START;
        end
        S_BLK_START: begin
          cd_start <= 1'b1;
          blk_rb <= blk_rb_n[RB_AW-1:0];
          state <= S_BLK_WAIT;
        end
        S_BLK_WAIT: if (cd_done) begin
          if (skip_idct) state <= S_NEXT_BLK;
          else begin id_start <= 1'b1; rb_row <= blk_rb; state <= S_IDCT_WAIT; end
        end
        S_IDCT_WAIT: if (id_done) state <= S_NEXT_BLK;
        S_NEXT_BLK: begin
          if (!last_blk_of_comp) begin
            if (hh == cur_h2) begin hh <= 1'b0; vv <= 1'b1; end
            else hh <= 1'b1;
            state <= S_BLK_START;
          end else begin
            hh <= 1'b0; vv <= 1'b0;
            if (!last_comp) begin j <= j + 2'd1; state <= S_BLK_START; end
            else begin j <= '0; state <= RASTER_OUT ? S_NEXT_MCU : S_PIX_START; end
          end
        end
        S_PIX_START: begin pg_start <= 1'b1; state <= S_PIX_WAIT; end
        S_PIX_WAIT: if (pg_done) state <= S_NEXT_MCU;
        S_NEXT_MCU: begin
          rst_cnt <= rst_cnt - 16'd1;
          rst_due <= (ri != 16'd0) && (rst_cnt == 16'd1);
          if (more_x) begin
            mcu_x0 <= mcu_x0 + mcu_w; mx <= mx + 13'd1;
            state <= (ri != 16'd0 && rst_cnt == 16'd1) ? S_RESTART : S_BLK_START;
          end else if (RASTER_OUT) begin
            state <= S_ROW_DEF;                                 // MCU row complete: emit it
          end else if (more_y) begin
            mcu_x0 <= '0; mx <= '0; mcu_y0 <= mcu_y0 + mcu_h; my <= my + 13'd1;
            state <= (ri != 16'd0 && rst_cnt == 16'd1) ? S_RESTART : S_BLK_START;
          end else state <= S_DRAIN;
        end
        // ---- raster mode: emit the MCU row (see jpeg_raster for the protocol)
        S_ROW_DEF: begin
          if (!suppress && defer_pending) begin
            rs_start <= 1'b1; rs_mode <= CMD_DEFER; defer_pending <= 1'b0;   // row = last row emitted + 1
            state <= S_ROW_DEF_W;
          end else state <= S_ROW_EMIT;
        end
        S_ROW_DEF_W: if (rs_done) state <= S_ROW_EMIT;
        S_ROW_EMIT: begin
          if (suppress) state <= S_ROW_NEXT;
          else begin
            rs_start <= 1'b1; rs_mode <= CMD_LINES; rs_y0 <= mcu_y0;
            if (!more_y)    rs_nlines <= img_h[4:0] - mcu_y0[4:0];   // last MCU row: all remaining lines
            else if (defer) rs_nlines <= mcu_h[4:0] - 5'd1;          // keep the last line for later
            else            rs_nlines <= mcu_h[4:0];
            state <= S_ROW_EMIT_W;
          end
        end
        S_ROW_EMIT_W: if (rs_done) state <= (defer && more_y) ? S_ROW_COPY : S_ROW_NEXT;
        S_ROW_COPY: begin rs_start <= 1'b1; rs_mode <= CMD_COPY; state <= S_ROW_COPY_W; end
        S_ROW_COPY_W: if (rs_done) begin defer_pending <= 1'b1; state <= S_ROW_NEXT; end
        S_ROW_NEXT: begin
          if (more_y) begin
            mcu_x0 <= '0; mx <= '0; mcu_y0 <= mcu_y0 + mcu_h; my <= my + 13'd1;
            state <= rst_due ? S_RESTART : S_BLK_START;
          end else state <= S_DRAIN;
        end
        S_RESTART: begin                              // T.81 F.2.2.5 / B.2.4.4: RSTn between MCUs
          restart_req <= 1'b1;
          if (restart_done) begin
            restart_req <= 1'b0; pred_clear <= 1'b1; rst_cnt <= ri; state <= S_BLK_START;
          end
        end
        // ---- skip whatever is left of the entropy-coded segment (padding bits, RSTn markers, or
        //      the whole scan of an unusable frame) up to the EOI marker, so the parser and the
        //      bit reader are in step for the next file.  The bit reader's restart mechanism drops
        //      data bytes one per clock and stops at the next marker.
        S_DRAIN: begin
          restart_req <= 1'b1;
          if (restart_done) begin
            restart_req <= 1'b0;
            if (br_at_end) begin                        // EOI (or another marker) reached
              br_clear <= 1'b1; state <= S_SKIP;          // then the rest of the image, up to its EOI
            end
          end
        end
        // after the scan: consume the rest of the image (e.g. a COM segment after the scan) and end
        // the frame once, at its EOI - or at the EOI the parser appends when in_last comes first
        // (ERR_TRUNC), so frame_done always carries the image's final error.  An unusable image
        // (e.g. progressive) may have more scans: drain them; a second scan after a decoded one is
        // not supported (ERR_SCAN).
        S_SKIP: if (eoi_seen || eoi) begin skip_img <= 1'b0; state <= S_DONE; end
                else if (scan_start) begin
                  if (!skip_img) err[ERR_SCAN] <= 1'b1;
                  skip_img <= 1'b1; state <= S_DRAIN;
                end
        S_DONE: if (out_idle) begin                   // wait for the last pixel to be accepted
          frame_done <= 1'b1; decoding <= 1'b0; br_clear <= 1'b1; state <= S_IDLE;
        end
        default: state <= S_IDLE;
      endcase
    end
  end
endmodule
