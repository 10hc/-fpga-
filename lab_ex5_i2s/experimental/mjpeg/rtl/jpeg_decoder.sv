// jpeg_decoder.sv - top level of the streaming baseline JPEG decoder.
//
//   JPEG bytes in (valid/ready) -> pixels out (x, y, c0, c1, c2, sof, eol) with valid/ready
//
// Build options (parameters):
//   FAST           0: compact core (jpeg_dec_small): one block at a time, ~13.5 clocks per pixel
//                     for 4:2:0 colour; fits a Cyclone II EP2C5 in every configuration.
//                  1: pipelined core (jpeg_dec_fast): table-driven Huffman decoding, a two-lane
//                     IDCT (32 clocks per block) and 1-pixel-per-clock output, all overlapped;
//                     ~1-1.5 clocks per pixel.  Same pixels, errors and interface.
//                  2: wide core (jpeg_dec_wide): one Huffman symbol per clock, an IDCT of 8 clocks
//                     per block and four pixels per output beat (NPIX = 4), MCU order only
//                     (RASTER_OUT = 0); ~0.25-0.56 clocks per pixel on phone photos, ~6k LUTs and
//                     33 DSP48 on an Artix-7 (150 MHz).  Same pixels and errors.
//   NPIX           pixels per output beat: 4 for FAST = 2, 1 otherwise (derived; do not set).
//   RASTER_OUT     0: pixels leave in MCU order (8x8/16x16 tiles) with their coordinates; needs
//                     only a small MCU buffer and handles any image width.
//                  1: pixels leave row by row (x = 0..W-1, then the next row): the decoded MCU
//                     row is kept in a ROWBUF_BYTES row buffer.  Images whose MCU row does not
//                     fit set err[ERR_WIDTH] and produce no pixels (the file is still consumed).
//   ROWBUF_BYTES   row-buffer size (RASTER_OUT = 1).  Bytes needed per 16 pixels of width:
//                  grey 128, 4:2:2 256, 4:2:0 384 (+32 with FANCY), 4:4:4 384, 4:4:0 512 (+48);
//                  FMT_Y stores luma only (4:2:0: 256).
//   ROWBUF_Y_BYTES FAST raster build: the row buffer is one RAM per component (so all three are
//   ROWBUF_C_BYTES read in parallel): component 0 gets ROWBUF_Y_BYTES, components 1 and 2
//                  ROWBUF_C_BYTES each (0 = ROWBUF_BYTES/2 and /4).  Per 16 pixels of width a
//                  component needs 16*Hc*Vc bytes per plane (+2*Hc per line-buffer row with FANCY,
//                  2 rows); with room for two planes (+3 line-buffer rows) decoding and output of
//                  consecutive MCU rows overlap (about twice as fast), otherwise they alternate.
//   FANCY_UPSAMPLE libjpeg-turbo's triangle-filter chroma upsampling (the default of Pillow and
//                  OpenCV) instead of sample replication.  Requires RASTER_OUT = 1.
//   CC_TURBO       YCbCr->RGB constants of libjpeg-turbo (Pillow, OpenCV) instead of libjpeg 9.
//   RGB_OUT        0 leaves the YCbCr->RGB converter out (~280 LEs) for consumers that only want
//                  YCbCr or luma; FMT_RGB then delivers YCbCr.
//   CHECKS         1: header/table validation (see jpeg_parser: tables complete and legal before
//                     use, segment lengths, component selectors, RSTn sequence).  0 saves ~100
//                     logic elements for the smallest devices; unsupported features, table ids
//                     and the end-of-input contract (in_last) are still handled.
//   RASTER_OUT=1, FANCY_UPSAMPLE=1, CC_TURBO=1 reproduces Pillow / OpenCV / turbo djpeg exactly;
//   the defaults reproduce libjpeg 9 `djpeg -dct int -nosmooth` exactly.
//
// Output formats (out_fmt, sampled when a frame starts): FMT_RGB, FMT_YCBCR (upsampled Y,Cb,Cr,
// no colour conversion), FMT_Y (luma only: chroma blocks are entropy-decoded to keep the
// bitstream in step but not transformed).  See jpeg_pkg.
//
// Supported: baseline sequential DCT (SOF0), 8-bit, Huffman coding, 1 or 3 components,
// sampling factors 1 or 2 (4:4:4, 4:2:2, 4:2:0, 4:4:0 ...), one interleaved scan (its components
// in frame order, as T.81 B.2.3 requires), restart
// intervals, any APPn/COM segments, multiple tables per DQT/DHT segment.
// Not supported (flagged in `err`): progressive/lossless/arithmetic, 12-bit, 16-bit quant
// tables, non-interleaved multi-scan files, sampling factors 3 or 4, DNL (height 0).
//
// Completion contract.  Files may follow each other back to back without reset.  err holds the
// errors of the current image: it is cleared when the next image's SOI is accepted (the input is
// held at that SOI until the previous frame_done), so read it at frame_done.  Every image that
// reaches a scan (SOS) ends with exactly one frame_start and one frame_done; an image with a
// header or table error (see jpeg_pkg: invalid, missing or incomplete DQT/DHT/SOF/SOS, unsupported
// type) produces no pixels.  If err is not zero at frame_done, discard the frame's pixels.
// in_last = 1 on the last byte of a file lets the decoder finish a truncated file: before its
// EOI it raises ERR_TRUNC and still ends the frame: missing entropy-coded data is decoded from zero
// bits (libjpeg fills the rest of the scan with uniform grey instead - discard the frame anyway);
// a file cut inside its headers gives a frame without pixels.  With in_last
// tied to 0 a truncated file leaves the decoder waiting for more input: reset it to abort.
// A file without a scan (e.g. tables only, or no SOI at all) produces no frame.
module jpeg_decoder #(
  parameter int FAST           = 0,
  parameter bit RASTER_OUT     = 1'b0,
  parameter int ROWBUF_BYTES   = 16384,
  parameter int ROWBUF_Y_BYTES = 0,
  parameter int ROWBUF_C_BYTES = 0,
  parameter bit FANCY_UPSAMPLE = 1'b0,
  parameter bit CC_TURBO       = 1'b0,
  parameter bit RGB_OUT        = 1'b1,
  parameter bit CHECKS         = 1'b1,
  parameter int NPIX           = (FAST == 2) ? 4 : 1
) (
  input  logic        clk,
  input  logic        rst,
  // JPEG byte stream
  input  logic        in_valid,
  input  logic [7:0]  in_data,
  input  logic        in_last,       // marks the last byte of a file (tie to 0 if unknown; see below)
  output logic        in_ready,
  // output format for the next frame (FMT_RGB / FMT_YCBCR / FMT_Y)
  input  logic [1:0]  out_fmt,
  // decoded pixels: NPIX per beat, pixel i in bits 8i+7..8i at (px_x + i, px_y), i < px_n
  output logic        px_valid,
  input  logic        px_ready,
  output logic [15:0] px_x,
  output logic [15:0] px_y,
  output logic [2:0]  px_n,          // pixels in the beat (1..NPIX; fewer than NPIX only at the right edge)
  output logic [8*NPIX-1:0] px_c0,   // R | Y
  output logic [8*NPIX-1:0] px_c1,   // G | Cb   (128 for FMT_Y)
  output logic [8*NPIX-1:0] px_c2,   // B | Cr   (128 for FMT_Y)
  output logic        px_sof,        // first pixel of the frame (x = 0, y = 0)
  output logic        px_eol,        // the beat holds the last pixel of an image row (x = width-1)
  // status
  output logic [15:0] img_w,
  output logic [15:0] img_h,
  output logic        frame_start,   // pulse: header parsed, img_w/img_h valid, pixels follow
  output logic        frame_done,    // pulse: last pixel has been accepted
  output logic [12:0] err            // errors of the current image (see jpeg_pkg)
);
  generate
    // unsupported combinations stop the elaboration with a readable name
    if (NPIX != ((FAST == 2) ? 4 : 1)) begin : g_bad_npix
      jpeg_decoder_NPIX_must_be_4_for_FAST_2_and_1_otherwise u_error ();
    end
    if (FAST == 2 && RASTER_OUT) begin : g_bad_raster
      jpeg_decoder_FAST_2_supports_MCU_order_only u_error ();
    end
    if (FAST == 2) begin : g_wide
      jpeg_dec_wide #(.CC_TURBO(CC_TURBO), .RGB_OUT(RGB_OUT), .CHECKS(CHECKS)) u_core (
        .clk(clk), .rst(rst), .in_valid(in_valid), .in_data(in_data), .in_last(in_last), .in_ready(in_ready), .out_fmt(out_fmt),
        .px_valid(px_valid), .px_ready(px_ready), .px_x(px_x), .px_y(px_y), .px_n(px_n),
        .px_c0(px_c0), .px_c1(px_c1), .px_c2(px_c2), .px_sof(px_sof), .px_eol(px_eol),
        .img_w(img_w), .img_h(img_h), .frame_start(frame_start), .frame_done(frame_done), .err(err));
    end else if (FAST == 1) begin : g_fast
      jpeg_dec_fast #(.RASTER_OUT(RASTER_OUT), .ROWBUF_BYTES(ROWBUF_BYTES), .ROWBUF_Y_BYTES(ROWBUF_Y_BYTES),
                      .ROWBUF_C_BYTES(ROWBUF_C_BYTES), .FANCY_UPSAMPLE(FANCY_UPSAMPLE),
                      .CC_TURBO(CC_TURBO), .RGB_OUT(RGB_OUT), .CHECKS(CHECKS)) u_core (
        .clk(clk), .rst(rst), .in_valid(in_valid), .in_data(in_data), .in_last(in_last), .in_ready(in_ready), .out_fmt(out_fmt),
        .px_valid(px_valid), .px_ready(px_ready), .px_x(px_x), .px_y(px_y),
        .px_c0(px_c0), .px_c1(px_c1), .px_c2(px_c2), .px_sof(px_sof), .px_eol(px_eol),
        .img_w(img_w), .img_h(img_h), .frame_start(frame_start), .frame_done(frame_done), .err(err));
    end else begin : g_small
      jpeg_dec_small #(.RASTER_OUT(RASTER_OUT), .ROWBUF_BYTES(ROWBUF_BYTES), .FANCY_UPSAMPLE(FANCY_UPSAMPLE),
                       .CC_TURBO(CC_TURBO), .RGB_OUT(RGB_OUT), .CHECKS(CHECKS)) u_core (
        .clk(clk), .rst(rst), .in_valid(in_valid), .in_data(in_data), .in_last(in_last), .in_ready(in_ready), .out_fmt(out_fmt),
        .px_valid(px_valid), .px_ready(px_ready), .px_x(px_x), .px_y(px_y),
        .px_c0(px_c0), .px_c1(px_c1), .px_c2(px_c2), .px_sof(px_sof), .px_eol(px_eol),
        .img_w(img_w), .img_h(img_h), .frame_start(frame_start), .frame_done(frame_done), .err(err));
    end
    if (FAST != 2) begin : g_npix1
      assign px_n = 3'd1;
    end
  endgenerate
endmodule
