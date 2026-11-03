// jpeg_pkg.sv - shared constants / helpers for the synthesizable baseline JPEG decoder.
//
// All modules in this directory are written for the Quartus II 13.0sp1 SystemVerilog-2005
// subset (Cyclone II EP2C5T144C8) and are verified with Verilator against libjpeg (djpeg).
// References are to ITU-T T.81 (the "official JPEG guide").
package jpeg_pkg;

  // T.81 Figure A.6: zig-zag position k (0..63) -> natural row-major index.
  function automatic logic [5:0] zigzag_to_natural(input logic [5:0] k);
    case (k)
      6'd0:  zigzag_to_natural = 6'd0;   6'd1:  zigzag_to_natural = 6'd1;   6'd2:  zigzag_to_natural = 6'd8;   6'd3:  zigzag_to_natural = 6'd16;
      6'd4:  zigzag_to_natural = 6'd9;   6'd5:  zigzag_to_natural = 6'd2;   6'd6:  zigzag_to_natural = 6'd3;   6'd7:  zigzag_to_natural = 6'd10;
      6'd8:  zigzag_to_natural = 6'd17;  6'd9:  zigzag_to_natural = 6'd24;  6'd10: zigzag_to_natural = 6'd32;  6'd11: zigzag_to_natural = 6'd25;
      6'd12: zigzag_to_natural = 6'd18;  6'd13: zigzag_to_natural = 6'd11;  6'd14: zigzag_to_natural = 6'd4;   6'd15: zigzag_to_natural = 6'd5;
      6'd16: zigzag_to_natural = 6'd12;  6'd17: zigzag_to_natural = 6'd19;  6'd18: zigzag_to_natural = 6'd26;  6'd19: zigzag_to_natural = 6'd33;
      6'd20: zigzag_to_natural = 6'd40;  6'd21: zigzag_to_natural = 6'd48;  6'd22: zigzag_to_natural = 6'd41;  6'd23: zigzag_to_natural = 6'd34;
      6'd24: zigzag_to_natural = 6'd27;  6'd25: zigzag_to_natural = 6'd20;  6'd26: zigzag_to_natural = 6'd13;  6'd27: zigzag_to_natural = 6'd6;
      6'd28: zigzag_to_natural = 6'd7;   6'd29: zigzag_to_natural = 6'd14;  6'd30: zigzag_to_natural = 6'd21;  6'd31: zigzag_to_natural = 6'd28;
      6'd32: zigzag_to_natural = 6'd35;  6'd33: zigzag_to_natural = 6'd42;  6'd34: zigzag_to_natural = 6'd49;  6'd35: zigzag_to_natural = 6'd56;
      6'd36: zigzag_to_natural = 6'd57;  6'd37: zigzag_to_natural = 6'd50;  6'd38: zigzag_to_natural = 6'd43;  6'd39: zigzag_to_natural = 6'd36;
      6'd40: zigzag_to_natural = 6'd29;  6'd41: zigzag_to_natural = 6'd22;  6'd42: zigzag_to_natural = 6'd15;  6'd43: zigzag_to_natural = 6'd23;
      6'd44: zigzag_to_natural = 6'd30;  6'd45: zigzag_to_natural = 6'd37;  6'd46: zigzag_to_natural = 6'd44;  6'd47: zigzag_to_natural = 6'd51;
      6'd48: zigzag_to_natural = 6'd58;  6'd49: zigzag_to_natural = 6'd59;  6'd50: zigzag_to_natural = 6'd52;  6'd51: zigzag_to_natural = 6'd45;
      6'd52: zigzag_to_natural = 6'd38;  6'd53: zigzag_to_natural = 6'd31;  6'd54: zigzag_to_natural = 6'd39;  6'd55: zigzag_to_natural = 6'd46;
      6'd56: zigzag_to_natural = 6'd53;  6'd57: zigzag_to_natural = 6'd60;  6'd58: zigzag_to_natural = 6'd61;  6'd59: zigzag_to_natural = 6'd54;
      6'd60: zigzag_to_natural = 6'd47;  6'd61: zigzag_to_natural = 6'd55;  6'd62: zigzag_to_natural = 6'd62;  default: zigzag_to_natural = 6'd63;
    endcase
  endfunction

  // Entropy-byte stream between parser and bit reader: 9-bit tokens.
  // bit 8 = 0 : ordinary (already un-stuffed) data byte in bits 7:0
  // bit 8 = 1 : a marker was met inside the scan; bits 7:0 hold the marker code
  //             (0xD0..0xD7 = RSTn, anything else ends the scan, normally 0xD9 EOI)
  localparam int TOK_W = 9;

  // Error flags of the current image (jpeg_decoder.err): cleared when the next image's SOI is
  // accepted, and by rst.  An image with any header error (ERR_SOF_TYPE .. ERR_SCAN, ERR_FRAME,
  // ERR_TRUNC) produces no pixels; its frame still ends with frame_done.  Pixels of a frame
  // whose err is not zero at frame_done must be discarded by the consumer.
  localparam int ERR_SOF_TYPE   = 0;  // not a baseline SOF0 frame (progressive, lossless, ...)
  localparam int ERR_PRECISION  = 1;  // sample precision != 8
  localparam int ERR_DQT        = 2;  // DQT invalid (B.2.4.1): Pq != 0, Tq > 3, a zero Qk, segment
                                      // too short, or the scan uses a table that was never (fully) defined
  localparam int ERR_DHT        = 3;  // DHT invalid (B.2.4.2, Annex C): Tc/Th > 1, too many codes,
                                      // over-subscribed code tree or all-ones code, DC symbol > 11,
                                      // segment too short, or the scan uses an undefined table
  localparam int ERR_NCOMP      = 4;  // Nf not 1 or 3
  localparam int ERR_SAMPLING   = 5;  // Hi or Vi not in {1,2}
  localparam int ERR_SCAN       = 6;  // scan is not a single interleaved baseline scan (Ns != Nf,
                                      // Ss/Se/AhAl, unknown or repeated Csj, wrong SOS length)
  localparam int ERR_HUFF       = 7;  // no Huffman code matched within 16 bits (corrupt data)
  localparam int ERR_MARKER     = 8;  // bits requested past a marker, unexpected marker in scan,
                                      // or RSTn out of sequence / without a restart interval
  localparam int ERR_SYNC       = 9;  // lost marker sync inside an image (bytes before an SOI are skipped)
  localparam int ERR_WIDTH      = 10; // raster mode: image too wide for the row buffer (no pixels are produced)
  localparam int ERR_FRAME      = 11; // SOF0 invalid (B.2.2): width or height 0 (DNL is not supported),
                                      // wrong length, repeated Ci, Tqi > 3, or no SOF before the SOS
  localparam int ERR_TRUNC      = 12; // in_last arrived before EOI: the file is truncated
  localparam int ERR_BITS       = 13;
  // earlier names
  localparam int ERR_DQT_16BIT  = ERR_DQT;
  localparam int ERR_DHT_ID     = ERR_DHT;

  // Output formats (jpeg_decoder.out_fmt, sampled at the start of each frame)
  //   FMT_RGB  : c0,c1,c2 = R,G,B          (grey images: Y,Y,Y)
  //   FMT_YCBCR: c0,c1,c2 = Y,Cb,Cr after upsampling, colour conversion skipped (grey: Y,128,128)
  //   FMT_Y    : c0 = Y (luma only), c1 = c2 = 128; chroma blocks are entropy-decoded but not transformed
  localparam logic [1:0] FMT_RGB   = 2'd0;
  localparam logic [1:0] FMT_YCBCR = 2'd1;
  localparam logic [1:0] FMT_Y     = 2'd2;

  // Commands from jpeg_decoder to jpeg_raster (raster output mode)
  localparam logic [1:0] CMD_LINES = 2'd0;  // emit lines 0..nlines-1 of the MCU row in the row buffer
  localparam logic [1:0] CMD_DEFER = 2'd1;  // emit the deferred last line of the previous MCU row
  localparam logic [1:0] CMD_COPY  = 2'd2;  // copy the last plane row of each component to the line buffer
endpackage
