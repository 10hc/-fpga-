// jpeg_ycc2rgb.sv - YCbCr -> RGB exactly like libjpeg's ycc_rgb_convert (jdcolor.c, SCALEBITS=16):
//   R = Y + DESCALE(FIX(1.402) * (Cr-128))
//   G = Y + ((-FIX(k_cb) * (Cb-128) + ONE_HALF - FIX(0.714...) * (Cr-128)) >> 16)
//   B = Y + DESCALE(FIX(1.772) * (Cb-128))           each clamped to 0..255
// CC_TURBO selects the Cb->G constant: libjpeg 9 uses FIX(0.344136286) = 22553, libjpeg 6b /
// libjpeg-turbo (Pillow, OpenCV) use FIX(0.34414) = 22554.  The other constants are identical.
// Purely combinational; the caller registers inputs and outputs.
// Based in part on the work of the Independent JPEG Group: the arithmetic reproduces libjpeg /
// libjpeg-turbo so that the output is bit-identical to them (see NOTICE.md).
module jpeg_ycc2rgb #(
  parameter bit CC_TURBO = 1'b0
) (
  input  logic [7:0] y,
  input  logic [7:0] cb,
  input  logic [7:0] cr,
  output logic [7:0] r,
  output logic [7:0] g,
  output logic [7:0] b
);
  localparam logic signed [17:0] K_R_CR = 18'sd91881;                        // FIX(1.402)
  localparam logic signed [17:0] K_B_CB = 18'sd116130;                       // FIX(1.772)
  localparam logic signed [17:0] K_G_CB = CC_TURBO ? -18'sd22554 : -18'sd22553; // -FIX(0.34414) / -FIX(0.344136286)
  localparam logic signed [17:0] K_G_CR = -18'sd46802;                       // -FIX(0.71414..)
  logic signed [8:0]  xcb, xcr;
  (* multstyle = "logic" *) logic signed [26:0] m_rr, m_bb, m_gb, m_gr;   // constant multipliers in LEs: the DSP blocks go to the IDCT
  logic signed [26:0] s_r, s_b, s_g;
  logic signed [9:0]  r_raw, g_raw, b_raw;
  assign xcb  = $signed({1'b0, cb}) - 9'sd128;
  assign xcr  = $signed({1'b0, cr}) - 9'sd128;
  assign m_rr = xcr * K_R_CR;
  assign m_bb = xcb * K_B_CB;
  assign m_gb = xcb * K_G_CB;
  assign m_gr = xcr * K_G_CR;
  assign s_r  = (m_rr + 27'sd32768) >>> 16;
  assign s_b  = (m_bb + 27'sd32768) >>> 16;
  assign s_g  = (m_gb + m_gr + 27'sd32768) >>> 16;
  assign r_raw = $signed({2'b00, y}) + s_r[9:0];
  assign g_raw = $signed({2'b00, y}) + s_g[9:0];
  assign b_raw = $signed({2'b00, y}) + s_b[9:0];

  function automatic logic [7:0] clamp8(input logic signed [9:0] v);
    if (v < 10'sd0)        clamp8 = 8'd0;
    else if (v > 10'sd255) clamp8 = 8'd255;
    else                   clamp8 = v[7:0];
  endfunction
  assign r = clamp8(r_raw);
  assign g = clamp8(g_raw);
  assign b = clamp8(b_raw);
endmodule
