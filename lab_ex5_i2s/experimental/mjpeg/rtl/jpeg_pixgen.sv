// jpeg_pixgen.sv - MCU-order output stage (RASTER_OUT = 0): reads the decoded samples of one MCU
// back in raster order *within the MCU*, upsamples chroma by sample replication (T.81 A.1.1,
// libjpeg -nosmooth h2v1/h2v2_upsample) and formats the pixel as RGB (jdcolor.c
// ycc_rgb_convert), YCbCr (no colour conversion) or Y only.
//
// Pixels carry their absolute image coordinates; pixels of the MCU that fall outside the image
// (right/bottom padding, T.81 A.2.4) are dropped.  Output is a valid/ready stream so a slow sink
// (UART, framebuffer) can apply backpressure.
module jpeg_pixgen #(
  parameter bit CC_TURBO = 1'b0,
  parameter bit RGB_OUT  = 1'b1       // 0: no colour converter, FMT_RGB behaves like FMT_YCBCR
) (
  input  logic        clk,
  input  logic        rst,
  input  logic        start,          // pulse: MCU buffer complete
  output logic        done,           // pulse: all pixels of this MCU emitted
  // geometry (stable while busy)
  input  logic [15:0] img_w,
  input  logic [15:0] img_h,
  input  logic [15:0] mcu_x0,         // pixel origin of the MCU
  input  logic [15:0] mcu_y0,
  input  logic        gray,           // 1 = single component
  input  logic [1:0]  fmt,            // FMT_RGB / FMT_YCBCR / FMT_Y
  input  logic        hmax2,          // Hmax == 2  (else 1)
  input  logic        vmax2,          // Vmax == 2
  input  logic [2:0]  h2,             // per component c: Hc == 2
  input  logic [2:0]  v2,             // per component c: Vc == 2
  input  logic [11:0] blk_base,       // per component c: first block slot (4 bits each)
  // MCU sample buffer read port (1-cycle latency), addr = {slot[3:0], row[2:0], col[2:0]}
  output logic [9:0]  buf_raddr,
  input  logic [7:0]  buf_rdata,
  // pixel stream
  output logic        px_valid,
  input  logic        px_ready,
  output logic [15:0] px_x,
  output logic [15:0] px_y,
  output logic [7:0]  px_c0,
  output logic [7:0]  px_c1,
  output logic [7:0]  px_c2,
  output logic        px_sof,         // first pixel of the frame (x = 0, y = 0)
  output logic        px_eol          // last pixel of an image row (x = width-1)
);
  import jpeg_pkg::*;

  typedef enum logic [2:0] { IDLE, RD_Y, RD_CB, RD_CR, LAST, EMIT } state_t;
  state_t state;

  logic [3:0] x, y;                   // position inside the MCU (0..15)
  logic [3:0] xmax, ymax;
  assign xmax = hmax2 ? 4'd15 : 4'd7;
  assign ymax = vmax2 ? 4'd15 : 4'd7;

  logic luma_only;                    // grey image or FMT_Y: component 0 only
  assign luma_only = gray | (fmt == FMT_Y);

  // sample address of component c for MCU position (x, y): replicate when the component
  // is subsampled relative to Hmax/Vmax (xc = x >> (log2 Hmax - log2 Hc)).
  function automatic logic [9:0] samp_addr(input logic [3:0] xx, input logic [3:0] yy,
                                           input logic hh2, input logic vv2, input logic [3:0] base);
    logic [3:0] xc, yc, slot;
    xc = (hmax2 & ~hh2) ? {1'b0, xx[3:1]} : xx;
    yc = (vmax2 & ~vv2) ? {1'b0, yy[3:1]} : yy;
    slot = base + (yc[3] ? (hh2 ? 4'd2 : 4'd1) : 4'd0) + {3'd0, xc[3]};
    samp_addr = {slot, yc[2:0], xc[2:0]};
  endfunction

  logic [7:0]  sy, scb, scr;
  logic [15:0] ax, ay;                // absolute coordinates of the current pixel
  logic        in_image;
  assign ax = mcu_x0 + {12'd0, x};
  assign ay = mcu_y0 + {12'd0, y};
  assign in_image = (ax < img_w) && (ay < img_h);

  always_comb begin
    case (state)
      RD_Y:    buf_raddr = samp_addr(x, y, h2[0], v2[0], blk_base[3:0]);
      RD_CB:   buf_raddr = samp_addr(x, y, h2[1], v2[1], blk_base[7:4]);
      default: buf_raddr = samp_addr(x, y, h2[2], v2[2], blk_base[11:8]);
    endcase
  end

  logic [7:0] cr_r, cr_g, cr_b;
  jpeg_ycc2rgb #(.CC_TURBO(CC_TURBO)) u_cc (.y(sy), .cb(scb), .cr(scr), .r(cr_r), .g(cr_g), .b(cr_b));

  logic last_px;
  assign last_px = (x == xmax) && (y == ymax);

  always_ff @(posedge clk) begin
    done <= 1'b0;
    if (rst) begin
      state <= IDLE; x <= '0; y <= '0; px_valid <= 1'b0; sy <= '0; scb <= '0; scr <= '0;
      px_x <= '0; px_y <= '0; px_c0 <= '0; px_c1 <= '0; px_c2 <= '0; px_sof <= 1'b0; px_eol <= 1'b0;
    end else begin
      if (px_valid && px_ready) px_valid <= 1'b0;
      case (state)
        IDLE: if (start) begin x <= '0; y <= '0; state <= RD_Y; end
        RD_Y: begin                                  // Y address presented
          if (in_image) state <= luma_only ? LAST : RD_CB;
          else          state <= EMIT;               // outside the image: skip
        end
        RD_CB: begin sy <= buf_rdata; state <= RD_CR; end          // Y arrives, Cb addressed
        RD_CR: begin scb <= buf_rdata; state <= LAST; end          // Cb arrives, Cr addressed
        LAST: begin                                                // Cr (or Y when luma only) arrives
          if (luma_only) begin sy <= buf_rdata; scb <= 8'd128; scr <= 8'd128; end
          else scr <= buf_rdata;
          state <= EMIT;
        end
        EMIT: begin
          if (!px_valid || px_ready) begin           // output register free
            if (in_image) begin
              px_valid <= 1'b1;
              px_x <= ax; px_y <= ay;
              px_sof <= (ax == 16'd0) && (ay == 16'd0);
              px_eol <= (ax == img_w - 16'd1);
              if (RGB_OUT && fmt == FMT_RGB && !gray) begin px_c0 <= cr_r; px_c1 <= cr_g; px_c2 <= cr_b; end
              else if (RGB_OUT && fmt == FMT_RGB) begin px_c0 <= sy; px_c1 <= sy;  px_c2 <= sy;   end
              else                         begin px_c0 <= sy;   px_c1 <= scb;  px_c2 <= scr;  end
            end
            if (last_px) begin state <= IDLE; done <= 1'b1; x <= '0; y <= '0; end
            else begin
              if (x == xmax) begin x <= '0; y <= y + 4'd1; end
              else x <= x + 4'd1;
              state <= RD_Y;
            end
          end
        end
        default: state <= IDLE;
      endcase
    end
  end
endmodule
