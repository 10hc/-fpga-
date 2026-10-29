// -----------------------------------------------------------------------------
// bmp_scale.v - nearest neighbour picture scaler for a fixed 640x480 raster
//
//   Source stream : src_w * src_h pixels, row major, one pixel per clock,
//                   RGB888.  The module does not care about the vertical
//                   orientation: the first row received is the first target
//                   row.  No "frame start" pulse and no "input ready" signal
//                   are needed - src_w/src_h alone describe a frame.
//   Output stream : exactly 640*480 pixels, row major, on the fixed raster,
//                   with an out_ready handshake.  Aspect ratio is preserved,
//                   the picture is centred and the rest is filled with black.
//
//   Geometry (no multiplier and no divider, shifts/adds only):
//     The scale factor is the same on both axes and is expressed as a ratio
//         S / D          with   point(c) = A0 + c*S   (c = target coordinate)
//       width  limited (640*src_h <= 480*src_w) : S = src_w, D = 640
//       height limited (otherwise)              : S = src_h, D = 480
//     A target coordinate c belongs to the picture when
//         0 <= A0 + c*S < src_axis_length * D
//     A0 is 0 on the axis that fills the raster and
//     +-(320*src_h - 240*src_w) on the centred axis.  This symmetric form is
//     exactly the continuous "fit and centre" mapping, evaluated at integer
//     target coordinates, so the whole thing needs no division at all.
//     Inside the picture, the i-th (i = 0,1,2,...) content pixel samples the
//     source index floor(i*S/D), which is tracked with an incremental
//     Bresenham accumulator (0/1 step when S <= D, pre-skipped when S > D).
//
//   Implementation:
//     * The horizontal mapping is applied while the source row is written:
//       only the sampled columns are stored, so a stored row is never wider
//       than 640 pixels.  The line buffer is declared 2048 x 24 as requested
//       and is used as three slots of 640 pixels (an elastic row FIFO).
//     * The vertical mapping replays one stored row for every target row it
//       covers, so an upscaled source row is emitted several times without
//       touching the input stream again.
//     * out_de is only asserted together with out_ready; a pixel that is not
//       accepted is held and presented again (never dropped).
// -----------------------------------------------------------------------------
`timescale 1ns / 1ps

module bmp_scale (
    input  wire        clk,          // 100 MHz, single clock domain
    input  wire        rst,          // synchronous, active high
    input  wire [15:0] src_w,        // source width  1..4096
    input  wire [15:0] src_h,        // source height 1..4096
    input  wire        in_de,        // 1 = in_rgb is a valid source pixel
    input  wire [23:0] in_rgb,       // source pixel, RGB888
    input  wire        out_ready,    // downstream can accept a pixel
    output reg         out_de,       // 1 = out_rgb is a valid output pixel
    output reg  [23:0] out_rgb,      // output pixel on the 640x480 raster
    output reg         frame_done    // 1-clock pulse after the last pixel
);

    // ------------------------------------------------------------------
    // raster / buffer constants
    // ------------------------------------------------------------------
    localparam integer OUT_W   = 640;
    localparam integer OUT_H   = 480;
    localparam integer OUT_PIX = OUT_W * OUT_H;      // 307200
    localparam integer SLOT_W  = 640;                // stored samples per row

    localparam [1:0] ST_EMIT = 2'd0,                 // emitting output pixels
                     ST_WAIT = 2'd1;                 // waiting for a source row

    // ------------------------------------------------------------------
    // slot ring: three line-buffer slots at 0 / 640 / 1280
    // ------------------------------------------------------------------
    function [10:0] base_next;
        input [10:0] b;
        begin
            base_next = (b == 11'd0) ? 11'd640 :
                        (b == 11'd640) ? 11'd1280 : 11'd0;
        end
    endfunction

    // ------------------------------------------------------------------
    // scale geometry (combinational, from the source size)
    // ------------------------------------------------------------------
    wire [12:0] sw = src_w[12:0];
    wire [12:0] sh = src_h[12:0];

    wire [24:0] wx640 = ({12'd0, sw} << 10) - ({12'd0, sw} << 7);   // sw*640
    wire [24:0] hy480 = ({12'd0, sh} << 9)  - ({12'd0, sh} << 5);   // sh*480
    wire        width_lim = (wx640 >= hy480);        // 640*src_h <= 480*src_w

    wire [12:0] S  = width_lim ? sw : sh;            // scale numerator
    wire [9:0]  D  = width_lim ? 10'd640 : 10'd480;  // scale denominator
    wire        down = (S > D);                      // both axes downscale
    wire [12:0] Sd = S - D;                          // S-D, valid when down

    wire signed [24:0] S_s = {12'd0, S};             // signed helpers
    wire signed [24:0] D_s = {15'd0, D};

    // 320*src_h - 240*src_w  (centre offset numerator)
    wire signed [24:0] h320 = ({12'd0, sh} << 8) + ({12'd0, sh} << 6);
    wire signed [24:0] w240 = ({12'd0, sw} << 8) - ({12'd0, sw} << 4);
    wire signed [24:0] a0c  = h320 - w240;
    wire signed [24:0] a0x  = width_lim ? 25'sd0 : (25'sd0 - a0c);
    wire signed [24:0] a0y  = width_lim ? a0c : 25'sd0;

    // content limits (exclusive): src_axis_length * D
    wire [24:0] w480 = ({12'd0, sw} << 9) - ({12'd0, sw} << 5);     // sw*480
    wire [24:0] h640 = ({12'd0, sh} << 10) - ({12'd0, sh} << 7);    // sh*640
    wire [23:0] limx_u = width_lim ? wx640[23:0] : w480[23:0];
    wire [23:0] limy_u = width_lim ? h640[23:0] : hy480[23:0];
    wire signed [24:0] limx = {1'b0, limx_u};
    wire signed [24:0] limy = {1'b0, limy_u};

    // ------------------------------------------------------------------
    // line buffer: 2048 x 24, one write port + one registered read port
    // ------------------------------------------------------------------
    reg [23:0] linebuf [0:2047];

    reg [10:0] wr_base, rd_base;      // slot bases of the write / read side
    reg [1:0]  fifo_cnt;              // complete, not yet consumed rows (0..3)

    // ------------------------------------------------------------------
    // write side: sample the incoming row and store the sampled columns
    // ------------------------------------------------------------------
    reg [12:0] w_col;                 // source column index
    reg [12:0] w_row;                 // source row index inside the frame
    reg [12:0] w_hn;                  // horizontal sampling accumulator
    reg [12:0] w_vn;                  // vertical   sampling accumulator
    reg [10:0] w_ptr;                 // write offset inside the slot
    reg        w_store;               // current source row is being stored

    wire        col_keep = !down || (w_hn == 13'd0) || (w_hn > Sd);
    wire [12:0] hn_sum   = w_hn + {3'd0, D};
    wire [12:0] w_hn_nxt = (hn_sum >= S) ? (hn_sum - S) : hn_sum;

    wire        row_keep = !down || (w_vn == 13'd0) || (w_vn > Sd);
    wire [12:0] vn_sum   = w_vn + {3'd0, D};
    wire [12:0] w_vn_nxt = (vn_sum >= S) ? (vn_sum - S) : vn_sum;

    wire        src_ok   = (S != 13'd0) && (sw != 13'd0) && (sh != 13'd0);
    wire        last_col = (w_col == (sw - 13'd1));
    wire        last_row = (w_row == (sh - 13'd1));

    wire        we     = in_de && src_ok && w_store && col_keep;
    wire [10:0] w_addr = wr_base + w_ptr;

    wire        wr_row_end = in_de && src_ok && last_col;
    wire        wr_push    = wr_row_end && w_store;
    wire [1:0]  cnt_pushed = fifo_cnt + (wr_push ? 2'd1 : 2'd0);
    // sampling decision for the next source row (accumulator + FIFO room)
    wire [12:0] vn_use  = last_row ? 13'd0 : w_vn_nxt;
    wire        keep_nxt = (!down || (vn_use == 13'd0) || (vn_use > Sd))
                           && (cnt_pushed != 2'd3);

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            w_col   <= 13'd0;
            w_row   <= 13'd0;
            w_hn    <= 13'd0;
            w_vn    <= 13'd0;
            w_ptr   <= 11'd0;
            w_store <= 1'b1;                 // row 0 is always sampled
            wr_base <= 11'd0;
        end else if (in_de && src_ok) begin
            w_hn <= w_hn_nxt;
            if (we && (w_ptr != 11'd639))
                w_ptr <= w_ptr + 11'd1;

            if (last_col) begin
                w_col   <= 13'd0;
                w_hn    <= 13'd0;
                w_ptr   <= 11'd0;
                if (w_store)
                    wr_base <= base_next(wr_base);
                w_vn    <= vn_use;
                w_store <= keep_nxt;
                w_row   <= last_row ? 13'd0 : (w_row + 13'd1);
            end else begin
                w_col <= w_col + 13'd1;
            end
        end
    end

    // ------------------------------------------------------------------
    // read side
    // ------------------------------------------------------------------
    reg [1:0]  st;
    reg [9:0]  ox;                    // target column inside the row
    reg signed [24:0] bx;             // horizontal picture test accumulator
    reg [12:0] mxn;                   // horizontal Bresenham fraction
    reg [9:0]  mxi;                   // stored sample index (read offset)
    reg        rowdat;                // this target row is inside the picture

    reg signed [24:0] by;             // vertical picture test accumulator
    reg [12:0] myn;                   // vertical Bresenham fraction
    reg [9:0]  myi;                   // stored row index of this target row

    reg [18:0] px_cnt;                // accepted pixels of the current frame
    reg [15:0] wd_cnt;                // wait watchdog

    reg [23:0] rd_q;                  // presented pixel data
    reg        rd_in;                 // presented pixel is inside the picture
    reg        rd_val;                // presented pixel is valid

    wire        bx_inside = (bx >= 25'sd0) && (bx < limx);
    wire [13:0] mxn_sum   = {1'b0, mxn} + {1'b0, S};
    wire [13:0] myn_sum   = {1'b0, myn} + {1'b0, S};
    wire [10:0] rd_addr   = rd_base + mxi;

    wire        pipe_adv = (!rd_val) || out_ready;
    wire        px_acc   = rd_val && out_ready;

    // ---- next state ---------------------------------------------------
    reg [1:0]  st_n;
    reg [9:0]  ox_n;
    reg signed [24:0] bx_n, by_n;
    reg [12:0] mxn_n, myn_n;
    reg [9:0]  mxi_n, myi_n;
    reg        rowdat_n, rd_pop;
    reg [10:0] addr_n;
    reg        in_n, val_n;

    reg        restart;

    always @(*) begin
        // hold by default
        st_n = st;  ox_n = ox;  bx_n = bx;  mxn_n = mxn;  mxi_n = mxi;
        rowdat_n = rowdat;  by_n = by;  myn_n = myn;  myi_n = myi;
        rd_pop = 1'b0;
        restart = px_acc && (px_cnt == OUT_PIX-1);

        if (st == ST_EMIT) begin
            if (ox == 10'd639) begin
                // ---- last pixel of a target row: set up the next row ----
                ox_n  = 10'd0;
                bx_n  = a0x;
                mxn_n = 13'd0;
                mxi_n = 10'd0;
                by_n  = by + S_s;
                if (down) begin
                    // one stored row per target row (rows pre-skipped)
                    myn_n  = 13'd0;
                    myi_n  = myi + 10'd1;
                    rd_pop = rowdat;
                end else begin
                    // 0/1 Bresenham step: repeat a stored row when needed
                    if (myn_sum >= {4'd0, D}) begin
                        myn_n  = myn_sum[12:0] - {3'd0, D};
                        myi_n  = myi + 10'd1;
                        rd_pop = rowdat;
                    end else begin
                        myn_n  = myn_sum[12:0];
                    end
                end
                rowdat_n = (by_n >= 25'sd0) && (by_n < limy);
                st_n = (rowdat_n && (fifo_cnt == 2'd0) && (wd_cnt != 16'hFFFF))
                       ? ST_WAIT : ST_EMIT;
            end else begin
                // ---- mid row ----
                ox_n = ox + 10'd1;
                bx_n = bx + S_s;
                if (bx_inside && rowdat) begin
                    if (down) begin
                        mxi_n = mxi + 10'd1;
                    end else if (mxn_sum >= {4'd0, D}) begin
                        mxn_n = mxn_sum[12:0] - {3'd0, D};
                        mxi_n = mxi + 10'd1;
                    end else begin
                        mxn_n = mxn_sum[12:0];
                    end
                end
            end
        end else begin
            // ST_WAIT: start as soon as a complete row is available
            if (fifo_cnt != 2'd0)
                st_n = ST_EMIT;
            else if (wd_cnt == 16'hFFFF) begin
                rowdat_n = 1'b0;          // watchdog: black row, never hang
                st_n     = ST_EMIT;
            end
        end

        // frame restart overrides the row transition
        if (restart) begin
            ox_n     = 10'd0;
            bx_n     = a0x;
            mxn_n    = 13'd0;
            mxi_n    = 10'd0;
            by_n     = a0y;
            myn_n    = 13'd0;
            myi_n    = 10'd0;
            rowdat_n = (a0y >= 25'sd0) && (a0y < limy);
            st_n     = (rowdat_n && (fifo_cnt == 2'd0)) ? ST_WAIT : ST_EMIT;
            rd_pop   = 1'b0;
        end

        // address / flags of the pixel described by the next state
        addr_n = 11'd0;
        in_n   = 1'b0;
        val_n  = 1'b0;
        if (st_n == ST_EMIT) begin
            val_n = 1'b1;
            in_n  = rowdat_n && (bx_n >= 25'sd0) && (bx_n < limx);
            if (rowdat_n)
                addr_n = (rd_pop ? base_next(rd_base) : rd_base) + mxi_n;
        end
    end

    // ---- registers ----------------------------------------------------
    always @(posedge clk) begin
        // block RAM: one write port, one registered read port
        if (we)
            linebuf[w_addr] <= in_rgb;
        if (pipe_adv)
            rd_q <= linebuf[addr_n];
    end

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            st        <= ST_EMIT;
            ox        <= 10'd0;
            bx        <= 25'sd0;
            mxn       <= 13'd0;
            mxi       <= 10'd0;
            rowdat    <= 1'b0;
            by        <= 25'sd0;
            myn       <= 13'd0;
            myi       <= 10'd0;
            px_cnt    <= 19'd0;
            wd_cnt    <= 16'd0;
            // rd_q 是块 RAM 的读出寄存器，只由上面那个 always 块驱动（去掉这里的复位，避免多驱动）
            rd_in     <= 1'b0;
            rd_val    <= 1'b0;
            rd_base   <= 11'd0;
            fifo_cnt  <= 2'd0;
            frame_done <= 1'b0;
        end else begin
            frame_done <= 1'b0;

            if (pipe_adv) begin
                st     <= st_n;
                ox     <= ox_n;
                bx     <= bx_n;
                mxn    <= mxn_n;
                mxi    <= mxi_n;
                rowdat <= rowdat_n;
                by     <= by_n;
                myn    <= myn_n;
                myi    <= myi_n;
                rd_in  <= in_n;
                rd_val <= val_n;
            end

            if (pipe_adv && rd_pop)
                rd_base <= base_next(rd_base);

            // elastic row FIFO: write side pushes, read side pops
            fifo_cnt <= fifo_cnt + (wr_push ? 2'd1 : 2'd0)
                                 - ((pipe_adv && rd_pop) ? 2'd1 : 2'd0);

            if (px_acc) begin
                if (px_cnt == OUT_PIX-1) begin
                    frame_done <= 1'b1;
                    px_cnt     <= 19'd0;
                end else begin
                    px_cnt <= px_cnt + 19'd1;
                end
            end

            if (st == ST_WAIT)
                wd_cnt <= wd_cnt + 16'd1;
            else
                wd_cnt <= 16'd0;
        end
    end

    // ------------------------------------------------------------------
    // output registers: assert out_de only when out_ready is high
    // ------------------------------------------------------------------
    always @(*) begin
        out_de  = rd_val && out_ready;
        out_rgb = (rd_val && rd_in) ? rd_q : 24'h000000;
    end

endmodule