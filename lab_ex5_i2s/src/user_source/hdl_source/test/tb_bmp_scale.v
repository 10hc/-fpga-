// -----------------------------------------------------------------------------
// tb_bmp_scale.v - self checking testbench for bmp_scale
//
//   Three source images (plus a degenerate one) are pushed through the scaler
//   one pixel per clock, the downstream ready is pseudo random (low about one
//   clock in five) and every accepted output pixel is compared against the
//   nearest neighbour expectation computed here from the same "fit and centre"
//   mapping.
//
//   The source is paced so that at most two source rows are in flight: the DUT
//   only owns a three row line buffer (and has no input ready output), so a
//   source that runs further ahead could not be absorbed.  A real source (an
//   SD card BMP reader driven by the video pipeline) behaves the same way.
// -----------------------------------------------------------------------------
`timescale 1ns / 1ps

module tb_bmp_scale;

    // ---------------- clock / reset ----------------
    reg clk;
    initial clk = 1'b0;
    always #5 clk = ~clk;                        // 100 MHz

    reg         rst;
    reg  [15:0] src_w, src_h;
    reg         in_de;
    reg  [23:0] in_rgb;
    reg         out_ready;

    wire        out_de;
    wire [23:0] out_rgb;
    wire        frame_done;

    bmp_scale dut (
        .clk        (clk),
        .rst        (rst),
        .src_w      (src_w),
        .src_h      (src_h),
        .in_de      (in_de),
        .in_rgb     (in_rgb),
        .out_ready  (out_ready),
        .out_de     (out_de),
        .out_rgb    (out_rgb),
        .frame_done (frame_done)
    );

    // ---------------- downstream ready: pseudo random ----------------
    reg [15:0] lfsr;
    initial begin
        lfsr      = 16'hACE1;
        out_ready = 1'b1;
    end
    always @(posedge clk) begin
        lfsr      <= {lfsr[14:0], lfsr[15] ^ lfsr[13] ^ lfsr[12] ^ lfsr[10]};
        out_ready <= ((lfsr % 16'd5) != 16'd0);   // low 1 clock in 5 on average
    end

    // =====================================================================
    // test images
    // =====================================================================
    function [23:0] src_pix;
        input integer sel;
        input integer x;
        input integer y;
        integer t;
        begin
            case (sel)
            0: begin  // 800x600 : red/green gradient + 40x40 blue tag at (100,100)
                if ((x >= 100) && (x < 140) && (y >= 100) && (y < 140))
                    src_pix = 24'h0000ff;
                else begin
                    t = (x * 255) / 799;
                    src_pix = {t[7:0], (255 - t), 8'h00};
                end
            end
            1: begin  // 320x240 : red / green / blue / white quadrants
                if (y < 120)
                    src_pix = (x < 160) ? 24'hff0000 : 24'h00ff00;
                else
                    src_pix = (x < 160) ? 24'h0000ff : 24'hffffff;
            end
            2: begin  // 1024x600 : vertical stripes, 16 px black / white
                src_pix = ((x / 16) % 2 == 0) ? 24'hffffff : 24'h000000;
            end
            3: begin  // 1920x1080 : 32x32 checker board, two loud colours
                src_pix = (((x / 32) + (y / 32)) % 2 == 0) ? 24'hff8000 : 24'h0040ff;
            end
            default: src_pix = 24'h000000;
            endcase
        end
    endfunction

    // =====================================================================
    // geometry model (same "fit and centre" mapping as the DUT)
    // =====================================================================
    integer g_w, g_h, g_sel;                     // current image
    integer S, Dp, is_down;                      // scale ratio
    integer a0x, a0y, limx, limy;                // boundary accumulators
    integer colin [0:639];                       // target column is inside
    integer colsrc[0:639];                       // source column used
    integer rowin [0:479];                       // target row is inside
    integer rowsrc[0:479];                       // stored row used
    integer keeprow[0:4095];                     // source row is stored
    integer ncols, nrows, nstored;
    integer x0p, y0p, wrect, hrect;

    task setup_geometry;
        integer c, r, k;
        begin
            if ((640 * g_h) <= (480 * g_w)) begin
                S  = g_w;
                Dp = 640;
            end else begin
                S  = g_h;
                Dp = 480;
            end
            is_down = (S > Dp);

            if ((640 * g_h) <= (480 * g_w)) begin
                a0x = 0;
                a0y = 320 * g_h - 240 * g_w;
            end else begin
                a0x = -(320 * g_h - 240 * g_w);
                a0y = 0;
            end
            limx = g_w * Dp;
            limy = g_h * Dp;

            k = 0;
            for (c = 0; c < 640; c = c + 1) begin
                if (((a0x + c * S) >= 0) && ((a0x + c * S) < limx)) begin
                    colin[c]  = 1;
                    colsrc[c] = (k * S) / Dp;
                    k = k + 1;
                end else begin
                    colin[c]  = 0;
                    colsrc[c] = 0;
                end
            end
            ncols = k;

            k = 0;
            for (r = 0; r < 480; r = r + 1) begin
                if (((a0y + r * S) >= 0) && ((a0y + r * S) < limy)) begin
                    rowin[r]  = 1;
                    rowsrc[r] = (k * S) / Dp;
                    k = k + 1;
                end else begin
                    rowin[r]  = 0;
                    rowsrc[r] = 0;
                end
            end
            nrows = k;
            nstored = (nrows == 0) ? 0 : (((nrows - 1) * S) / Dp) + 1;

            // source rows that carry a stored row (pacing model)
            for (r = 0; r < 4096; r = r + 1)
                keeprow[r] = 0;
            if (is_down == 0) begin
                for (r = 0; r < g_h; r = r + 1)
                    keeprow[r] = 1;
            end else begin
                for (k = 0; k < nrows; k = k + 1)
                    if (((k * S) / Dp) < 4096)
                        keeprow[(k * S) / Dp] = 1;
            end

            // report rectangle
            x0p = 640; wrect = 0; y0p = 480; hrect = 0;
            for (c = 0; c < 640; c = c + 1)
                if (colin[c]) begin
                    if (c < x0p) x0p = c;
                    wrect = wrect + 1;
                end
            for (r = 0; r < 480; r = r + 1)
                if (rowin[r]) begin
                    if (r < y0p) y0p = r;
                    hrect = hrect + 1;
                end
            if (wrect == 0) begin x0p = 0; y0p = 0; end
        end
    endtask

    // =====================================================================
    // output checker
    // =====================================================================
    integer chk_pix;            // accepted pixels of the current frame
    integer frame_mism;         // mismatches of the current frame
    integer frame_border;       // border pixels that were not black
    integer frame_done_cnt;     // frame_done pulses of the current frame
    integer overrun;            // pixels beyond 307200 in one frame
    integer shown;              // mismatch lines printed for this frame
    reg     frm_seen;           // a frame just finished
    integer g_fail;             // global problem counter
    integer wd_fires;           // watchdog substitutions (black row inserted)
    integer cur_wait, max_wait; // consecutive wait cycles
    integer xx, yy;
    reg [23:0] exp_px;

    initial begin
        chk_pix        = 0;
        frame_mism     = 0;
        frame_border   = 0;
        frame_done_cnt = 0;
        overrun        = 0;
        shown          = 0;
        frm_seen       = 1'b0;
        g_fail         = 0;
        wd_fires       = 0;
        cur_wait       = 0;
        max_wait       = 0;
    end

    always @(posedge clk) begin
        if (rst) begin
            chk_pix        <= 0;
            frame_mism     <= 0;
            frame_border   <= 0;
            frame_done_cnt <= 0;
            overrun        <= 0;
            shown          <= 0;
            frm_seen       <= 1'b0;
        end else begin
            frm_seen <= 1'b0;

            if (frame_done) begin
                frame_done_cnt <= frame_done_cnt + 1;
                frm_seen       <= 1'b1;
                if (chk_pix != 307200) begin
                    $display("  ERROR: frame_done after %0d pixels (expected 307200)", chk_pix);
                    g_fail = g_fail + 1;
                end
            end

            if (out_de && out_ready) begin
                if (chk_pix >= 307200) begin
                    overrun = overrun + 1;
                    if (overrun == 1)
                        $display("  ERROR: more than 307200 pixels in one frame");
                end else begin
                    xx = chk_pix % 640;
                    yy = chk_pix / 640;
                    if (colin[xx] && rowin[yy])
                        exp_px = src_pix(g_sel, colsrc[xx], rowsrc[yy]);
                    else
                        exp_px = 24'h000000;

                    if (out_rgb !== exp_px) begin
                        frame_mism = frame_mism + 1;
                        if (shown < 8) begin
                            shown = shown + 1;
                            $display("    MISMATCH at (%0d,%0d): got %06h expected %06h",
                                     xx, yy, out_rgb, exp_px);
                        end
                    end
                    if (!(colin[xx] && rowin[yy]) && (out_rgb !== 24'h000000))
                        frame_border = frame_border + 1;

                    chk_pix <= chk_pix + 1;
                end
            end
        end
    end

    // structural guards
    always @(posedge clk) begin
        if (!rst) begin
            if (out_de && !out_ready)
                $display("  ERROR: out_de asserted while out_ready is low");
            if (dut.wr_push && (dut.fifo_cnt == 2'd3))
                $display("  ERROR: line buffer row FIFO overflow at %0t", $time);
            if (dut.st == 2'd1) begin
                cur_wait = cur_wait + 1;
                if (cur_wait > max_wait) max_wait = cur_wait;
            end else begin
                cur_wait = 0;
            end
            if ((dut.st == 2'd1) && (dut.wd_cnt == 16'hFFFF)) begin
                wd_fires = wd_fires + 1;
                if (wd_fires < 6)
                    $display("  WARNING: row watchdog t=%0t img=%0d fifo=%0d chk=%0d waited=%0d",
                             $time, g_sel, dut.fifo_cnt, chk_pix, max_wait);
            end
        end
    end

    // =====================================================================
    // source: feed one image, paced to two rows in flight
    // =====================================================================
    integer pushed_kept;

    function integer consumed_rows;
        input dummy;
        begin
            if (chk_pix >= 307200)       consumed_rows = nstored;
            else if (rowin[chk_pix/640]) consumed_rows = rowsrc[chk_pix/640];
            else                         consumed_rows = 0;
        end
    endfunction

    task feed_frame;
        integer r, c, g;
        begin
            for (r = 0; r < g_h; r = r + 1) begin
                if ((r < 4096) && keeprow[r]) begin
                    g = 0;
                    while (((pushed_kept - consumed_rows(0)) >= 2) && (g < 4000000)) begin
                        @(posedge clk);
                        g = g + 1;
                    end
                    if (g >= 4000000) $display("  ERROR: source pacing timed out");
                end
                for (c = 0; c < g_w; c = c + 1) begin
                    @(posedge clk);
                    in_de  <= 1'b1;
                    in_rgb <= src_pix(g_sel, c, r);
                end
                @(posedge clk);
                in_de <= 1'b0;
                if ((r < 4096) && keeprow[r])
                    pushed_kept = pushed_kept + 1;
            end
            @(posedge clk);
            in_de <= 1'b0;
        end
    endtask

    task wait_frame_done;
        integer g;
        begin
            g = 0;
            while (!frm_seen && (g < 200000000)) begin
                @(posedge clk);
                g = g + 1;
            end
            if (g >= 200000000) begin
                $display("  ERROR: no frame_done (timeout)");
                g_fail = g_fail + 1;
            end
        end
    endtask

    // =====================================================================
    // one test case
    // =====================================================================
    integer f_pix, f_mism, f_border, f_done;

    task run_frame;
        input integer w;
        input integer h;
        input integer sel;
        input integer do_rst;
        input integer idx;
        begin
            g_w   = w;
            g_h   = h;
            g_sel = sel;
            setup_geometry;

            if (do_rst) begin
                rst    = 1'b1;
                src_w  = w[15:0];
                src_h  = h[15:0];
                in_de  = 1'b0;
                in_rgb = 24'h000000;
                repeat (6) @(posedge clk);
                rst = 1'b0;
                @(posedge clk);
            end else begin
                // the DUT has just finished a frame: update the size inside the
                // frame gap window
                src_w = w[15:0];
                src_h = h[15:0];
            end

            pushed_kept = 0;
            feed_frame;
            wait_frame_done;

            f_pix    = chk_pix;
            f_mism   = frame_mism;
            f_border = frame_border;
            f_done   = frame_done_cnt;

            if ((f_pix == 307200) && (f_mism == 0) && (f_border == 0) &&
                (f_done == 1) && (overrun == 0)) begin
                $display("  image %0d : %0dx%-4d rect=(%0d,%0d,%0d,%0d) pixels=%0d border=%0d mismatches=%0d frame_done=%0d maxwait=%0d wdfires=%0d  PASS",
                         idx, w, h, x0p, y0p, wrect, hrect, f_pix, f_border, f_mism, f_done, max_wait, wd_fires);
            end else begin
                $display("  image %0d : %0dx%-4d rect=(%0d,%0d,%0d,%0d) pixels=%0d border=%0d mismatches=%0d frame_done=%0d maxwait=%0d wdfires=%0d  FAIL",
                         idx, w, h, x0p, y0p, wrect, hrect, f_pix, f_border, f_mism, f_done, max_wait, wd_fires);
                g_fail = g_fail + 1;
            end

            // restart the per frame counters
            frame_mism     = 0;
            frame_border   = 0;
            frame_done_cnt = 0;
            overrun        = 0;
            shown          = 0;
            chk_pix        = 0;
            max_wait       = 0;
            cur_wait       = 0;
        end
    endtask

    // =====================================================================
    // main
    // =====================================================================
    initial begin
        rst    = 1'b1;
        src_w  = 16'd800;
        src_h  = 16'd600;
        in_de  = 1'b0;
        in_rgb = 24'h000000;

        $display("# ------------------------------------------------------------");
        $display("# bmp_scale : nearest neighbour 640x480 scaler test");
        $display("# ------------------------------------------------------------");

        //                 w     h    image  reset  index
        run_frame(       800,  600,   0,     1,     0);   // downscale, 4:3
        run_frame(       320,  240,   1,     1,     1);   // upscale  x2
        run_frame(      1024,  600,   2,     1,     2);   // downscale, 16:9-ish
        run_frame(      1024,  600,   2,     0,     2);   // back to back frame
        run_frame(       800,  600,   0,     0,     3);   // size change, no reset
        run_frame(         0,    0,   0,     1,     4);   // degenerate source
        run_frame(      1920, 1080,   3,     1,     5);   // big downscale, 16:9

        $display("# ------------------------------------------------------------");
        if (g_fail == 0)
            $display("RESULT: PASS");
        else
            $display("RESULT: FAIL (%0d problem(s))", g_fail);
        $display("# ------------------------------------------------------------");
        $finish;
    end

    // global watchdog
    initial begin
        #2000000000;
        $display("RESULT: FAIL (global timeout)");
        $finish;
    end

endmodule