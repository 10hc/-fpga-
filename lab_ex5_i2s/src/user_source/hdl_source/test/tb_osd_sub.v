`timescale 1ns/1ps
// tb_osd_sub.v — 验证字幕版 OSD：图像区应原样透传，底部黑条+白字
module tb_osd_sub;
    reg clk = 0;
    always #20 clk = ~clk;     // 25MHz

    reg rst = 1, de = 0, vs = 0;
    reg [23:0] rgb_in = 24'h336699;
    reg [7:0]  key_dbg = 8'b1010_1010;      // 棋盘格，便于检查方块
    wire [23:0] rgb_out;

    osd_overlay u_dut(
        .clk(clk), .rst(rst), .de(de), .vs(vs),
        .osd_en(1'b1), .rgb_in(rgb_in), .key_dbg(key_dbg), .rgb_out(rgb_out)
    );

    integer line, col;
    integer passthru_ok, passthru_bad, bar_ok, bar_bad, text_white, text_black;
    integer dbg_red, dbg_green, dbg_other;
    integer gap_ok, gap_bad;
    integer dx, idx;
    initial begin
        passthru_ok=0; passthru_bad=0; bar_ok=0; bar_bad=0; text_white=0; text_black=0;
        dbg_red=0; dbg_green=0; dbg_other=0; gap_ok=0; gap_bad=0;
    end

    // 检查：在 de 有效期间，按行号/列号统计输出
    //   行 8..23 且列 8..135 = 顶部诊断方块区（每格宽 16、右侧留 4 像素缝）
    //   其余行为图像透传；底部 20 行是黑条 + 白字
    // 注意：DUT 输出比输入晚 1 拍，列号边界会有 ±1 列的错位，故 CHECK 留容差
    always @(posedge clk) begin
        if (!rst && de) begin
            if (line >= 8 && line < 24 && col >= 8 && col < 136) begin
                // 方块区：红/绿方块 + 4 像素缝隙（缝隙透传图像）。
                // tb 的列计数与 DUT 输出有 1~2 列错位，故只按颜色计数、CHECK 留容差。
                if      (rgb_out === 24'hFF0000) dbg_red   = dbg_red   + 1;
                else if (rgb_out === 24'h00FF00) dbg_green = dbg_green + 1;
                else if (rgb_out === 24'h336699) gap_ok    = gap_ok    + 1;
                else                             dbg_other = dbg_other + 1;
            end else if (line < 454 || (line >= 8 && line < 24)) begin
                if (rgb_out === 24'h336699) passthru_ok = passthru_ok + 1;
                else passthru_bad = passthru_bad + 1;
            end else if (line < 474) begin
                if (rgb_out === 24'hFFFFFF) text_white = text_white + 1;
                else if (rgb_out === 24'h000000) text_black = text_black + 1;
                else if (rgb_out === 24'h336699) bar_bad = bar_bad + 1;
                else bar_ok = bar_ok + 1;
            end
        end
    end

    // 简化时序：每行 640 个 de，480 行
    initial begin
        rst = 1;
        repeat (10) @(posedge clk);
        rst = 0;
        @(posedge clk);
        for (line = 0; line < 480; line = line + 1) begin
            vs <= (line == 0) ? 1'b1 : 1'b0;
            @(posedge clk);
            vs <= 1'b0;
            de <= 1'b1;
            for (col = 0; col < 640; col = col + 1) @(posedge clk);
            de <= 1'b0;
            repeat (4) @(posedge clk);
        end
        $display("RESULT: passthru_ok=%0d passthru_bad=%0d | bar_white=%0d bar_black=%0d bar_other=%0d | dbg_red=%0d dbg_green=%0d dbg_other=%0d gap_ok=%0d",
                 passthru_ok, passthru_bad, text_white, text_black, bar_bad,
                 dbg_red, dbg_green, dbg_other, gap_ok);
        // key_dbg=1010_1010 → 4 格红 4 格绿，每格 12 列 × 16 行 = 192 → 期望各 768
        // tb 列计数与 DUT 输出有 1~2 列错位，CHECK 留 ±64 容差
        if (dbg_red >= 704 && dbg_red <= 832 &&
            dbg_green >= 704 && dbg_green <= 832 &&
            dbg_other <= 64 && passthru_bad <= 64 &&
            gap_ok >= 448 && text_white > 0)
            $display("CHECK: PASS");
        else
            $display("CHECK: FAIL");
        $stop;
    end
endmodule
