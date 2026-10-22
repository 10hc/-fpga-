`timescale 1ns/1ps
// tb_osd_sub.v — 验证字幕版 OSD：图像区应原样透传，底部黑条+白字
module tb_osd_sub;
    reg clk = 0;
    always #20 clk = ~clk;     // 25MHz

    reg rst = 1, de = 0, vs = 0;
    reg [23:0] rgb_in = 24'h336699;
    wire [23:0] rgb_out;

    osd_overlay u_dut(
        .clk(clk), .rst(rst), .de(de), .vs(vs),
        .osd_en(1'b1), .rgb_in(rgb_in), .rgb_out(rgb_out)
    );

    integer line, col;
    integer passthru_ok, passthru_bad, bar_ok, bar_bad, text_white, text_black;
    initial begin
        passthru_ok=0; passthru_bad=0; bar_ok=0; bar_bad=0; text_white=0; text_black=0;
    end

    // 检查：在 de 有效期间，按行号统计输出
    always @(posedge clk) begin
        if (!rst && de) begin
            // 注意：OSD 输出比输入晚 1 拍，用 line 近似判断
            if (line < 454) begin
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
        $display("RESULT: passthru_ok=%0d passthru_bad=%0d | bar_white=%0d bar_black=%0d bar_other=%0d",
                 passthru_ok, passthru_bad, text_white, text_black, bar_bad);
        $stop;
    end
endmodule
