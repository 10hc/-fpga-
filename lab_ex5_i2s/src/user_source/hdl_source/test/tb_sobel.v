// tb_sobel.v —— 验证 sobel_edge（块 RAM 三缓冲版）的 3×3 对齐
//   测试图：640×480，左上 320×240 为亮(200)，其余为暗(60) → 竖边 x=320、横边 y=240
//   检查：输出强边缘只应出现在这两条边附近；若行缓冲对齐错一拍，会出现大面积误报
`timescale 1ns / 1ps

module tb_sobel;

    reg         clk = 1'b0;
    reg         rst = 1'b1;
    reg         de  = 1'b0;
    reg         vs  = 1'b0;
    reg  [23:0] rgb = 24'd0;
    wire        de_out, vs_out;
    wire [23:0] rgb_out;

    sobel_edge #(.WIDTH(640)) dut (
        .clk(clk), .rst(rst), .de(de), .vs(vs), .rgb_in(rgb),
        .de_out(de_out), .vs_out(vs_out), .rgb_out(rgb_out)
    );

    always #20 clk = ~clk;          // 25MHz

    // ---------------- 统计输出流 ----------------
    // 判据（与帧号无关，按行内位置折算）：
    //   · 竖边（源图 x=320）→ 输出列 320±3 应有一列强边缘
    //   · 横边（源图 y=240）→ 输出行 240±2 应有一行强边缘
    //   · 除这两条边、以及每帧头 4 行/行头 8 像素（行缓冲尚未填满，属正常过渡）之外，
    //     不应出现任何强边缘
    integer col_o = 0, row_o = 0;
    integer n_edge = 0, n_bad = 0, n_col = 0, n_row = 0;
    integer c, r;
    always @(posedge clk) begin
        if (de_out) begin
            if (col_o == 639) begin col_o <= 0; row_o <= row_o + 1; end
            else              col_o <= col_o + 1;

            if (rgb_out != 24'd0) begin
                c = col_o;  r = row_o % 480;
                n_edge = n_edge + 1;
                if (c >= 317 && c <= 326) n_col = n_col + 1;
                if (r >= 238 && r <= 242) n_row = n_row + 1;
                if (!((c >= 317 && c <= 326) || (r >= 238 && r <= 242))) begin
                    if (!(r <= 4 || c <= 8)) n_bad = n_bad + 1;
                end
            end
        end
    end

    // ---------------- 激励 ----------------
    integer frame, row, col;
    initial begin
        repeat (5) @(posedge clk);
        #1 rst = 1'b0;
        repeat (5) @(posedge clk);

        for (frame = 0; frame < 2; frame = frame + 1) begin
            @(negedge clk); vs = 1'b1;
            @(negedge clk); vs = 1'b0;
            for (row = 0; row < 480; row = row + 1) begin
                for (col = 0; col < 640; col = col + 1) begin
                    @(negedge clk);
                    de  = 1'b1;
                    rgb = (row < 240 && col < 320) ? 24'hC8C8C8 : 24'h3C3C3C;
                end
                @(negedge clk);
                de = 1'b0;
                repeat (4) @(negedge clk);
            end
        end
        repeat (50) @(posedge clk);

        $display("RESULT: edges=%0d  col_band=%0d  row_band=%0d  bad=%0d", n_edge, n_col, n_row, n_bad);
        if (n_bad == 0 && n_col > 200 && n_row > 300)
            $display("CHECK: PASS");
        else
            $display("CHECK: FAIL");
        $finish;
    end

endmodule