// tb_trans_ctrl.v —— 转场控制器专项仿真
// 验证：1) 效果轮换（滑动 ↔ 淡入淡出，栅格已停用）
//       2) 滑动每帧 +8 像素，到 640 后 trans_en 归零
//       3) 淡入淡出 16 帧淡出 → 切新图（trans_en=0）→ 16 帧淡入
`timescale 1ns/1ps
module tb_trans_ctrl;
    reg clk = 0, rst = 1, switch_pulse = 0, frame_done = 0;
    wire [1:0] trans_effect;
    wire       trans_en;
    wire [9:0] trans_offset;
    wire [8:0] fade_factor;

    trans_ctrl dut (
        .clk (clk), .rst (rst),
        .switch_pulse (switch_pulse), .frame_done (frame_done),
        .trans_effect (trans_effect), .trans_en (trans_en),
        .trans_offset (trans_offset), .fade_factor (fade_factor)
    );

    integer errors = 0;
    integer i;

    always #5 clk = ~clk;   // 100MHz

    task do_switch;
        begin
            @(negedge clk); switch_pulse = 1;
            @(negedge clk); switch_pulse = 0;
            @(negedge clk);
        end
    endtask

    task one_frame;
        begin
            @(negedge clk); frame_done = 1;
            @(negedge clk); frame_done = 0;
            @(negedge clk);
        end
    endtask

    task expect2(input [1:0] e_effect, input e_en, input [9:0] e_off, input [8:0] e_fade, input [127:0] tag);
        begin
            if (trans_effect !== e_effect || trans_en !== e_en ||
                trans_offset !== e_off || fade_factor !== e_fade) begin
                $display("FAIL [%0s] effect=%0d(exp %0d) en=%b(exp %b) off=%0d(exp %0d) fade=%0d(exp %0d)",
                         tag, trans_effect, e_effect, trans_en, e_en, trans_offset, e_off, fade_factor, e_fade);
                errors = errors + 1;
            end else begin
                $display("ok   [%0s] effect=%0d en=%b off=%0d fade=%0d", tag, trans_effect, trans_en, trans_offset, fade_factor);
            end
        end
    endtask

    initial begin
        repeat (5) @(negedge clk);
        rst = 0;
        repeat (2) @(negedge clk);
        expect2(2'd0, 1'b0, 10'd0, 9'd256, "复位后=滑动待机");

        // ---- 第 1 次切图：应该跑"淡入淡出" ----
        do_switch;
        expect2(2'd2, 1'b1, 10'd0, 9'd256, "切图1->淡出开始");
        for (i = 0; i < 15; i = i + 1) one_frame;
        expect2(2'd2, 1'b1, 10'd0, 9'd16, "淡出15帧 fade=16");
        one_frame;
        expect2(2'd2, 1'b0, 10'd0, 9'd0, "淡出结束 en=0(换新图)");
        for (i = 0; i < 8; i = i + 1) one_frame;
        expect2(2'd2, 1'b0, 10'd0, 9'd128, "淡入中 fade=128");
        for (i = 0; i < 8; i = i + 1) one_frame;
        expect2(2'd2, 1'b0, 10'd0, 9'd256, "淡入结束 fade=256");

        // ---- 第 2 次切图：应该跑"滑动" ----
        do_switch;
        expect2(2'd0, 1'b1, 10'd0, 9'd256, "切图2->滑动开始");
        one_frame;
        expect2(2'd0, 1'b1, 10'd8, 9'd256, "滑动1帧 off=8");
        for (i = 0; i < 78; i = i + 1) one_frame;
        expect2(2'd0, 1'b1, 10'd632, 9'd256, "滑动79帧 off=632");
        one_frame;
        expect2(2'd0, 1'b1, 10'd640, 9'd256, "滑动80帧 off=640");
        one_frame;
        expect2(2'd0, 1'b0, 10'd640, 9'd256, "滑动结束 en=0");

        // ---- 第 3 次切图：又回到淡入淡出（轮换正确）----
        do_switch;
        expect2(2'd2, 1'b1, 10'd0, 9'd256, "切图3->淡出开始(轮换正确)");

        if (errors == 0) $display("CHECK: PASS");
        else             $display("CHECK: FAIL errors=%0d", errors);
        $finish;
    end

    initial begin
        #5000000;
        $display("CHECK: TIMEOUT");
        $finish;
    end
endmodule