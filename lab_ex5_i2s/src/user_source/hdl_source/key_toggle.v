// key_toggle.v
// 按键消抖 + 电平翻转开关：每按一次按键，输出翻转一次。
// 用于 KEY_3(B1) 控制 OSD 开关。按键默认松开=1、按下=0（低有效，与板载按键一致）。
module key_toggle #(
    parameter integer CLK_FREQ_HZ = 25_000_000,
    parameter integer DEBOUNCE_MS = 50
)(
    input  wire clk,        // 建议接 video_clk(25MHz)，与 OSD 同域
    input  wire rst,        // 高有效复位
    input  wire button,     // 按键输入：松开=1，按下=0
    output reg  osd_on      // 输出状态：初始为 1（开）
);
    localparam integer CNT = (CLK_FREQ_HZ / 1000) * DEBOUNCE_MS;

    reg [1:0]  sync;      // 两级同步
    reg        stable;    // 消抖后的稳定值
    reg [31:0] cnt;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            sync   <= 2'b11;
            stable <= 1'b1;
            cnt    <= 32'd0;
            osd_on <= 1'b1;       // 上电默认 OSD 开
        end else begin
            sync <= {sync[0], button};
            if (sync[1] == stable) begin
                cnt <= 32'd0;     // 电平稳定，计数清零
            end else if (cnt >= CNT - 1) begin
                if (stable && !sync[1]) begin
                    osd_on <= ~osd_on;   // 检测到按下沿 → 翻转
                end
                stable <= sync[1];
                cnt    <= 32'd0;
            end else begin
                cnt <= cnt + 32'd1;      // 抖动计数中
            end
        end
    end
endmodule
