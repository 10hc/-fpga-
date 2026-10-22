// ============================================================
// video_adjust.v —— 亮度 / 对比度 / 淡入淡出（单乘法器版）
//
// 公式:  out = (in - 128) * eff_gain / 128 + post
//   其中  eff_gain = gain * fade / 256      （有效对比度增益）
//         post     = (128 + bright) * fade / 256 （有效偏置）
//
// 为什么要折叠：
//   fade=256（不淡变）时 eff_gain=gain、post=128+bright，与原公式
//     out = (in-128)*gain/128 + 128 + bright
//   完全等价（逐位相同）。而把淡变折叠成"每拍预算的常数"后，
//   像素数据路径上只剩 1 个乘法器，避免 125MHz 下时序违例。
//
// 端口说明：
//   fade 来自 ext_mem_clk 域的转场控制器（0=全黑, 256=不衰减），
//   模块内部做两级同步。
//   像素路径纯组合（0 延迟），与 OSD 的 1 拍流水线对齐。
// ============================================================
module video_adjust(
    input  wire        clk,
    input  wire        rst,
    input  wire        de,
    input  wire [1:0]  sw_bright,     // 亮度档位
    input  wire [1:0]  sw_contrast,   // 对比度档位
    input  wire [8:0]  fade,          // 淡入淡出因子（跨时钟域，内部同步）
    input  wire [23:0] rgb_in,
    output wire [23:0] rgb_out
);
    // ---------------- 档位译码 ----------------
    reg [7:0]         gain;
    reg signed [8:0]  bright;

    always @(*) begin
        case (sw_bright)                  // 亮度（默认 11 = 正常）
            2'd0:    bright = 9'sd80;
            2'd1:    bright = -9'sd40;
            2'd2:    bright = 9'sd40;
            default: bright = 9'sd0;
        endcase
        case (sw_contrast)                // 对比度增益（128 = 1.0×）
            2'd0:    gain = 8'd192;
            2'd1:    gain = 8'd96;
            2'd2:    gain = 8'd160;
            default: gain = 8'd128;
        endcase
    end

    // ---------------- fade 两级同步（来自 mem_clk 域）----------------
    reg [8:0] f_s0, f_s1;
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            f_s0 <= 9'd256;
            f_s1 <= 9'd256;
        end else begin
            f_s0 <= fade;
            f_s1 <= f_s0;
        end
    end

    // ---------------- 每拍预计算两个常数（不经过像素数据）----------------
    wire        [16:0] gain_mul = gain * f_s1;                    // 8×9 -> 17 位
    wire signed [17:0] base_s   = 18'sd128 + bright;              // (128 + 亮度)
    wire signed [26:0] post_mul = base_s * $signed({1'b0, f_s1}); // 18×9 -> 27 位

    reg [8:0]         eff_gain_r;      // gain*fade/256
    reg signed [16:0] post_r;          // (128+bright)*fade/256

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            eff_gain_r <= 9'd128;
            post_r     <= 17'sd128;
        end else begin
            eff_gain_r <= gain_mul[16:8];     // >>8
            post_r     <= post_mul[24:8];     // 算术右移 8 位
        end
    end

    // ---------------- 像素路径（仅 1 个乘法器）----------------
    wire signed [16:0] t_r, t_g, t_b;
    assign t_r = (($signed({1'b0, rgb_in[23:16]}) - 17'sd128) * $signed({1'b0, eff_gain_r})) / 128 + post_r;
    assign t_g = (($signed({1'b0, rgb_in[15:8]})  - 17'sd128) * $signed({1'b0, eff_gain_r})) / 128 + post_r;
    assign t_b = (($signed({1'b0, rgb_in[7:0]})   - 17'sd128) * $signed({1'b0, eff_gain_r})) / 128 + post_r;

    assign rgb_out[23:16] = de ? ((t_r > 17'sd255) ? 8'd255 : (t_r < 17'sd0) ? 8'd0 : t_r[7:0]) : 8'd0;
    assign rgb_out[15:8]  = de ? ((t_g > 17'sd255) ? 8'd255 : (t_g < 17'sd0) ? 8'd0 : t_g[7:0]) : 8'd0;
    assign rgb_out[7:0]   = de ? ((t_b > 17'sd255) ? 8'd255 : (t_b < 17'sd0) ? 8'd0 : t_b[7:0]) : 8'd0;
endmodule