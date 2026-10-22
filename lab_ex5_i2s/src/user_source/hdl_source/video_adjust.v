// video_adjust.v
// 亮度/对比度调节模块（纯组合，不引入延迟，保证与 de/vs 对齐）
// 公式: out = clamp( (in-128)*gain/128 + 128 + bright, 0, 255 )
// 输入: 两对拨码开关，各 4 档。
// 拨码开关接法: 带 10K 上拉（OFF=1, ON=0），所以"全部 OFF"=11=默认档。
// 若你的板子拨码方向相反，把 case 分支顺序对调即可。
module video_adjust(
    input  wire        de,
    input  wire [1:0]  sw_bright,    // 亮度档位
    input  wire [1:0]  sw_contrast,  // 对比度档位
    input  wire [23:0] rgb_in,
    output wire [23:0] rgb_out
);
    reg [7:0]         gain;
    reg signed [8:0]  bright;

    always @(*) begin
        case (sw_bright)              // 亮度（OFF=1, ON=0）
            2'd0:    bright = 9'sd80;      // 00 两拨都ON = 很亮
            2'd1:    bright = -9'sd40;     // 01 = 暗
            2'd2:    bright = 9'sd40;      // 10 = 亮
            default: bright = 9'sd0;       // 11 全OFF = 正常（默认）
        endcase
        case (sw_contrast)            // 对比度增益
            2'd0:    gain = 8'd192;        // 00 = 1.5x
            2'd1:    gain = 8'd96;         // 01 = 0.75x
            2'd2:    gain = 8'd160;        // 10 = 1.25x
            default: gain = 8'd128;        // 11 全OFF = 1.0x（默认）
        endcase
    end

    wire signed [16:0] t_r, t_g, t_b;
    assign t_r = (($signed({1'b0, rgb_in[23:16]}) - 17'sd128) * $signed({1'b0, gain})) / 128 + 17'sd128 + bright;
    assign t_g = (($signed({1'b0, rgb_in[15:8]})  - 17'sd128) * $signed({1'b0, gain})) / 128 + 17'sd128 + bright;
    assign t_b = (($signed({1'b0, rgb_in[7:0]})   - 17'sd128) * $signed({1'b0, gain})) / 128 + 17'sd128 + bright;

    assign rgb_out[23:16] = de ? ((t_r > 17'sd255) ? 8'd255 : (t_r < 17'sd0) ? 8'd0 : t_r[7:0]) : 8'd0;
    assign rgb_out[15:8]  = de ? ((t_g > 17'sd255) ? 8'd255 : (t_g < 17'sd0) ? 8'd0 : t_g[7:0]) : 8'd0;
    assign rgb_out[7:0]   = de ? ((t_b > 17'sd255) ? 8'd255 : (t_b < 17'sd0) ? 8'd0 : t_b[7:0]) : 8'd0;
endmodule
