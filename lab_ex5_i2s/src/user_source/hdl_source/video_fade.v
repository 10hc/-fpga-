// video_fade.v
// 淡入淡出（视频域）：out = in × fade_factor / 256
// 纯组合输出（0 延迟）；fade_factor 跨时钟域（来自 mem_clk 域），内部两级同步。
module video_fade(
    input  wire        clk,           // video_clk(25MHz)
    input  wire        rst,           // 高有效
    input  wire        de,
    input  wire [8:0]  fade_factor,   // 0=全黑  256=不衰减
    input  wire [23:0] rgb_in,
    output wire [23:0] rgb_out
);
    reg [8:0] f_s0, f_s1;
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            f_s0 <= 9'd256;
            f_s1 <= 9'd256;
        end else begin
            f_s0 <= fade_factor;
            f_s1 <= f_s0;
        end
    end

    assign rgb_out[23:16] = de ? ((rgb_in[23:16] * f_s1) >> 8) : 8'd0;
    assign rgb_out[15:8]  = de ? ((rgb_in[15:8]  * f_s1) >> 8) : 8'd0;
    assign rgb_out[7:0]   = de ? ((rgb_in[7:0]   * f_s1) >> 8) : 8'd0;
endmodule
