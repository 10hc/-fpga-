// trans_ctrl.v
// 多效果转场控制器：
//   每次切图（switch_pulse）自动轮换效果：滑动 → 栅格 → 淡入淡出 → 滑动 → ...
//   推进以"帧完成脉冲 frame_done"为节拍，每帧内参数恒定 → 画面平滑。
// 输出：
//   trans_effect  : 0=滑动  1=栅格  2=淡入淡出
//   trans_en      : 读控制器转场使能（滑动/栅格全程；淡出的"显示旧图"阶段）
//   trans_offset  : 滑动=像素偏移(0~639)；栅格=对角线进度(0~68)
//   fade_factor   : 淡入淡出因子 0(黑)~256(不衰减)，其余效果恒为 256
module trans_ctrl #(
    parameter integer SLIDE_STEP = 8,    // 滑动：每帧前进像素（8≈1.3s，越小越丝滑）
    parameter integer GRID_STEP  = 2,    // 栅格：每帧前进对角档（2≈0.6s）
    parameter integer FADE_STEP  = 16    // 淡入/淡出：每帧因子增量（16≈每段0.27s）
)(
    input  wire        clk,            // ext_mem_clk(125MHz)
    input  wire        rst,            // 高有效
    input  wire        switch_pulse,   // 切图脉冲（disp_buf_idx 变化，1 拍）
    input  wire        frame_done,     // 每帧读完脉冲（read_finish）
    output reg  [1:0]  trans_effect,
    output reg         trans_en,
    output reg  [9:0]  trans_offset,
    output reg  [8:0]  fade_factor     // 0~256
);
    localparam FX_SLIDE = 2'd0;
    localparam FX_GRID  = 2'd1;
    localparam FX_FADE  = 2'd2;

    localparam F_OUT = 2'd1;   // 淡出（显示旧图）
    localparam F_IN  = 2'd2;   // 淡入（显示新图）

    reg [1:0]  f_phase;        // 淡入淡出相位（仅 FX_FADE 用）
    reg [7:0]  trans_frames;   // 转场已持续的帧数（安全阀用）

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            trans_effect <= FX_SLIDE;
            trans_en     <= 1'b0;
            trans_offset <= 10'd0;
            fade_factor  <= 9'd256;
            f_phase      <= 2'd0;
            trans_frames <= 8'd0;
        end else if (switch_pulse) begin
            // 轮换效果并启动
            // 注意：上板实测"栅格"效果花屏（2026-10-24），暂时停用，
            // 只轮换 滑动(0) / 淡入淡出(2)，修好后把 FX_SLIDE 的下一效果改回 FX_GRID。
            case (trans_effect)
                FX_SLIDE: begin
                    trans_effect <= FX_FADE;
                    trans_en     <= 1'b1;      // 淡出阶段：读控制器保持显示旧图
                    trans_offset <= 10'd0;     // 偏移0 = 整行旧图
                    fade_factor  <= 9'd256;
                    f_phase      <= F_OUT;
                    trans_frames <= 8'd0;
                end
                default: begin                  // FX_FADE（或 FX_GRID）→ 下一个是 SLIDE
                    trans_effect <= FX_SLIDE;
                    trans_en     <= 1'b1;
                    trans_offset <= 10'd0;
                    fade_factor  <= 9'd256;
                    trans_frames <= 8'd0;
                end
            endcase
        end else if (frame_done) begin
            if (trans_en)
                trans_frames <= trans_frames + 8'd1;
            case (trans_effect)
                FX_SLIDE: begin
                    if (trans_offset >= 10'd640)
                        trans_en <= 1'b0;
                    else if (trans_offset > 10'd640 - SLIDE_STEP)
                        trans_offset <= 10'd640;
                    else
                        trans_offset <= trans_offset + SLIDE_STEP;
                end
                FX_GRID: begin
                    if (trans_offset >= 10'd69)
                        trans_en <= 1'b0;
                    else if (trans_offset > 10'd69 - GRID_STEP)
                        trans_offset <= 10'd69;
                    else
                        trans_offset <= trans_offset + GRID_STEP;
                end
                FX_FADE: begin
                    case (f_phase)
                        F_OUT: begin
                            if (fade_factor <= FADE_STEP) begin
                                fade_factor <= 9'd0;
                                f_phase     <= F_IN;
                                trans_en    <= 1'b0;   // 换读新图
                            end else begin
                                fade_factor <= fade_factor - FADE_STEP;
                            end
                        end
                        F_IN: begin
                            if (fade_factor >= 9'd256 - FADE_STEP) begin
                                fade_factor <= 9'd256;
                                f_phase     <= 2'd0;
                            end else begin
                                fade_factor <= fade_factor + FADE_STEP;
                            end
                        end
                        default: fade_factor <= 9'd256;
                    endcase
                end
                default: ;
            endcase

            // ---- 安全阀：任何效果超过 200 帧仍未收尾，强制结束，避免画面卡死在转场中 ----
            if (trans_en && trans_frames >= 8'd200) begin
                trans_en     <= 1'b0;
                trans_offset <= 10'd640;     // 滑动视为已到终点
                fade_factor  <= 9'd256;      // 淡入淡出视为完全显示
                f_phase      <= 2'd0;
            end
        end
    end
endmodule
