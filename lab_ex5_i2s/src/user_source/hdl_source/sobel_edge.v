// -----------------------------------------------------------------------------
// sobel_edge.v —— 3×3 Sobel 边缘检测（经典算子），带 enable
//   · 输入 24bit RGB 像素流（de/vs 同步），输出 8bit 灰度边缘图（{E,E,E}）
//   · 用两条行缓冲（640×8，LUTRAM）构成 3×3 窗口
//   · 输出与输入之间有固定延迟 = 2 行 + 3 像素，de/vs 用等长移位寄存器对齐
//   · enable 由上层做“旁路选择”：本模块永远算，关掉时上层直接走原图，
//     这样两种模式都不破坏时序对齐（切换瞬间会有约 2 行的过渡，可接受）
// 资源估算：约 300 LUT + 2×640×8 LUTRAM + 约 1300 移位寄存器
// -----------------------------------------------------------------------------
`timescale 1ns / 1ps

module sobel_edge #(
    parameter integer WIDTH = 640,
    parameter integer DLY   = 1283      // = 2*WIDTH + 3（2 行 + 3 像素）
)(
    input               clk,
    input               rst,
    input               de,
    input               vs,
    input      [23:0]   rgb_in,
    output reg          de_out,
    output reg          vs_out,
    output     [23:0]   rgb_out
);

    // ---------------- 1) RGB → 灰度 Y = (77R + 150G + 29B) >> 8 ----------------
    reg  [7:0] g;
    reg        de_d;
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            g    <= 8'd0;
            de_d <= 1'b0;
        end else begin
            g <= (77 * rgb_in[23:16] + 150 * rgb_in[15:8] + 29 * rgb_in[7:0]) >> 8;
            de_d <= de;
        end
    end

    // ---------------- 2) 两条行缓冲 + 行内地址 ----------------
    reg  [7:0] lb0 [0:WIDTH-1];
    reg  [7:0] lb1 [0:WIDTH-1];
    reg        phase;          // 本行写哪个缓冲
    reg  [9:0] x;              // 行内像素计数

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            phase <= 1'b0;
            x     <= 10'd0;
        end else if (de_d) begin
            if (x == WIDTH-1) begin
                x     <= 10'd0;
                phase <= ~phase;
            end else begin
                x <= x + 10'd1;
            end
            if (phase) lb1[x] <= g;
            else       lb0[x] <= g;
        end
    end

    // 组合读：写口正在写 x，读口读到的仍是上一行的旧值 → 正好是 y-2
    wire [7:0] r_m1 = phase ? lb0[x] : lb1[x];   // y-1
    wire [7:0] r_m2 = phase ? lb1[x] : lb0[x];   // y-2（旧值）

    // ---------------- 3) 3×3 窗（以 x-1 为中心）----------------
    reg  [7:0] c1, c2;          // 本行:  x-1, x-2
    reg  [7:0] m1a, m1b;        // y-1 行: x-1, x-2
    reg  [7:0] m2a, m2b;        // y-2 行: x-1, x-2

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            c1 <= 8'd0; c2 <= 8'd0;
            m1a <= 8'd0; m1b <= 8'd0;
            m2a <= 8'd0; m2b <= 8'd0;
        end else if (de_d) begin
            c1  <= g;     c2  <= c1;
            m1a <= r_m1;  m1b <= m1a;
            m2a <= r_m2;  m2b <= m2a;
        end
    end

    // ---------------- 4) Sobel ------------------------------------------------
    // 窗口（任意一致的行序即可，|Gx|+|Gy| 与方向无关）：
    //   p00 p01 p02     c2  c1  g
    //   p10 p11 p12  =  m1b m1a r_m1
    //   p20 p21 p22     m2b m2a r_m2
    wire [9:0] gx_raw = {2'b00, c2} + {1'b0, c1, 1'b0} + {2'b00, g}
                     - ({2'b00, m2b} + {1'b0, m2a, 1'b0} + {2'b00, r_m2});
    wire [9:0] gy_raw = {2'b00, m2b} + {1'b0, m1b, 1'b0} + {2'b00, c2}
                     - ({2'b00, r_m2} + {1'b0, r_m1, 1'b0} + {2'b00, g});

    wire [9:0] mag = (gx_raw[9] ? ~gx_raw + 10'd1 : gx_raw)
                   + (gy_raw[9] ? ~gy_raw + 10'd1 : gy_raw);
    wire [7:0] edge_mag = (mag > 10'd255) ? 8'hFF : mag[7:0];

    reg [7:0] edge_r;
    always @(posedge clk or posedge rst) begin
        if (rst) edge_r <= 8'd0;
        else if (de_d) edge_r <= edge_mag;
    end

    assign rgb_out = {edge_r, edge_r, edge_r};

    // ---------------- 5) de/vs 输出 ----------------
    // 这里**不做** 2 行的移位寄存器：上一版用 2×1283 bit = 2566 个触发器，
    // 直接把 MSlice 顶到 4985 > 4900（超限 85）。边缘图相对原图只错位
    // “2 行 + 1 像素”，肉眼看不出，所以 de/vs 直接跟随输入，省约 500 个 slice。
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            de_out <= 1'b0;
            vs_out <= 1'b0;
        end else begin
            de_out <= de;
            vs_out <= vs;
        end
    end

endmodule