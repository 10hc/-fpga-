// -----------------------------------------------------------------------------
// sobel_edge.v —— 3×3 Sobel 边缘检测（经典算子），带 enable
//   · 输入 24bit RGB 像素流（de/vs 同步），输出 8bit 灰度边缘图（{E,E,E}）
//   · 三条行缓冲（640×8，**块 RAM**）构成 3×3 窗口
//     —— 三缓冲（而不是两缓冲）是为了彻底避开“读写同一块 RAM、依赖 read-first
//        语义”的隐患：写第 p 块、读第 p+1/p+2 块，永不冲突。
//   · 块 RAM 读出多一拍寄存 → 数据通路上多一级流水；灰度窗口 reg 同步后移一拍，
//     3×3 窗的九个数仍然严格对齐（见下面的注释图）
//   · enable 由上层做“旁路选择”：本模块永远算，关掉时上层直接走原图，
//     这样两种模式都不破坏时序对齐（切换瞬间会有约 2 行的过渡，可接受）
// 资源估算：约 350 LUT + 3×640×8 BRAM（≈2 个 9K）+ 少量寄存器
// -----------------------------------------------------------------------------
`timescale 1ns / 1ps

module sobel_edge #(
    parameter integer WIDTH = 640
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

    // ---------------- 1) RGB → 灰度 Y = (R + 2G + B) >> 2 ----------------
    reg  [7:0] g;
    reg        de_d;
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            g    <= 8'd0;
            de_d <= 1'b0;
        end else begin
            // 灰度用最省的 (R + 2G + B) >> 2：
            // 原来写 77R+150G+29B 被综合成 3 个 24bit 常数乘法器（TD 没用 DSP，全落 LUT），
            // 综合后出现 slack = -13.931ns 的致命路径（u_sobel_edge/g_reg[*]），并导致
            // P&R 报 "Abort timing optimization due to too bad SWNS"。
            // 边缘检测对亮度权重不敏感，这个廉价近似只要 2 个 8bit 加法器。
            g    <= ({1'b0, rgb_in[23:16]} + {rgb_in[15:8], 1'b0} + {1'b0, rgb_in[7:0]}) >> 2;
            de_d <= de;
        end
    end

    // ---------------- 2) 三条行缓冲（块 RAM）+ 行内地址 ----------------
    (* syn_ramstyle = "block_ram" *) reg [7:0] lb0 [0:WIDTH-1];
    (* syn_ramstyle = "block_ram" *) reg [7:0] lb1 [0:WIDTH-1];
    (* syn_ramstyle = "block_ram" *) reg [7:0] lb2 [0:WIDTH-1];

    reg  [1:0] phase;             // 本行写哪一块（0/1/2 轮转）
    reg  [9:0] x;                 // 行内像素计数
    reg  [1:0] rd_p1, rd_p2;      // 读 y-1 / y-2 的块号

    wire wr0 = de_d && (phase == 2'd0);
    wire wr1 = de_d && (phase == 2'd1);
    wire wr2 = de_d && (phase == 2'd2);

    // 【关键】三块 RAM 的“写 + 寄存读”必须写在同一个 always 块里，
    // 且每个数组的读口各自寄存（不能先做多路选择再寄存），否则安路综合器
    // 认不出 RAM，会退化成分布式 RAM（上一版就是因此占了 3628 个 LUT）。
    reg [7:0] q0, q1, q2;
    always @(posedge clk) begin
        if (wr0) lb0[x] <= g;
        if (wr1) lb1[x] <= g;
        if (wr2) lb2[x] <= g;
        if (de_d) begin
            q0 <= lb0[x];
            q1 <= lb1[x];
            q2 <= lb2[x];
        end
    end

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            phase <= 2'd0;
            x     <= 10'd0;
        end else if (de_d) begin
            if (x == WIDTH-1) begin
                x     <= 10'd0;
                phase <= (phase == 2'd2) ? 2'd0 : phase + 2'd1;
            end else begin
                x <= x + 10'd1;
            end
        end
    end

    // 读口：本行写 phase，则 y-1 在 phase+1、y-2 在 phase+2
    always @(*) begin
        case (phase)
            2'd0:    begin rd_p1 = 2'd1; rd_p2 = 2'd2; end
            2'd1:    begin rd_p1 = 2'd2; rd_p2 = 2'd0; end
            default: begin rd_p1 = 2'd0; rd_p2 = 2'd1; end
        endcase
    end

    // 寄存后的读值做选择：r1_q = 上一行，r2_q = 上两行（与 q0..q2 同拍）
    wire [7:0] r1_q = (rd_p1 == 2'd0) ? q0 : (rd_p1 == 2'd1) ? q1 : q2;
    wire [7:0] r2_q = (rd_p2 == 2'd0) ? q0 : (rd_p2 == 2'd1) ? q1 : q2;

    // ---------------- 3) 3×3 窗（与 r1_q/r2_q 对齐）----------------
    reg  [7:0] g_d1;             // 本行 x（与 r1_q 同拍）
    reg  [7:0] c1, c2;           // 本行 x-1, x-2
    reg  [7:0] m1a, m1b;         // y-1 行 x-1, x-2
    reg  [7:0] m2a, m2b;         // y-2 行 x-1, x-2

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            g_d1 <= 8'd0;
            c1 <= 8'd0; c2 <= 8'd0;
            m1a <= 8'd0; m1b <= 8'd0;
            m2a <= 8'd0; m2b <= 8'd0;
        end else if (de_d) begin
            g_d1 <= g;
            c1   <= g_d1;  c2 <= c1;
            m1a  <= r1_q;  m1b <= m1a;
            m2a  <= r2_q;  m2b <= m2a;
        end
    end

    // ---------------- 4) Sobel ------------------------------------------------
    // 九个数（同一拍，严格对齐）：
    //   p00 p01 p02     c2   c1    g_d1      ← 本行   y
    //   p10 p11 p12  =  m1b  m1a   r1_q      ← 上一行 y-1
    //   p20 p21 p22     m2b  m2a   r2_q      ← 上两行 y-2
    wire [9:0] gx_raw = {2'b00, c2} + {1'b0, c1, 1'b0} + {2'b00, g_d1}
                     - ({2'b00, m2b} + {1'b0, m2a, 1'b0} + {2'b00, r2_q});
    wire [9:0] gy_raw = {2'b00, m2b} + {1'b0, m1b, 1'b0} + {2'b00, c2}
                     - ({2'b00, r2_q} + {1'b0, r1_q, 1'b0} + {2'b00, g_d1});

    wire [9:0] mag = (gx_raw[9] ? ~gx_raw + 10'd1 : gx_raw)
                   + (gy_raw[9] ? ~gy_raw + 10'd1 : gy_raw);
    wire [7:0] edge_mag = (mag > 10'd255) ? 8'hFF : mag[7:0];

    reg [7:0] edge_r;
    always @(posedge clk or posedge rst) begin
        if (rst) edge_r <= 8'd0;
        // 每行最前面 3 个像素的窗口会跨行（左邻取到上一行末尾），会画出一条竖线，
        // 这里直接压黑，省得屏幕上出现一道贴边的亮线。
        else if (de_d) edge_r <= (x < 10'd3) ? 8'd0 : edge_mag;
    end

    assign rgb_out = {edge_r, edge_r, edge_r};

    // ---------------- 5) de/vs 输出 ----------------
    // 不做多级移位寄存器：块 RAM 读出多一拍后，边缘图相对原图只错位
    // “2 行 + 3 像素”，肉眼看不出，所以 de/vs 直接跟随输入，省触发器。
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