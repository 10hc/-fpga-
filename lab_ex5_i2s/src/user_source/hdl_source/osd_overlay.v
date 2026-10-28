// ============================================================
// osd_overlay.v —— 底部滚动字幕 + 左上诊断方块 + 右上"亮度/对比度/清晰度"信息框
//
// 三个叠加层，互不影响：
//   1) 底部字幕条（黑底白字，横向滚动，8x16 ASCII 字库 font_rom）
//   2) 左上 8 个诊断方块（红=1 绿=0，不受 osd_en 控制，用来判断按键/状态）
//   3) 右上信息框（黑底白字带白边，16x16 点阵汉字字库 cjk_rom）
//      ┌──────────┐
//      │ 亮度   3 │
//      │ 对比度 3 │
//      │ 清晰度 开│
//      └──────────┘
//      数值来源：顶层 bright_sel / contrast_sel / cfg_sobel（SWKEY1 边缘检测）
//
// ● 内部 1 拍流水线：顶层需把送 AXIS 的 de/vs 延迟 1 拍对齐
// ● 汉字字库是 192x16 的 ROM（12 个字模 x 16 行），已加 syn_ramstyle=block_ram
//   让 TD 推断成 BRAM，不占 LUT
// ============================================================
module osd_overlay #(
    parameter integer CHAR_W      = 8,     // 字符宽
    parameter integer CHAR_H      = 16,    // 字符高
    parameter integer TEXT_LEN    = 64,    // 文本字符数（需为 2 的幂）
    parameter integer BAR_Y       = 456,   // 字幕条文字顶部 Y（480-24）
    parameter integer SCROLL_STEP = 2,     // 每帧滚动像素
    parameter integer PX0         = 552,   // 右上信息框左边界（640-8-80）
    parameter integer PY0         = 8,     // 右上信息框上边界
    parameter integer PW          = 80,    // 信息框宽
    parameter integer PH          = 56     // 信息框高（两行：亮度 / 对比度）
)(
    input  wire        clk,
    input  wire        rst,
    input  wire        de,
    input  wire        vs,
    input  wire        osd_en,
    input  wire [23:0] rgb_in,
    input  wire [7:0]  key_dbg,      // 顶部 8 个诊断方块（1=红 0=绿），不受 osd_en 控制
    input  wire [1:0]  bri_level,    // 亮度档位 0..3
    input  wire [1:0]  con_level,    // 对比度档位 0..3
    input  wire        shp_on,       // 清晰度（边缘检测）开关 1=开
    output wire [23:0] rgb_out
);
    // ==================== 文本表：改这一行即可 ====================
    // 必须恰好 TEXT_LEN(64) 个字符；不足用空格补齐
    localparam [TEXT_LEN*8-1:0] OSD_TEXT =
        "ANLU 2026  KEYFIX-V25  FPGA 2026                                ";

    // ---------- x/y 有效像素坐标跟踪 ----------
    reg [9:0] x, y;
    reg       de_r, vs_r;
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            x <= 10'd0; y <= 10'd0; de_r <= 1'b0; vs_r <= 1'b0;
        end else begin
            de_r <= de;
            vs_r <= vs;
            if (de && !de_r)  x <= 10'd0;
            else if (de)      x <= x + 10'd1;
            if (vs && !vs_r)  y <= 10'd0;
            else if (!de && de_r) y <= y + 10'd1;
        end
    end

    // ---------- 滚动偏移：每帧 +SCROLL_STEP ----------
    reg [9:0] scroll;
    always @(posedge clk or posedge rst) begin
        if (rst) scroll <= 10'd0;
        else if (vs && !vs_r) scroll <= scroll + SCROLL_STEP;
    end

    // ---------- 字幕区域 ----------
    wire inside_text = (y >= BAR_Y) && (y < BAR_Y + CHAR_H);
    wire inside_box  = (y >= (BAR_Y - 2)) && (y < (BAR_Y + CHAR_H + 2));

    // ---------- 字符位置（含滚动）----------
    wire [10:0] posx     = {1'b0, x} + {1'b0, scroll};
    wire [5:0]  char_idx = posx[8:3];          // /8，取 64 字符（2^6）循环
    wire [2:0]  char_col = posx[2:0];          // %8
    wire [4:0]  char_row = y - BAR_Y;          // 0..15

    // ---------- 字符码（文本表低字节 = 最后一个字符）----------
    wire [5:0]  byte_idx  = 6'd63 - char_idx;
    wire [7:0]  text_char = OSD_TEXT[byte_idx*8 +: 8];
    wire [6:0]  code_idx  = (text_char[6:0] < 7'h20) ? 7'd0 : (text_char[6:0] - 7'h20);
    wire [10:0] raddr     = {code_idx, 4'd0} + {7'd0, char_row[3:0]};

    // ---------- 字库 ROM（同步读，1 拍延迟）----------
    wire [7:0] glyph_byte;
    font_rom u_font_rom(
        .doa   (glyph_byte),
        .addra (raddr),
        .clka  (clk)
    );

    // ---------- 左上按键诊断方块 ----------
    // 8 个 16x16 方块，从左到右对应 key_dbg[7]..key_dbg[0]，
    // 红色 = 1，绿色 = 0。用来判断"按下按键，FPGA 到底有没有收到"。
    // 方块间距取 16（2 的幂），避免综合出除法器。
    wire [9:0]  dbg_dx  = x - 10'd8;                     // 0..127
    wire        in_dbg  = (x >= 10'd8) && (x < 10'd136) &&
                          (y >= 10'd8) && (y < 10'd24) &&
                          (dbg_dx[3:0] >= 4'd4);         // 每格右侧留 4 像素缝
    wire [2:0]  dbg_idx = dbg_dx[6:4];                   // /16 → 0..7，无除法器
    wire        dbg_val = key_dbg[3'd7 - dbg_idx];       // 左 = bit7

    // ==================== 右上信息框：亮度 / 对比度 / 清晰度 ====================
    // 框内 3 行，行高 24；每行 4 个 16x16 字格（x = 8..71），右侧留白
    wire        in_panel = (x >= PX0) && (x < PX0 + PW) &&
                           (y >= PY0) && (y < PY0 + PH);
    wire [9:0]  ppx = x - PX0;
    wire [9:0]  ppy = y - PY0;

    wire [1:0]  prow  = (ppy < 10'd24) ? 2'd0 : (ppy < 10'd48) ? 2'd1 : 2'd2;
    wire [9:0]  prowy = ppy - ((prow == 2'd0) ? 10'd0 :
                               (prow == 2'd1) ? 10'd24 : 10'd48);
    wire        pborder = in_panel && ((ppx == 10'd0) || (ppx == PW - 1) ||
                                       (ppy == 10'd0) || (ppy == PH - 1));

    // 字格区域：x 8..71（4 格），行内 y 4..19（16 行）
    wire        in_glyph = (ppx >= 10'd8) && (ppx < 10'd72) &&
                           (prowy >= 10'd4) && (prowy < 10'd20);
    wire [2:0]  pcol  = (ppx - 10'd8) >> 4;      // 0..3
    wire [3:0]  pgrow = prowy - 10'd4;           // 0..15
    wire [3:0]  pbit  = ppx[3:0] - 4'd8;                // 列（0..15）

    // 字模编号：0亮 1度 2对 3比 4清 5晰 6开 7关 8..11 = '0'..'3'
    reg  [3:0]  gid;
    always @(*) begin
        case (prow)
            2'd0: case (pcol)
                    3'd0: gid = 4'd0;                            // 亮
                    3'd1: gid = 4'd1;                            // 度
                    3'd3: gid = 4'd8 + {2'd0, bri_level};        // 数值
                    default: gid = 4'd0;
                  endcase
            2'd1: case (pcol)
                    3'd0: gid = 4'd2;                            // 对
                    3'd1: gid = 4'd3;                            // 比
                    3'd2: gid = 4'd1;                            // 度
                    3'd3: gid = 4'd8 + {2'd0, con_level};        // 数值
                    default: gid = 4'd0;
                  endcase
            default: case (pcol)
                    3'd0: gid = 4'd4;                            // 清
                    3'd1: gid = 4'd5;                            // 晰
                    3'd2: gid = 4'd1;                            // 度
                    3'd3: gid = shp_on ? 4'd6 : 4'd7;            // 开 / 关
                    default: gid = 4'd0;
                  endcase
        endcase
    end
    // 第一行第 3 格留空（"亮度" 只有两个字）
    wire        pblank = (prow == 2'd0) && (pcol == 3'd2);
    // 信息框只保留两行（亮度 / 对比度）；第三行区域不再画字
    wire        panel_row_ok = (prow != 2'd2);

    // ---------- 汉字字模 ROM：192 x 16（12 字模 x 16 行）----------
    (* syn_ramstyle = "block_ram" *) reg [15:0] cjk_rom [0:191];
    initial begin
        cjk_rom[  0] = 16'h0000;
        cjk_rom[  1] = 16'h0000;
        cjk_rom[  2] = 16'h0100;
        cjk_rom[  3] = 16'h7FFE;
        cjk_rom[  4] = 16'h0810;
        cjk_rom[  5] = 16'h0FF0;
        cjk_rom[  6] = 16'h0810;
        cjk_rom[  7] = 16'h0000;
        cjk_rom[  8] = 16'h3FFC;
        cjk_rom[  9] = 16'h2004;
        cjk_rom[ 10] = 16'h07E0;
        cjk_rom[ 11] = 16'h0422;
        cjk_rom[ 12] = 16'h1C26;
        cjk_rom[ 13] = 16'h703C;
        cjk_rom[ 14] = 16'h0000;
        cjk_rom[ 15] = 16'h0000;
        cjk_rom[ 16] = 16'h0000;
        cjk_rom[ 17] = 16'h0000;
        cjk_rom[ 18] = 16'h0080;
        cjk_rom[ 19] = 16'h3FFE;
        cjk_rom[ 20] = 16'h3210;
        cjk_rom[ 21] = 16'h3FFE;
        cjk_rom[ 22] = 16'h3210;
        cjk_rom[ 23] = 16'h33F0;
        cjk_rom[ 24] = 16'h3000;
        cjk_rom[ 25] = 16'h3FF8;
        cjk_rom[ 26] = 16'h3210;
        cjk_rom[ 27] = 16'h21E0;
        cjk_rom[ 28] = 16'h63F0;
        cjk_rom[ 29] = 16'h5E1E;
        cjk_rom[ 30] = 16'h0000;
        cjk_rom[ 31] = 16'h0000;
        cjk_rom[ 32] = 16'h0000;
        cjk_rom[ 33] = 16'h7E08;
        cjk_rom[ 34] = 16'h0208;
        cjk_rom[ 35] = 16'h27FE;
        cjk_rom[ 36] = 16'h3608;
        cjk_rom[ 37] = 16'h1488;
        cjk_rom[ 38] = 16'h1CC8;
        cjk_rom[ 39] = 16'h0C48;
        cjk_rom[ 40] = 16'h0C48;
        cjk_rom[ 41] = 16'h1E08;
        cjk_rom[ 42] = 16'h1208;
        cjk_rom[ 43] = 16'h2208;
        cjk_rom[ 44] = 16'h4078;
        cjk_rom[ 45] = 16'h0000;
        cjk_rom[ 46] = 16'h0000;
        cjk_rom[ 47] = 16'h0000;
        cjk_rom[ 48] = 16'h0000;
        cjk_rom[ 49] = 16'h0000;
        cjk_rom[ 50] = 16'h2080;
        cjk_rom[ 51] = 16'h2080;
        cjk_rom[ 52] = 16'h2084;
        cjk_rom[ 53] = 16'h208C;
        cjk_rom[ 54] = 16'h3E98;
        cjk_rom[ 55] = 16'h20F0;
        cjk_rom[ 56] = 16'h20C0;
        cjk_rom[ 57] = 16'h2080;
        cjk_rom[ 58] = 16'h2086;
        cjk_rom[ 59] = 16'h2686;
        cjk_rom[ 60] = 16'h3E86;
        cjk_rom[ 61] = 16'h30FC;
        cjk_rom[ 62] = 16'h0000;
        cjk_rom[ 63] = 16'h0000;
        cjk_rom[ 64] = 16'h0000;
        cjk_rom[ 65] = 16'h2040;
        cjk_rom[ 66] = 16'h37FE;
        cjk_rom[ 67] = 16'h1840;
        cjk_rom[ 68] = 16'h63FC;
        cjk_rom[ 69] = 16'h3040;
        cjk_rom[ 70] = 16'h17FE;
        cjk_rom[ 71] = 16'h0204;
        cjk_rom[ 72] = 16'h13FC;
        cjk_rom[ 73] = 16'h3204;
        cjk_rom[ 74] = 16'h33FC;
        cjk_rom[ 75] = 16'h2204;
        cjk_rom[ 76] = 16'h223C;
        cjk_rom[ 77] = 16'h0000;
        cjk_rom[ 78] = 16'h0000;
        cjk_rom[ 79] = 16'h0000;
        cjk_rom[ 80] = 16'h0000;
        cjk_rom[ 81] = 16'h0000;
        cjk_rom[ 82] = 16'h010C;
        cjk_rom[ 83] = 16'h3938;
        cjk_rom[ 84] = 16'h2920;
        cjk_rom[ 85] = 16'h2FE0;
        cjk_rom[ 86] = 16'h293E;
        cjk_rom[ 87] = 16'h3BAC;
        cjk_rom[ 88] = 16'h2B6C;
        cjk_rom[ 89] = 16'h2F2C;
        cjk_rom[ 90] = 16'h2D2C;
        cjk_rom[ 91] = 16'h396C;
        cjk_rom[ 92] = 16'h294C;
        cjk_rom[ 93] = 16'h014C;
        cjk_rom[ 94] = 16'h0000;
        cjk_rom[ 95] = 16'h0000;
        cjk_rom[ 96] = 16'h0000;
        cjk_rom[ 97] = 16'h0000;
        cjk_rom[ 98] = 16'h3FFC;
        cjk_rom[ 99] = 16'h0430;
        cjk_rom[100] = 16'h0420;
        cjk_rom[101] = 16'h0420;
        cjk_rom[102] = 16'h0420;
        cjk_rom[103] = 16'h7FFE;
        cjk_rom[104] = 16'h0430;
        cjk_rom[105] = 16'h0420;
        cjk_rom[106] = 16'h0C20;
        cjk_rom[107] = 16'h0820;
        cjk_rom[108] = 16'h1820;
        cjk_rom[109] = 16'h2020;
        cjk_rom[110] = 16'h0000;
        cjk_rom[111] = 16'h0000;
        cjk_rom[112] = 16'h0000;
        cjk_rom[113] = 16'h0000;
        cjk_rom[114] = 16'h0830;
        cjk_rom[115] = 16'h0C20;
        cjk_rom[116] = 16'h0460;
        cjk_rom[117] = 16'h3FFC;
        cjk_rom[118] = 16'h0180;
        cjk_rom[119] = 16'h0180;
        cjk_rom[120] = 16'h7FFE;
        cjk_rom[121] = 16'h0180;
        cjk_rom[122] = 16'h03C0;
        cjk_rom[123] = 16'h0660;
        cjk_rom[124] = 16'h1C38;
        cjk_rom[125] = 16'h300C;
        cjk_rom[126] = 16'h0000;
        cjk_rom[127] = 16'h0000;
        cjk_rom[128] = 16'h0000;
        cjk_rom[129] = 16'h0000;
        cjk_rom[130] = 16'h0000;
        cjk_rom[131] = 16'h03C0;
        cjk_rom[132] = 16'h0660;
        cjk_rom[133] = 16'h0620;
        cjk_rom[134] = 16'h0470;
        cjk_rom[135] = 16'h04F0;
        cjk_rom[136] = 16'h0730;
        cjk_rom[137] = 16'h0630;
        cjk_rom[138] = 16'h0620;
        cjk_rom[139] = 16'h0660;
        cjk_rom[140] = 16'h03C0;
        cjk_rom[141] = 16'h0000;
        cjk_rom[142] = 16'h0000;
        cjk_rom[143] = 16'h0000;
        cjk_rom[144] = 16'h0000;
        cjk_rom[145] = 16'h0000;
        cjk_rom[146] = 16'h0000;
        cjk_rom[147] = 16'h0180;
        cjk_rom[148] = 16'h0380;
        cjk_rom[149] = 16'h0480;
        cjk_rom[150] = 16'h0080;
        cjk_rom[151] = 16'h0080;
        cjk_rom[152] = 16'h0080;
        cjk_rom[153] = 16'h0080;
        cjk_rom[154] = 16'h0080;
        cjk_rom[155] = 16'h0080;
        cjk_rom[156] = 16'h07E0;
        cjk_rom[157] = 16'h0000;
        cjk_rom[158] = 16'h0000;
        cjk_rom[159] = 16'h0000;
        cjk_rom[160] = 16'h0000;
        cjk_rom[161] = 16'h0000;
        cjk_rom[162] = 16'h0000;
        cjk_rom[163] = 16'h03C0;
        cjk_rom[164] = 16'h0660;
        cjk_rom[165] = 16'h0060;
        cjk_rom[166] = 16'h0060;
        cjk_rom[167] = 16'h0060;
        cjk_rom[168] = 16'h00C0;
        cjk_rom[169] = 16'h0180;
        cjk_rom[170] = 16'h0300;
        cjk_rom[171] = 16'h0600;
        cjk_rom[172] = 16'h07F0;
        cjk_rom[173] = 16'h0000;
        cjk_rom[174] = 16'h0000;
        cjk_rom[175] = 16'h0000;
        cjk_rom[176] = 16'h0000;
        cjk_rom[177] = 16'h0000;
        cjk_rom[178] = 16'h0000;
        cjk_rom[179] = 16'h07C0;
        cjk_rom[180] = 16'h0060;
        cjk_rom[181] = 16'h0060;
        cjk_rom[182] = 16'h0040;
        cjk_rom[183] = 16'h03C0;
        cjk_rom[184] = 16'h0060;
        cjk_rom[185] = 16'h0020;
        cjk_rom[186] = 16'h0020;
        cjk_rom[187] = 16'h0060;
        cjk_rom[188] = 16'h07C0;
        cjk_rom[189] = 16'h0000;
        cjk_rom[190] = 16'h0000;
        cjk_rom[191] = 16'h0000;
    end

    wire [7:0]  cjk_addr = {gid, pgrow};
    reg  [15:0] cjk_row;
    always @(posedge clk) cjk_row <= cjk_rom[cjk_addr];

    // ---------- 输出流水线（对齐 ROM 1 拍）----------
    reg [23:0] rgb_d;
    reg        de_d, text_d, box_d, osd_d;
    reg        dbg_d, dbg_val_d;
    reg [2:0]  col_d;
    reg        panel_d, panel_border_d, panel_blank_d, panel_gl_d;
    reg        panel_row_ok_d;
    reg [3:0]  pbit_d;
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            rgb_d <= 24'd0; de_d <= 1'b0; text_d <= 1'b0; box_d <= 1'b0;
            osd_d <= 1'b0; col_d <= 3'd0; dbg_d <= 1'b0; dbg_val_d <= 1'b0;
            panel_d <= 1'b0; panel_border_d <= 1'b0; panel_blank_d <= 1'b0;
            panel_row_ok_d <= 1'b1;
            panel_gl_d <= 1'b0; pbit_d <= 4'd0;
        end else begin
            rgb_d          <= rgb_in;
            de_d           <= de;
            text_d         <= inside_text;
            box_d          <= inside_box;
            osd_d          <= osd_en;
            col_d          <= char_col;
            dbg_d          <= in_dbg;
            dbg_val_d      <= dbg_val;
            panel_d        <= in_panel;
            panel_border_d <= pborder;
            panel_blank_d  <= pblank;
            panel_gl_d     <= in_glyph;
            panel_row_ok_d <= panel_row_ok;
            pbit_d         <= pbit;
        end
    end

    wire glyph_on     = glyph_byte[7 - col_d];
    wire cjk_on       = cjk_row[15 - pbit_d];

    assign rgb_out = (de_d && dbg_d)                              ? (dbg_val_d ? 24'hFF0000 : 24'h00FF00) :
                     (de_d && panel_d && panel_border_d)           ? 24'hFFFFFF :
                     (de_d && panel_d)                             ? ((panel_gl_d && !panel_blank_d && panel_row_ok_d && cjk_on)
                                                                      ? 24'hFFFFFF : 24'h000000) :
                     (osd_d && de_d && text_d && glyph_on)         ? 24'hFFFFFF :
                     (osd_d && de_d && box_d)                      ? 24'h000000 :
                                                                     rgb_d;
endmodule
