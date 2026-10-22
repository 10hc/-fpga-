// ============================================================
// osd_overlay.v —— 底部字幕条 + 横向滚动（跑马灯）版
//
// ● 黑底白字字幕条：位于屏幕底部（BAR_Y 可调）
// ● 文字每帧向左滚动 SCROLL_STEP 像素，循环播放
// ● 字库：0x20~0x7E 完整 ASCII，8×16 点阵（font_rom + font8x16.mif）
// ● 改文字：只改下面 OSD_TEXT 一行 —— 必须恰好 TEXT_LEN 个字符，
//   不足用空格补齐（TEXT_LEN 默认 64，即 64×8=512 像素宽的文本带）
// ● 内部 1 拍流水线：顶层需把送 AXIS 的 de/vs 延迟 1 拍对齐
// ============================================================
module osd_overlay #(
    parameter integer CHAR_W      = 8,     // 字符宽
    parameter integer CHAR_H      = 16,    // 字符高
    parameter integer TEXT_LEN    = 64,    // 文本字符数（需为 2 的幂）
    parameter integer BAR_Y       = 456,   // 字幕条文字顶部 Y（480-24）
    parameter integer SCROLL_STEP = 2      // 每帧滚动像素
)(
    input  wire        clk,
    input  wire        rst,
    input  wire        de,
    input  wire        vs,
    input  wire        osd_en,
    input  wire [23:0] rgb_in,
    output wire [23:0] rgb_out
);
    // ==================== 文本表：改这一行即可 ====================
    // 必须恰好 TEXT_LEN(64) 个字符；不足用空格补齐
    localparam [TEXT_LEN*8-1:0] OSD_TEXT =
        "ANLU 2026  SMART HOME  FPGA 2026                                ";

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

    // ---------- 输出流水线（对齐 ROM 1 拍）----------
    reg [23:0] rgb_d;
    reg        de_d, text_d, box_d, osd_d;
    reg [2:0]  col_d;
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            rgb_d <= 24'd0; de_d <= 1'b0; text_d <= 1'b0; box_d <= 1'b0;
            osd_d <= 1'b0; col_d <= 3'd0;
        end else begin
            rgb_d  <= rgb_in;
            de_d   <= de;
            text_d <= inside_text;
            box_d  <= inside_box;
            osd_d  <= osd_en;
            col_d  <= char_col;
        end
    end

    wire glyph_on = glyph_byte[7 - col_d];

    assign rgb_out = (osd_d && de_d && text_d && glyph_on) ? 24'hFFFFFF :
                     (osd_d && de_d && box_d)              ? 24'h000000 :
                                                             rgb_d;
endmodule
