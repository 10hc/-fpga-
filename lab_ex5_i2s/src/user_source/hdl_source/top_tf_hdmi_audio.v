
module top(
    input                       clk,
    input                       rst_n,
    input                       key1,           // 手动下一张
    input                       key2,           // 自动播放 开/关
    input                       key3,           // OSD 文字开关
    input                       swkey1, swkey2, swkey3, swkey4,   // 拨码：亮度/对比度

    output [5:0]                seg_sel,
    output [7:0]                seg_data,

    // HDMI TMDS
    output                      HDMI_CLK_P,
    output                      HDMI_D2_P,
    output                      HDMI_D1_P,
    output                      HDMI_D0_P,

    // HDMI DDC
    output                      HDMI_DDC_SCL,
    inout                       HDMI_DDC_SDA,

    // TF card SPI
    output                      sd_ncs,
    output                      sd_dclk,
    output                      sd_mosi,
    input                       sd_miso
);

parameter MEM_DATA_BITS = 32;
parameter ADDR_BITS     = 21;
parameter BUSRT_BITS    = 10;
parameter FRAME_PIXELS  = 24'd307200;   // 640*480
parameter BUF0_ADDR     = 24'd0;
parameter BUF1_ADDR     = FRAME_PIXELS;

wire Sdr_init_done;
wire Sdr_init_ref_vld;
wire Sdr_busy;

wire sd_card_clk;
wire ext_mem_clk;
wire ext_mem_clk_sft;
wire video_clk;
wire hdmi_5x_clk;

wire hs;
wire vs;
wire de;

wire [23:0] vout_data_raw;
wire [23:0] vout_data;
wire        display_valid;

wire [3:0]  state_code;
wire [6:0]  seg_data_0;

wire        video_read_req;
wire        video_read_req_ack;
wire        video_read_en;
wire [31:0] video_read_data;

wire        sd_card_write_en;
wire [31:0] sd_card_write_data;
wire        sd_card_write_req;
wire        sd_card_write_req_ack;
wire        frame_write_finish;
reg         frame_write_toggle_mem;

wire [1:0]  write_buf_idx;
wire [1:0]  disp_buf_idx;

wire App_rd_en;
wire [ADDR_BITS-1:0] App_rd_addr;
wire Sdr_rd_en;
wire [MEM_DATA_BITS-1:0] Sdr_rd_dout;
wire App_wr_en;
wire [ADDR_BITS-1:0] App_wr_addr;
wire [MEM_DATA_BITS-1:0] App_wr_din;
wire [3:0] App_wr_dm;

wire hs_0;
wire vs_0;
wire de_0;

// HDMI 1.4b 音频发射相关
wire        audio_pll_lock;
wire        audio_mclk;
wire        audio_i2s_bclk;
wire        audio_i2s_lrck;
wire        audio_i2s_dout;
wire        audio_valid;
wire [23:0] audio_left_data;
wire [23:0] audio_right_data;
// 内部测试音（WAV 未播放时的兜底音源）
wire        tone_valid;
wire [23:0] tone_left_data;
wire [23:0] tone_right_data;
// TF 卡 WAV 播放
wire        wav_valid;
wire [23:0] wav_left_data;
wire [23:0] wav_right_data;
wire        wav_playing;
wire        wav_sec_read;
wire [31:0] wav_sec_read_addr;
wire [7:0]  wav_sec_read_data;
wire        wav_sec_read_data_valid;
wire        wav_sec_read_end;
wire        acr_valid;
wire [19:0] acr_cts;
wire [19:0] acr_n;

wire        axis_s_user;
wire        axis_s_valid;
wire        axis_s_last;
wire [23:0] axis_s_data;
wire        axis_s_ready;

wire        edid_trig;
wire        edid_valid;
wire [7:0]  edid_data;

wire [9:0]  tmds_ch0_data;
wire [9:0]  tmds_ch1_data;
wire [9:0]  tmds_ch2_data;
wire [9:0]  tmds_clk_data;

// 统一复位：TF 图像链路 + HDMI 音频链路
wire rst_all;
assign rst_all = ~rst_n | ~audio_pll_lock;

// 保持你原来的 TF / SDRAM / video 时钟
sys_pll sys_pll_m0(
    .refclk     (clk),
    .clk0_out   (sd_card_clk),
    .clk1_out   (ext_mem_clk),
    .clk2_out   (ext_mem_clk_sft),
    .reset      (1'b0)
);

video_pll video_pll_m0(
    .refclk     (clk),
    .clk0_out   (video_clk),
    .clk1_out   (hdmi_5x_clk),
    .reset      (1'b0)
);

// 复用你 I2S->HDMI 工程里的 PLL，只取 12.288MHz 音频主时钟
PLL_HDMI_AUDIO u_audio_pll(
    .refclk     (clk),
    .reset      (1'b0),
    .extlock    (audio_pll_lock),
    .clk0_out   (),
    .clk1_out   (),
    .clk2_out   (audio_mclk)
);

// mem_clk 域把 write_finish 单拍转成 toggle，供 sd_card_clk 域可靠同步
always @(posedge ext_mem_clk or posedge rst_all) begin
    if (rst_all)
        frame_write_toggle_mem <= 1'b0;
    else if (frame_write_finish)
        frame_write_toggle_mem <= ~frame_write_toggle_mem;
end

// ===================== 视频播放模式（KEY2 长按 >1.5s 切换）=====================
wire k2_short, k2_long;
key_short_long #(
    .CLK_FREQ_HZ (25_000_000),
    .LONG_MS     (1500)
) u_key2_video (
    .clk         (video_clk),
    .rst         (rst_all),
    .button      (key1),                 // 板上 KEY_2
    .short_pulse (k2_short),
    .long_pulse  (k2_long)
);

reg video_mode;
always @(posedge video_clk or posedge rst_all) begin
    if (rst_all)      video_mode <= 1'b0;
    else if (k2_long) video_mode <= ~video_mode;
end

// 视频模式下屏蔽图像路径的按键（高=未按下），避免它去抢 SD 卡
wire key_next_img = video_mode ? 1'b1 : key1;
wire key_auto_img = video_mode ? 1'b1 : key2;

// ---- 图像路径（sd_card_bmp）的原始信号 ----
wire        img_display_valid;
wire [1:0]  img_write_buf_idx;
wire [1:0]  img_disp_buf_idx;
wire        img_write_req;
wire        img_write_en;
wire [31:0] img_write_data;

// ---- 扇区读通道：sd_card_bmp 的那一对端口 ----
wire [7:0]  sec_from_bmp_data;
wire        sec_from_bmp_data_valid;
wire        sec_from_bmp_end;
wire        sec_to_bmp_read;
wire [31:0] sec_to_bmp_read_addr;

// ---- 视频播放器 ----
wire        vid_sec_read;
wire [31:0] vid_sec_read_addr;
wire [7:0]  vid_sec_read_data;
wire        vid_sec_read_data_valid;
wire        vid_sec_read_end;
wire        vid_wr_req;
wire        vid_wr_en;
wire [31:0] vid_wr_data;
wire        vid_display_valid;
wire [1:0]  vid_disp_buf_idx;
wire [1:0]  vid_write_buf_idx;
wire        vid_active;

video_player #(
    .CLK_FREQ_HZ (100_000_000)
) u_video_player (
    .clk                 (sd_card_clk),
    .rst                 (rst_all),
    .enable              (video_mode),
    .sec_read            (vid_sec_read),
    .sec_read_addr       (vid_sec_read_addr),
    .sec_read_data       (vid_sec_read_data),
    .sec_read_data_valid (vid_sec_read_data_valid),
    .sec_read_end        (vid_sec_read_end),
    .wr_req              (vid_wr_req),
    .wr_req_ack          (sd_card_write_req_ack),
    .wr_en               (vid_wr_en),
    .wr_data             (vid_wr_data),
    .wr_finish           (frame_write_toggle_mem),
    .display_valid       (vid_display_valid),
    .disp_buf_idx        (vid_disp_buf_idx),
    .write_buf_idx       (vid_write_buf_idx),
    .active              (vid_active)
);

// ---- 帧写通道与显示控制：视频模式接管，否则用图像路径 ----
assign display_valid      = video_mode ? vid_display_valid : img_display_valid;
assign disp_buf_idx       = video_mode ? vid_disp_buf_idx  : img_disp_buf_idx;
assign write_buf_idx      = video_mode ? vid_write_buf_idx : img_write_buf_idx;
assign sd_card_write_req  = video_mode ? vid_wr_req        : img_write_req;
assign sd_card_write_en   = video_mode ? vid_wr_en         : img_write_en;
assign sd_card_write_data = video_mode ? vid_wr_data       : img_write_data;

// ---- 扇区读通道：视频模式独占（音乐自动让路）；否则给 WAV 播放器 ----
assign sec_to_bmp_read      = video_mode ? vid_sec_read      : wav_sec_read;
assign sec_to_bmp_read_addr = video_mode ? vid_sec_read_addr : wav_sec_read_addr;

assign vid_sec_read_data       = sec_from_bmp_data;
assign vid_sec_read_data_valid = sec_from_bmp_data_valid &  video_mode;
assign vid_sec_read_end        = sec_from_bmp_end        &  video_mode;

assign wav_sec_read_data       = sec_from_bmp_data;
assign wav_sec_read_data_valid = sec_from_bmp_data_valid & ~video_mode;
assign wav_sec_read_end        = sec_from_bmp_end        & ~video_mode;

// ===================== TF 多图扫描与缓存（双缓冲） =====================
sd_card_bmp #(
    .CLK_FREQ_HZ       (100_000_000),
    .SCAN_START_SECTOR (32'd0),
    .SCAN_MAX_SECTOR   (32'd131071),     // 扫描范围 64MB。注意：不能改小！
                                         // 大容量卡的 FAT32 元数据可能占满前 16MB，
                                         // 图片数据会落在 16MB 之后（实测某卡在 16.6MB 处）
    .SCAN_TARGET_COUNT (3'd4)
) sd_card_bmp_m0(
    .clk               (sd_card_clk),
    .rst               (rst_all),
    .key_next          (key_next_img),
    .key_auto          (key_auto_img),
    .state_code        (state_code),
    .bmp_width         (16'd640),
    .bmp_height        (16'd480),
    .display_valid     (img_display_valid),

    .write_finish_toggle(frame_write_toggle_mem),
    .write_buf_idx     (img_write_buf_idx),
    .disp_buf_idx      (img_disp_buf_idx),

    .write_req         (img_write_req),
    .write_req_ack     (sd_card_write_req_ack),
    .write_en          (img_write_en),
    .write_data        (img_write_data),
    .SD_nCS            (sd_ncs),
    .SD_DCLK           (sd_dclk),
    .SD_MOSI           (sd_mosi),
    .SD_MISO           (sd_miso),

    // ===== 扇区读通道（视频模式由视频播放器独占，否则给 WAV）=====
    .wav_sec_read            (sec_to_bmp_read),
    .wav_sec_read_addr       (sec_to_bmp_read_addr),
    .wav_sec_read_data       (sec_from_bmp_data),
    .wav_sec_read_data_valid (sec_from_bmp_data_valid),
    .wav_sec_read_end        (sec_from_bmp_end)
);

seg_decoder seg_decoder_m0(
    .bin_data          (state_code),
    .seg_data          (seg_data_0)
);

seg_scan seg_scan_m0(
    .clk               (clk),
    .rst_n             (rst_n),
    .seg_sel           (seg_sel),
    .seg_data          (seg_data),
    .seg_data_0        ({1'b1,7'b1111_111}),
    .seg_data_1        ({1'b1,7'b1111_111}),
    .seg_data_2        ({1'b1,7'b1111_111}),
    .seg_data_3        ({1'b1,7'b1111_111}),
    .seg_data_4        ({1'b1,7'b1111_111}),
    .seg_data_5        ({1'b1,seg_data_0})
);

// ===================== 原图像时序与帧缓存 =====================
video_timing_data video_timing_data_m0(
    .video_clk         (video_clk),
    .rst               (rst_all),
    .read_req          (video_read_req),
    .read_req_ack      (video_read_req_ack),
    .hs                (hs_0),
    .vs                (vs_0),
    .de                (de_0)
);

video_delay video_delay_m0(
    .video_clk         (video_clk),
    .rst               (rst_all),
    .read_en           (video_read_en),
    .read_data         (video_read_data[31:8]),
    .hs                (hs_0),
    .vs                (vs_0),
    .de                (de_0),
    .hs_r              (hs),
    .vs_r              (vs),
    .de_r              (de),
    .vout_data         (vout_data_raw)
);

// 首图提交前黑屏；提交后一直显示当前显示缓冲区内容
// 未出图时用背景色表示状态（诊断用）
// 深蓝=视频模式但还没有画面   红=SD未初始化  绿=空闲(未找到图)  蓝=扫描/读BMP头
// 紫=等待写入  青=加载数据中  灰=其它
wire [23:0] diag_color =
    video_mode            ? 24'h000080 :
    (state_code == 4'd0) ? 24'hFF0000 :
    (state_code == 4'd1) ? 24'h00FF00 :
    (state_code == 4'd2) ? 24'h0080FF :
    (state_code == 4'd3) ? 24'h8000FF :
    (state_code == 4'd4) ? 24'h00FFFF :
                           24'h404040;
assign vout_data = display_valid ? vout_data_raw : diag_color;

// ===================== 花样转场：滑动 / 栅格 / 淡入淡出 =====================
// disp_buf_idx 同步到 mem_clk 域，用于检测"切图脉冲"
reg [1:0] disp_sync0, disp_sync1, disp_sync2;
always @(posedge ext_mem_clk or posedge rst_all) begin
    if (rst_all) begin
        disp_sync0 <= 2'd0;
        disp_sync1 <= 2'd0;
        disp_sync2 <= 2'd0;
    end else begin
        disp_sync0 <= disp_buf_idx;
        disp_sync1 <= disp_sync0;
        disp_sync2 <= disp_sync1;
    end
end
wire switch_pulse = (disp_sync2 != disp_sync1);

wire        read_finish;
wire        trans_en;
wire [1:0]  trans_effect;
wire [9:0]  trans_offset;
wire [8:0]  fade_factor;

// 每次切图自动轮换效果：滑动 → 栅格 → 淡入淡出 → 滑动 …
// 以"每帧读完"为节拍推进，帧内参数恒定 → 画面平滑
trans_ctrl u_trans_ctrl(
    .clk          (ext_mem_clk),
    .rst          (rst_all),
    .switch_pulse (switch_pulse),
    .frame_done   (read_finish),
    .trans_effect (trans_effect),
    .trans_en     (trans_en),
    .trans_offset (trans_offset),
    .fade_factor  (fade_factor)
);

// 转场时读的"旧/新"缓冲基址（disp_sync2 = 当前显示 = 新图）
wire [20:0] trans_new_base = (disp_sync2 == 2'd0) ? BUF0_ADDR[20:0] : BUF1_ADDR[20:0];
wire [20:0] trans_old_base = (disp_sync2 == 2'd0) ? BUF1_ADDR[20:0] : BUF0_ADDR[20:0];

// ===================== 扩展①/④：亮度对比度 + OSD 文字 =====================
// KEY3：短按 → 亮度循环切换；长按（>1s）→ 对比度循环切换
wire k3_short, k3_long;
key_short_long #(
    .CLK_FREQ_HZ (25_000_000),
    .LONG_MS     (1000)
) u_key3_adj (
    .clk         (video_clk),
    .rst         (rst_all),
    .button      (key3),
    .short_pulse (k3_short),
    .long_pulse  (k3_long)
);

reg [1:0] bright_sel;
reg [1:0] contrast_sel;
always @(posedge video_clk or posedge rst_all) begin
    if (rst_all) begin
        bright_sel   <= 2'd3;      // 3 = 默认档（正常）
        contrast_sel <= 2'd3;
    end else begin
        if (k3_short) bright_sel   <= bright_sel   + 2'd1;
        if (k3_long)  contrast_sel <= contrast_sel + 2'd1;
    end
end

// ===================== 按键诊断：屏幕顶部 8 个方块 =====================
// 目的：判断"按下按键，FPGA 到底有没有收到"。
// 从左到右分别对应 key_dbg[7]..key_dbg[0]：
//   [7] display_valid（图像已就绪）  [6] video_mode（视频模式）
//   [5] key2(板上KEY_4)短按已检测    [4] key1(板上KEY_2)长按已检测
//   [3] key1 短按已检测              [2] key3 实时电平（按下=1）
//   [1] key2 实时电平                [0] key1 实时电平
// 绿色 = 0，红色 = 1。按住不放时对应方块应变红。
reg k1_lvl, k2_lvl, k3_lvl;
always @(posedge video_clk or posedge rst_all) begin
    if (rst_all) begin
        k1_lvl <= 1'b1; k2_lvl <= 1'b1; k3_lvl <= 1'b1;
    end else begin
        k1_lvl <= key1; k2_lvl <= key2; k3_lvl <= key3;
    end
end

// key2（板上 KEY_4）原本直接给图像路径，这里额外加一个检测器用于显示
wire k4_short, k4_long;
key_short_long #(
    .CLK_FREQ_HZ (25_000_000),
    .LONG_MS     (1000)
) u_key4_det (
    .clk         (video_clk),
    .rst         (rst_all),
    .button      (key2),
    .short_pulse (k4_short),
    .long_pulse  (k4_long)
);

// 事件保持约 0.75 秒（25MHz × 0.75s = 18_750_000 拍），肉眼可见
localparam [24:0] DBG_HOLD = 25'd18_750_000;
reg [24:0] h_k1s, h_k1l, h_k4s, h_k4l;
always @(posedge video_clk or posedge rst_all) begin
    if (rst_all) begin
        h_k1s <= 25'd0; h_k1l <= 25'd0; h_k4s <= 25'd0; h_k4l <= 25'd0;
    end else begin
        h_k1s <= k2_short  ? DBG_HOLD : (h_k1s != 25'd0 ? h_k1s - 25'd1 : 25'd0);
        h_k1l <= k2_long   ? DBG_HOLD : (h_k1l != 25'd0 ? h_k1l - 25'd1 : 25'd0);
        h_k4s <= k4_short  ? DBG_HOLD : (h_k4s != 25'd0 ? h_k4s - 25'd1 : 25'd0);
        h_k4l <= k4_long   ? DBG_HOLD : (h_k4l != 25'd0 ? h_k4l - 25'd1 : 25'd0);
    end
end

wire [7:0] key_dbg = { display_valid, video_mode,
                       (h_k4s != 25'd0), (h_k1l != 25'd0), (h_k1s != 25'd0),
                       ~k3_lvl, ~k2_lvl, ~k1_lvl };

wire [23:0] vout_data_adj;
video_adjust u_video_adjust(
    .clk         (video_clk),
    .rst         (rst_all),
    .de          (de),
    .sw_bright   (bright_sel),
    .sw_contrast (contrast_sel),
    .fade        (fade_factor),      // 淡入淡出已折叠进本模块（像素路径仅 1 个乘法器）
    .rgb_in      (vout_data),
    .rgb_out     (vout_data_adj)
);

wire osd_on = 1'b1;   // OSD 文字常开

wire [23:0] vout_data_osd;
osd_overlay u_osd_overlay(
    .clk     (video_clk),
    .rst     (rst_all),
    .de      (de),
    .vs      (vs),
    .osd_en  (osd_on),
    .rgb_in  (vout_data_adj),
    .key_dbg (key_dbg),
    .rgb_out (vout_data_osd)
);

// OSD 内部有 1 拍流水线（字库 ROM），de/vs 同步延迟 1 拍对齐 AXIS
reg de_d1, vs_d1;
always @(posedge video_clk or posedge rst_all) begin
    if (rst_all) begin
        de_d1 <= 1'b0;
        vs_d1 <= 1'b0;
    end else begin
        de_d1 <= de;
        vs_d1 <= vs;
    end
end

frame_read_write #(
    .WRITE_V_FLIP     (1),
    .FRAME_WIDTH      (640),
    .FRAME_HEIGHT     (480)
) frame_read_write_m0(
    .mem_clk           (ext_mem_clk),
    .rst               (rst_all),
    .Sdr_init_done     (Sdr_init_done),
    .Sdr_init_ref_vld  (Sdr_init_ref_vld),
    .Sdr_busy          (Sdr_busy),

    .App_rd_en         (App_rd_en),
    .App_rd_addr       (App_rd_addr),
    .Sdr_rd_en         (Sdr_rd_en),
    .Sdr_rd_dout       (Sdr_rd_dout),

    .read_clk          (video_clk),
    .read_req          (video_read_req),
    .read_req_ack      (video_read_req_ack),
    .read_finish       (read_finish),
    .read_addr_0       (BUF0_ADDR),
    .read_addr_1       (BUF1_ADDR),
    .read_addr_2       (24'd0),
    .read_addr_3       (24'd0),
    .read_addr_index   (disp_buf_idx),
    .read_len          (FRAME_PIXELS),
    .read_en           (video_read_en),
    .read_data         (video_read_data),

    // ===== 花样转场 =====
    .trans_en          (trans_en),
    .trans_effect      (trans_effect),
    .trans_offset      (trans_offset),
    .trans_old_base    (trans_old_base),
    .trans_new_base    (trans_new_base),
    .dbg_rd_state      (),
    .dbg_rd_wrbusy     (),
    .dbg_rd_req2       (),
    .dbg_rd_init       (),

    .App_wr_en         (App_wr_en),
    .App_wr_addr       (App_wr_addr),
    .App_wr_din        (App_wr_din),
    .App_wr_dm         (App_wr_dm),

    .write_clk         (sd_card_clk),
    .write_req         (sd_card_write_req),
    .write_req_ack     (sd_card_write_req_ack),
    .write_finish      (frame_write_finish),
    .write_addr_0      (BUF0_ADDR),
    .write_addr_1      (BUF1_ADDR),
    .write_addr_2      (24'd0),
    .write_addr_3      (24'd0),
    .write_addr_index  (write_buf_idx),
    .write_len         (FRAME_PIXELS),
    .write_en          (sd_card_write_en),
    .write_data        (sd_card_write_data)
);

sdram U3(
    .Clk               (ext_mem_clk),
    .Clk_sft           (ext_mem_clk_sft),
    .Rst               (rst_all),
    .Sdr_init_done     (Sdr_init_done),
    .Sdr_init_ref_vld  (Sdr_init_ref_vld),
    .Sdr_busy          (Sdr_busy),
    .App_wr_en         (App_wr_en),
    .App_wr_addr       (App_wr_addr),
    .App_wr_dm         (App_wr_dm),
    .App_wr_din        (App_wr_din),
    .App_rd_en         (App_rd_en),
    .App_rd_addr       (App_rd_addr),
    .Sdr_rd_en         (Sdr_rd_en),
    .Sdr_rd_dout       (Sdr_rd_dout)
);

// ===================== 音频：内部 I2S 测试音 =====================
hdmi_audio_tone_i2s_64fs #(
    .PHASE_INC (32'd39370534),
    .AMP       (24'sd2000000)
) u_hdmi_audio_tone_i2s_64fs (
    .I_mclk      (audio_mclk),
    .I_rst       (rst_all),
    .O_i2s_BCLK  (audio_i2s_bclk),
    .O_i2s_LRCK  (audio_i2s_lrck),
    .O_i2s_DOUT  (audio_i2s_dout)
);

I2S_receiver u_I2S_receiver(
    .I_clk              (video_clk),
    .I_rst              (rst_all),
    .I_i2s_BCLK         (audio_i2s_bclk),
    .I_i2s_LRCK         (audio_i2s_lrck),
    .I_i2s_DOUT         (audio_i2s_dout),
    .O_audio_valid      (tone_valid),
    .O_audio_left_data  (tone_left_data),
    .O_audio_right_data (tone_right_data)
);

// ===================== 音频源：TF 卡 WAV 优先，找不到则用测试音 =====================
// 注意：sd_card_clk 由 sys_pll 的 clk0 提供，实测配置为 100MHz（不是 50MHz！）
//       48kHz 节拍按这个频率计算，写错会导致采样率翻倍、声音失真
wav_audio #(
    .SD_CLK_HZ   (100_000_000),
    .VID_CLK_HZ  (25_000_000),
    .SAMPLE_RATE (48_000)
) u_wav_audio (
    .sd_clk              (sd_card_clk),
    .sd_rst              (rst_all),
    .vid_clk             (video_clk),
    .vid_rst             (rst_all),
    .enable              (display_valid & ~video_mode),   // 视频模式时音频让路
    .sec_read            (wav_sec_read),
    .sec_read_addr       (wav_sec_read_addr),
    .sec_read_data       (wav_sec_read_data),
    .sec_read_data_valid (wav_sec_read_data_valid),
    .sec_read_end        (wav_sec_read_end),
    .aud_valid           (wav_valid),
    .aud_left            (wav_left_data),
    .aud_right           (wav_right_data),
    .playing             (wav_playing)
);

assign audio_valid      = wav_playing ? wav_valid      : tone_valid;
assign audio_left_data  = wav_playing ? wav_left_data  : tone_left_data;
assign audio_right_data = wav_playing ? wav_right_data : tone_right_data;

audio_arc_calculate #(
    .ACR_N         (6144)
) u_audio_arc_calculate (
    .I_clk         (video_clk),
    .I_rst         (rst_all),
    .I_audio_valid (audio_valid),
    .O_acr_valid   (acr_valid),
    .O_acr_cts     (acr_cts),
    .O_acr_n       (acr_n)
);

// ===================== RGB/DE 转 AXIS 视频 =====================
video_rgb_to_axis_640x480 u_video_rgb_to_axis_640x480(
    .I_clk         (video_clk),
    .I_rst         (rst_all),
    .I_vs          (vs_d1),
    .I_de          (de_d1),
    .I_rgb         (vout_data_osd),
    .O_video_user  (axis_s_user),
    .O_video_valid (axis_s_valid),
    .O_video_last  (axis_s_last),
    .O_video_data  (axis_s_data)
);

// 上电后自动打一拍，触发一次 EDID 读取
startup_pulse #(
    .CNT_MAX(20'd100000)
) u_startup_pulse (
    .I_clk   (video_clk),
    .I_rst   (rst_all),
    .O_pulse (edid_trig)
);

// ===================== 带音频的 HDMI 1.4b 发射 =====================
hdmi_1_4b_transmitter_core_wrapper #(
    .DEVICE                 ( "EG"       ),
    .HTOTAL                 ( 800        ),
    .HSA                    ( 96         ),
    .HFP                    ( 16         ),
    .HBP                    ( 48         ),
    .HACTIVE                ( 640        ),
    .VTOTAL                 ( 525        ),
    .VSA                    ( 2          ),
    .VFP                    ( 10         ),
    .VBP                    ( 33         ),
    .VACTIVE                ( 480        ),
    .VIDEO_VIC              ( 1          ),
    .VIDEO_TPG              ( "Disable"  ),
    .VIDEO_FORMAT           ( "RGB"      ),
    .AUDIO_SAMPLE_RATE      ( "48K"      ),
    .IIC_SCL_DIV            ( 250        )
) u_hdmi_1_4b_transmitter_core_wrapper(
    .I_pixel_clk        (video_clk),
    .I_rst              (rst_all),
    .I_edid_read_trig   (edid_trig),
    .O_edid_read_valid  (edid_valid),
    .O_edid_read_data   (edid_data),

    .I_axis_s_user      (axis_s_user),
    .I_axis_s_valid     (axis_s_valid),
    .I_axis_s_last      (axis_s_last),
    .I_axis_s_data      (axis_s_data),
    .O_axis_s_ready     (axis_s_ready),

    .I_audio_valid      (audio_valid),
    .I_audio_left_data  (audio_left_data),
    .I_audio_right_data (audio_right_data),
    .I_acr_valid        (acr_valid),
    .I_acr_cts          (acr_cts),
    .I_acr_n            (acr_n),

    .O_video_locked     (),
    .O_ddc_scl          (HDMI_DDC_SCL),
    .IO_ddc_sda         (HDMI_DDC_SDA),

    .O_ch0_tmds_data    (tmds_ch0_data),
    .O_ch1_tmds_data    (tmds_ch1_data),
    .O_ch2_tmds_data    (tmds_ch2_data),
    .O_clk_tmds_data    (tmds_clk_data)
);

hdmi_phy_wrapper #(
    .DEVICE ( "EG" )
) u_hdmi2phy_wrapper(
    .I_pixel_clk        (video_clk),
    .I_serial_clk       (hdmi_5x_clk),
    .I_rst              (rst_all),
    .I_tmds_channel_0   (tmds_ch0_data),
    .I_tmds_channel_1   (tmds_ch1_data),
    .I_tmds_channel_2   (tmds_ch2_data),
    .I_tmds_channel_clk (tmds_clk_data),
    .O_tmds_ch0_p       (HDMI_D0_P),
    .O_tmds_ch1_p       (HDMI_D1_P),
    .O_tmds_ch2_p       (HDMI_D2_P),
    .O_tmds_clk_p       (HDMI_CLK_P)
);

endmodule
