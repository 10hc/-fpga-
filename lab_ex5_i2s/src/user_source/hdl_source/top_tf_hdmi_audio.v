
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

// ---------------------------------------------------------------------------
// 【上电按键屏蔽】复位后 2 秒内忽略所有按键。
//   原因：上电瞬间 SD 卡初始化会让整机停顿，这段时间里按键电平若有毛刺/
//   长按电平会被 key_short_long 误判为“长按”，表现为一上电就进了视频模式（深蓝屏）。
// ---------------------------------------------------------------------------
reg [25:0] warmup_cnt;                       // 2s @25MHz
always @(posedge video_clk or posedge rst_all) begin
    if (rst_all) warmup_cnt <= 26'd0;
    else if (warmup_cnt != 26'd50_000_000) warmup_cnt <= warmup_cnt + 26'd1;
end
wire warmup_done = (warmup_cnt >= 26'd50_000_000);
wire key1_g = warmup_done ? key1 : 1'b1;     // 未到时间一律当“松开”
wire key2_g = warmup_done ? key2 : 1'b1;
wire key3_g = warmup_done ? key3 : 1'b1;

// ===================== 视频播放模式（KEY2 长按 >1.5s 切换）=====================
wire k2_short, k2_long;
key_short_long #(
    .CLK_FREQ_HZ (25_000_000),
    .LONG_MS     (1500)
) u_key2_video (
    .clk         (video_clk),
    .rst         (rst_all),
    .button      (key1_g),                 // 板上 KEY_2
    .short_pulse (k2_short),
    .long_pulse  (k2_long)
);

reg video_mode;
always @(posedge video_clk or posedge rst_all) begin
    if (rst_all)      video_mode <= 1'b0;
    else if (k2_long) video_mode <= ~video_mode;
end

// 视频模式下屏蔽图像路径的按键（高=未按下），避免它去抢 SD 卡

wire key_next_img = video_mode ? 1'b1 : key1_g;
wire key_auto_img = video_mode ? 1'b1 : key2_g;

// ---- 图像路径（sd_card_bmp）的原始信号 ----
wire        img_display_valid;
wire [1:0]  img_write_buf_idx;
wire [1:0]  img_disp_buf_idx;
wire        img_write_req;
wire        img_write_en;
wire [31:0] img_write_data;
wire [7:0]  img_dbg;          // 图像加载/提交状态（给屏幕方块）
wire        img_load_busy;    // 正在加载一张图（音频此时让出 SD 总线）
wire        img_wd_seen;      // 加载超时被看门狗中止过（诊断方块）
wire        img_sd_init_done; // SD 卡初始化完成（诊断方块 1：长期绿=卡没起来）

// ---- FAT32 目录解析结果：直接定位 .VID / .WAV，避免逐扇区扫描 ----
wire        sd_locate_done;    // 目录解析完成
wire        sd_locate_error;   // 解析失败（无卡 / 非 FAT32）
wire [3:0]  fat_state_dbg;      // 目录解析状态机状态（调试方块）
wire        vid_file_valid;    // 目录里找到了 .VID
wire [31:0] vid_file_sector;   // .VID 文件首扇区
wire        wav_file_valid;    // 目录里找到了 .WAV
wire [31:0] wav_file_sector;   // .WAV 文件首扇区

// 【已删除】单缓冲测试模式（KEY3 长按切换）与它的测试图案：
//   KEY3 的长按检测在上电/SD 初始化期间会被误触发，结果一上电就进测试模式、
//   屏幕显示渐变测试图案（红底）。诊断目的已达到，这个开关连同图案一起删掉。

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
wire [3:0]  vid_st_dbg;

video_player #(
    .CLK_FREQ_HZ (100_000_000)
) u_video_player (
    .clk                 (sd_card_clk),
    .rst                 (rst_all),
    .enable              (video_mode),
    .file_valid          (vid_file_valid),
    .file_sector         (vid_file_sector),
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
    .active              (vid_active),
    .st_dbg              (vid_st_dbg)
);

// ---- 帧写通道与显示控制：视频模式接管，否则用图像路径 ----
assign display_valid      = video_mode ? vid_display_valid : img_display_valid;
assign disp_buf_idx       = video_mode ? vid_disp_buf_idx  : img_disp_buf_idx;
assign write_buf_idx      = video_mode ? vid_write_buf_idx : img_write_buf_idx;
assign sd_card_write_req  = video_mode ? vid_wr_req        : img_write_req;
assign sd_card_write_en   = video_mode ? vid_wr_en         : img_write_en;

// ---- 诊断：单缓冲模式（KEY3 长按）下把写入数据换成"已知图形" ----
// 目的：把「SDRAM 写/读/显示这条链路」和「SD 读回来的像素数据」分开判断。
// 图像路径的写数据 = sd_card_bmp 送出的 BMP 像素（测试图案已删除）
assign sd_card_write_data = video_mode ? vid_wr_data : img_write_data;

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
    .suspend           (video_mode),
    .single_buf        (1'b0),
    .state_code        (state_code),
    .dbg               (img_dbg),
    .load_busy_o       (img_load_busy),
    .wd_seen           (img_wd_seen),
    .sd_init_done_o    (img_sd_init_done),
    .bmp_width         (16'd640),
    .bmp_height        (16'd480),
    .display_valid     (img_display_valid),

    // ===== FAT32 目录解析（一次到位，供视频/音频跳转）=====
    .locate_done       (sd_locate_done),
    .locate_error      (sd_locate_error),
    .fat_state_dbg     (fat_state_dbg),
    .vid_file_valid    (vid_file_valid),
    .vid_file_sector   (vid_file_sector),
    .wav_file_valid    (wav_file_valid),
    .wav_file_sector   (wav_file_sector),

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
// 视频模式下播放器每帧都交换显示缓冲区，不能当成"切图"去触发转场，
// 否则画面会被 SLIDE 效果一帧一帧地横向推来推去（上板现象：图片左右抽动）
wire switch_pulse = (~video_mode) && (disp_sync2 != disp_sync1);

// 【重要】转场特效暂时关闭：读地址走转场逻辑时，切图后整屏会变成灰黑乱码
// （已用仿真确认转场地址序列本身正确，问题在参数/回收时序，需要专门再调）。
// 关掉后切图就是干净的缓冲区切换。等基础显示彻底稳定后把这里改回 1'b1 再继续调特效。
localparam TRANS_EFFECT_ENABLE = 1'b0;
wire trans_switch_pulse = TRANS_EFFECT_ENABLE ? switch_pulse : 1'b0;

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
    .switch_pulse (trans_switch_pulse),
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
    .button      (key3_g),
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

// ===================== 屏幕顶部 8 个方块（诊断用） =====================
// 方块含义随模式自动切换，具体见文件末尾 key_dbg 定义处的注释。
// 绿色 = 0，红色 = 1；从左到右对应 key_dbg[7] .. key_dbg[0]。
reg k1_lvl, k2_lvl, k3_lvl;
always @(posedge video_clk or posedge rst_all) begin
    if (rst_all) begin
        k1_lvl <= 1'b1; k2_lvl <= 1'b1; k3_lvl <= 1'b1;
    end else begin
        k1_lvl <= key1_g; k2_lvl <= key2_g; k3_lvl <= key3_g;
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
    .button      (key2_g),
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

// 图像模式：看"按下 KEY_2 切图"这条链走到哪一步（方块 1 在最左）
//   [7] 方块1 sd_init_done（=1 说明 SD 卡初始化成功；一直绿=卡没起来，后面全白搭）
//   [6] 方块2 display_valid（=1 说明已经有一张图提交成功）
//   [5] 方块3 load_busy（正在加载；一直红=卡住）
//   [4] 方块4 img_wd_seen（看门狗曾经中止过一次加载）
//   [3] 方块5 short_load（=1 表示刚提交的那一帧没收全 307200 个像素 → SD 数据在路上丢了）
//   [2:0] 方块6~8 bmp_read 状态码（1=空闲/已加载完 2=扫描或读头 3=等写许可 4=收像素）
reg [7:0] dbg_sel;
always @(*) dbg_sel = video_mode
    // 视频模式用“红绿红绿红绿”交替图案，绝不会和图像模式混淆
    ? { vid_display_valid, 1'b1, vid_active, 1'b0, vid_st_dbg[3:0] }
    : { img_sd_init_done, img_dbg[7], img_load_busy, wav_playing, fat_state_dbg[3:0] };
wire [7:0] key_dbg = dbg_sel;

// 诊断方块要跨时钟域：key_dbg 在 sd_card_clk(100MHz) 域产生，方块画在 video_clk(25MHz) 域。
// 先打两拍再送进 OSD —— 否则这条跨时钟组合路径会被时序分析当成真实路径
// （实测 Slack -4.252ns，8 条）。方块是给人看的，打拍引入的瞬时错值无所谓。
reg [7:0] key_dbg_v1, key_dbg_v2;
always @(posedge video_clk or posedge rst_all) begin
    if (rst_all) begin
        key_dbg_v1 <= 8'd0;
        key_dbg_v2 <= 8'd0;
    end else begin
        key_dbg_v1 <= key_dbg;
        key_dbg_v2 <= key_dbg_v1;
    end
end

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
    .key_dbg (key_dbg_v2),
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
    .enable              (display_valid & ~video_mode & ~img_load_busy),  // 视频模式/加载图片时音频让路
    .file_valid          (wav_file_valid),
    .file_sector         (wav_file_sector),
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

assign audio_valid      = (wav_playing & wav_file_valid) ? wav_valid      : tone_valid;
assign audio_left_data  = (wav_playing & wav_file_valid) ? wav_left_data  : tone_left_data;
assign audio_right_data = (wav_playing & wav_file_valid) ? wav_right_data : tone_right_data;

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
