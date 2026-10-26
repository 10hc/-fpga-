// ============================================================
// wav_audio.v —— TF 卡 WAV 播放（HDMI 音频输出）
//
// 全部自包含，不依赖 TD 的 FIFO IP：
//   sd_clk 域：扫描 RIFF/WAVE 头 → 流式读扇区 → 组装 16bit 立体声
//              → 写入 512×32 环形缓冲（自动推断 BRAM）
//              → 48kHz 节拍取样本 → 四相握手送给视频域
//   vid_clk 域：握手接收 → 16bit 转 24bit → 送 HDMI 音频接口
//
// 扇区读接口接到 sd_card_bmp 的 WAV 通道（内部与图像请求仲裁，图像优先，
// 一次事务内不切换，所以绝不会打断 SD 卡正在执行的命令）。
//
// 注意：找到头之后从"头所在扇区的下一个扇区"开始送数据，
//       会丢掉开头 468 字节（约 2.4ms），听感无影响；
//       播到 data 块末尾自动回到开头循环。
// ============================================================
module wav_audio #(
    parameter integer SD_CLK_HZ    = 50_000_000,
    parameter integer VID_CLK_HZ   = 25_000_000,
    parameter integer SAMPLE_RATE  = 48_000,
    parameter [31:0]  SCAN_START   = 32'd0,
    parameter [31:0]  SCAN_MAX     = 32'd131071
)(
    input  wire        sd_clk,
    input  wire        sd_rst,
    input  wire        vid_clk,
    input  wire        vid_rst,
    input  wire        enable,             // 例如首图显示后拉高

    // 目录解析结果：file_valid=1 时直接从 file_sector 开始找 RIFF，不再全盘扫描
    input  wire        file_valid,
    input  wire [31:0] file_sector,

    // ---- 扇区读接口（→ sd_card_bmp 的 WAV 通道）----
    output reg         sec_read,
    output reg  [31:0] sec_read_addr,
    input  wire [7:0]  sec_read_data,
    input  wire        sec_read_data_valid,
    input  wire        sec_read_end,

    // ---- 音频输出（vid_clk 域）----
    output reg         aud_valid,          // 每个样本 1 拍
    output reg  [23:0] aud_left,
    output reg  [23:0] aud_right,
    output wire        playing
);

    // ==================== 样本环形缓冲（sd 域，512×32 推断 BRAM）====================
    reg [31:0] sbuf [0:511];
    reg [8:0] wrp, rdp;

    wire [8:0] buf_cnt = wrp - rdp;
    wire       buf_empty = (wrp == rdp);

    // ==================== 状态机（sd 域）====================
    // 跳转模式下只容忍文件首扇区后 3 个扇区
    wire [31:0] scan_stop = file_valid ? (file_sector + 32'd3) : SCAN_MAX;

    localparam S_IDLE     = 3'd0;
    localparam S_SCAN_REQ = 3'd1;
    localparam S_SCAN_DAT = 3'd2;
    localparam S_STR_REQ  = 3'd3;
    localparam S_STR_DAT  = 3'd4;

    reg [2:0]   st;
    reg [31:0]  addr;
    reg [9:0]   byte_cnt;
    reg [351:0] hdr;                 // 44 字节文件头
    reg [15:0]  l16;                 // 组装的左声道
    reg [7:0]   r_low;               // 右声道低字节
    reg         streaming;
    reg [31:0]  data_left;           // 剩余数据字节（循环用）
    reg [31:0]  data_start;

    assign playing = streaming;

    // FIFO 有空间才读下一个扇区。
    // 【必须留一格】wrp==rdp 既是“空”也是“满”的表示：若允许写到正好 512，
    // 指针相等会被当成空 → 读指针不再前进、写指针继续跑 → 指针永久错位，
    // 之后放出来的就是垃圾数据（持续刺耳噪音，且切图也回不来）。
    // 一个扇区最多加 128 个样本，所以阈值取 383 时最大只到 511，永不满格。
    // 【必须留一格】wrp==rdp 既是“空”也是“满”的表示：允许写到 4095 就不能再写，
    // 否则计数绕回 0 会被当成“空”，读写指针永久错位（噪音）。
    // 512 样本 = 10.7ms。**【不要再加大】** 这种小数组 TD 会做成分布式 RAM（吃 LUT/切片），
    // 4096 样本会把 mslice 顶到 6626 > 4900 直接超限；音频不断音靠的是
    // “配额仲裁 + 只在扇区边界让路”，不是靠缓冲深度。
    wire buf_room = (buf_cnt <= 9'd383);

    always @(posedge sd_clk or posedge sd_rst) begin
        if (sd_rst) begin
            st            <= S_IDLE;
            sec_read      <= 1'b0;
            sec_read_addr <= 32'd0;
            addr          <= SCAN_START;
            byte_cnt      <= 10'd0;
            hdr           <= 352'd0;
            l16           <= 16'd0;
            r_low         <= 8'd0;
            streaming     <= 1'b0;
            data_left     <= 32'd0;
            data_start    <= 32'd0;
            wrp <= 9'd0;
            // 注意：rdp 只在下面"48kHz 节拍 + 握手"那个 always 块里驱动
            //      （包括它的复位），否则综合器会报 multiple drivers
        end else begin
            case (st)
                // ---------- 等待使能 ----------
                S_IDLE: begin
                    sec_read <= 1'b0;
                    if (enable) begin
                        if (streaming) begin
                            // 之前已经在放：直接从当前扇区继续
                            // （让路只在扇区边界发生，字节对齐没被打断）
                            st <= S_STR_REQ;
                        end else begin
                            addr <= file_valid ? file_sector : SCAN_START;
                            st   <= S_SCAN_REQ;
                        end
                    end
                end

                // ---------- 扫描：发起扇区读 ----------
                S_SCAN_REQ: begin
                    sec_read      <= 1'b1;
                    sec_read_addr <= addr;
                    byte_cnt      <= 10'd0;
                    st            <= S_SCAN_DAT;
                end

                // ---------- 扫描：收数据 + 校验头 ----------
                S_SCAN_DAT: begin
                    if (sec_read_data_valid) begin
                        if (byte_cnt < 10'd44)
                            hdr[byte_cnt*8 +: 8] <= sec_read_data;
                        byte_cnt <= byte_cnt + 10'd1;
                    end
                    if (sec_read_end) begin
                        sec_read <= 1'b0;
                        if (hdr[31:0]    == 32'h46464952 &&   // "RIFF"
                            hdr[95:64]   == 32'h45564157 &&   // "WAVE"
                            hdr[175:160] == 16'd1        &&   // PCM
                            hdr[191:176] == 16'd2        &&   // 2 声道
                            hdr[223:192] == 32'd48000    &&   // 48kHz
                            hdr[287:272] == 16'd16       &&   // 16bit
                            hdr[319:288] == 32'h61746164) begin // "data"
                            streaming  <= 1'b1;
                            addr       <= addr + 32'd1;
                            data_start <= addr + 32'd1;
                            data_left  <= (hdr[351:320] > 32'd468)
                                          ? (hdr[351:320] - 32'd468) : 32'd0;
                            st         <= S_STR_REQ;
                        end else if (addr < scan_stop) begin
                            addr <= addr + 32'd1;
                            st   <= S_SCAN_REQ;
                        end else begin
                            st <= S_IDLE;          // 没找到 WAV，保持测试音
                        end
                    end
                end

                // ---------- 流式：等缓冲有空间再读下一扇区 ----------
                S_STR_REQ: begin
                    // 不在这里停：仲裁器已经让图像请求优先，WAV 只在图像的空档里
                    // 插一个扇区（约 300us），而音乐流只要 192KB/s，完全喂得饱。
                    // 中途停会把缓冲排空 + 指针错位，那正是“切图后音乐变噪音”的来源。
                    // 真的要让路（进视频模式）时，只在扇区边界停 —— 见 S_STR_DAT。
                    if (buf_room) begin
                        sec_read      <= 1'b1;
                        sec_read_addr <= addr;
                        byte_cnt      <= 10'd0;
                        st            <= S_STR_DAT;
                    end
                end

                // ---------- 流式：收数据并组装样本 ----------
                S_STR_DAT: begin
                    if (sec_read_data_valid) begin
                        case (byte_cnt[1:0])
                            2'd0: l16[7:0]  <= sec_read_data;
                            2'd1: l16[15:8] <= sec_read_data;
                            2'd2: r_low     <= sec_read_data;
                            2'd3: begin
                                sbuf[wrp] <= {sec_read_data, r_low, l16}; // {R,L}
                                wrp <= wrp + 9'd1;
                            end
                        endcase
                        byte_cnt <= byte_cnt + 10'd1;
                    end
                    if (sec_read_end) begin
                        sec_read <= 1'b0;
                        if (data_left <= 32'd512) begin
                            addr      <= data_start;      // 循环回开头
                            data_left <= hdr[351:320];
                        end else begin
                            addr      <= addr + 32'd1;
                            data_left <= data_left - 32'd512;
                        end
                        st <= enable ? S_STR_REQ : S_IDLE;   // 要让路也只在扇区边界停（字节对齐不被打断）
                    end
                end

                default: st <= S_IDLE;
            endcase
        end
    end

    // ==================== 48kHz 节拍 + 握手发送（sd 域）====================
    reg  [31:0] acc;
    wire tick = ((acc + SAMPLE_RATE) >= SD_CLK_HZ);

    always @(posedge sd_clk or posedge sd_rst) begin
        if (sd_rst) acc <= 32'd0;
        else if (tick) acc <= acc + SAMPLE_RATE - SD_CLK_HZ;
        else           acc <= acc + SAMPLE_RATE;
    end

    reg        s_valid;
    reg [23:0] s_l, s_r;
    reg [1:0]  ack_sync;             // 来自视频域的 ack
    reg        ack;                  // 回给 sd 域的应答（vid 域驱动）

    always @(posedge sd_clk or posedge sd_rst) begin
        if (sd_rst) begin
            s_valid  <= 1'b0;
            s_l      <= 24'd0;
            s_r      <= 24'd0;
            ack_sync <= 2'd0;
            rdp <= 9'd0;                              // rdp 唯一驱动点
        end else begin
            ack_sync <= {ack_sync[0], ack};
            if (!s_valid) begin
                if (tick) begin
                    if (!buf_empty) begin
                        s_l <= {sbuf[rdp][15:0],  8'b0};    // 16bit → 24bit
                        s_r <= {sbuf[rdp][31:16], 8'b0};
                        rdp <= rdp + 9'd1;
                    end else begin
                        // 缓冲空了（例如正在给图片加载让路）：补一个静音样本。
                        // 48kHz 的节拍一个都不能少 —— 给 HDMI 的音频流一旦断供，
                        // 接收端就失去同步，听到的就是持续的噪音（而不是安静）。
                        s_l <= 24'd0;
                        s_r <= 24'd0;
                    end
                    s_valid <= 1'b1;
                end
            end else if (ack_sync[1]) begin
                s_valid <= 1'b0;                            // 收到应答，结束本次
            end
        end
    end

    // ==================== 握手接收（vid 域）====================
    reg [1:0] v_sync;

    always @(posedge vid_clk or posedge vid_rst) begin
        if (vid_rst) begin
            v_sync    <= 2'd0;
            ack       <= 1'b0;
            aud_valid <= 1'b0;
            aud_left  <= 24'd0;
            aud_right <= 24'd0;
        end else begin
            v_sync <= {v_sync[0], s_valid};
            if (v_sync[1] && !ack) begin
                // s_valid 已被同步 2 拍，s_l/s_r 早已稳定
                aud_left  <= s_l;
                aud_right <= s_r;
                aud_valid <= 1'b1;
                ack       <= 1'b1;
            end else begin
                aud_valid <= 1'b0;
                if (!v_sync[1]) ack <= 1'b0;
            end
        end
    end

endmodule