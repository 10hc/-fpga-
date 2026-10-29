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
    parameter [31:0]  SCAN_MAX     = 32'd131071,
    // 自检音：1 = 不读 SD 数据，在视频域自己产生 1kHz 三角波 + 约 0.7s 通断
    // 0 = 正常播放 WAV；1 = 输出通路自检音；2 = 缓冲自检（写已知正弦，不读 SD）
    parameter  [1:0]  SELFTEST     = 2'd0,
    // 期望校验和：PC 端对卡上文件 data_start 起前 4 扇区(2048B) 求和取低 16 位
    parameter  [15:0] SUM_EXPECT   = 16'h0000
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
    output wire        playing,
    output reg  [15:0] sum16,          // 前 4 个流式扇区的字节和（冻结）
    output wire        sum_ok
);

    // ==================== 样本环形缓冲（sd 域，512×32 推断 BRAM）====================
    // [容错] 4096 样本 = 85.3ms 缓冲（原 512 样本 = 10.7ms）。
    // 真卡若偶发“整块读延迟 >10ms”，老缓冲会被抽干 → 持续沙沙声；85ms 足以吸收两三次抖动。
    // 必须推断成块 RAM：512×32 落 LUT 就要 2754 个 LUT4，4096×32 更不可能。
    reg [31:0] sbuf [0:4095];
    reg [11:0] wrp, rdp;
    // SELFTEST==2 时用独立写指针（避免和状态机抢同一个 wrp 造成多驱动）
    reg [11:0] wrp_cf;
    reg [15:0] cf_ph;
    reg [15:0] sum_acc;
    reg [1:0]  sum_sec;
    wire [15:0] cf_s16 = cf_ph[15] ? (16'hFFFF - cf_ph) : cf_ph;   // 1kHz 三角波
    wire [11:0] wrp_eff = (SELFTEST == 2'd2) ? wrp_cf : wrp;
    assign sum_ok = (sum16 == SUM_EXPECT);

    wire [11:0] buf_cnt = wrp_eff - rdp;
    wire       buf_empty = (wrp_eff == rdp);

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
    reg [24:0]  to_cnt;   // S_STR_DAT 超时计数（看门狗强拆事务时自恢复）
    reg         draining; // 超时后丢弃本块剩余字节、等 end 收尾（避免整块重复+半帧错位）
    // 【2026-10-28 修·只收“完整块”】每个扇区开始前记下写指针，块不完整（被强拆/超时/字节数≠512）
    //   就回滚到块首，绝不把半块样本留在缓冲里 —— 半块是“从第一个音符起就沙沙响”的直接成因。
    reg [11:0]  wrp_blk;
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
    wire buf_room = (buf_cnt <= 12'd3967);   // 不允许写满，避免满/空二义

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
            draining      <= 1'b0;
            wrp_blk       <= 12'd0;
            to_cnt        <= 25'd0;
            sum_acc       <= 16'd0;
            sum_sec       <= 2'd0;
            sum16         <= 16'd0;
            wrp <= 12'd0;
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
                        to_cnt        <= 25'd0;
                        draining      <= 1'b0;
                        wrp_blk       <= wrp;              // 记下块首，块不完整就回滚
                        st            <= S_STR_DAT;
                    end
                end

                // ---------- 流式：收数据并组装样本 ----------
                S_STR_DAT: begin
                    if (sec_read_data_valid && !draining) begin
                        if (sum_sec < 2'd4) sum_acc <= sum_acc + {8'd0, sec_read_data};
                        case (byte_cnt[1:0])
                            2'd0: l16[7:0]  <= sec_read_data;
                            2'd1: l16[15:8] <= sec_read_data;
                            2'd2: r_low     <= sec_read_data;
                            2'd3: begin
                                // 数据写入在模块末尾的专用 always 块里（为了被推断成块 RAM）
                                wrp <= wrp + 12'd1;
                            end
                        endcase
                        byte_cnt <= byte_cnt + 10'd1;
                    end
                    // 【2026-10-28 修·根因】超时后不再“原地重发同一个扇区”，而是进入 drain：
                    //   把这一块剩下的字节全部丢弃、等它真正的 end 来收尾（地址/计数照常前进），
                    //   否则旧块的尾巴会以 byte_cnt=0 的身份被拼成新块的开头 → 整块重复 + 半帧错位
                    //   = 从头到尾的沙沙噪音。sd_card_bmp 侧已保证被强拆的主一定能拿到 end。
                    if (sec_read_data_valid) to_cnt <= 25'd0;
                    else if (!draining && to_cnt >= 25'd2_000_000) begin  // 20ms @100MHz
                        draining <= 1'b1;                 // 不再收字节，等 end
                        wrp      <= wrp_blk;              // 半块样本整体丢弃（不留噪音）
                        l16      <= 16'd0;                // 丢弃半帧
                        r_low    <= 8'd0;
                        to_cnt   <= 25'd0;
                    end else if (draining && to_cnt >= 25'd4_000_000) begin
                        // 40ms 还没等到 end：卡真的不回了，强制重同步（相位归零，重发同一扇区）
                        draining <= 1'b0;
                        sec_read <= 1'b0;
                        byte_cnt <= 10'd0;
                        l16      <= 16'd0;
                        r_low    <= 8'd0;
                        to_cnt   <= 25'd0;
                        st       <= S_STR_REQ;
                    end else to_cnt <= to_cnt + 25'd1;
                    if (sec_read_end) begin
                        sec_read <= 1'b0;
                        draining <= 1'b0;                 // 收尾完成，恢复正常接收
                        if (byte_cnt != 10'd512) wrp <= wrp_blk;  // 字节数不对 → 整块丢弃
                        if (sum_sec < 2'd4) begin
                            sum_sec <= sum_sec + 2'd1;
                            if (sum_sec == 2'd3) sum16 <= sum_acc;
                        end
                        byte_cnt <= 10'd0;
                        if (data_left <= 32'd512) begin
                            addr      <= data_start;      // 循环回开头
                            data_left <= (hdr[351:320] > 32'd468)
                                          ? (hdr[351:320] - 32'd468) : 32'd0;  // 回卷同样要减掉 468 字节偏移
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

    // ==================== 样本搬运（sd 域 → 持稳寄存器）====================
    // 【2026-10-26 重写】原来的做法是“sd 域 48kHz 节拍 + 四相握手”：
    //   节拍产生在 sd 域；若上一拍样本还没被视频域取走，这一拍就被整拍丢掉；
    //   缓冲空时还补静音。结果送给 HDMI 的音频流 cadence 不稳、偶尔断供，
    //   接收端失锁 → 持续刺耳的沙沙声（测试音那条链在音频域精确 48kHz，所以干净）。
    // 现在：sd 域只负责“把样本搬进持稳寄存器，并用 toggle 通知”；
    //       输出节拍改在 **视频域** 用累加器精确产生 48kHz，一个节拍都不丢；
    //       缓冲空时保持上一个样本（不补静音，避免爆音）。
    reg        s_toggle;             // 每送出一个样本翻转一次（跨域通知用）
    reg [23:0] s_l, s_r;

    always @(posedge sd_clk or posedge sd_rst) begin
        if (sd_rst) begin
            s_toggle <= 1'b0;
            rdp      <= 12'd0;                         // rdp 唯一驱动点
        end else if (tick) begin
            if (!buf_empty) begin
                rdp <= rdp + 12'd1;
            end
            // 缓冲空（例如正在给图片加载让路）：保持上一个样本，
            // 节拍照走 —— 给 HDMI 的音频流一旦断供，接收端失锁就是持续噪音。
            s_toggle <= ~s_toggle;
        end
    end

    // ==================== 48kHz 输出节拍（视频域，精确）====================
    reg  [31:0] vacc;
    wire vtick = ((vacc + SAMPLE_RATE) >= VID_CLK_HZ);

    always @(posedge vid_clk or posedge vid_rst) begin
        if (vid_rst) vacc <= 32'd0;
        else if (vtick) vacc <= vacc + SAMPLE_RATE - VID_CLK_HZ;
        else            vacc <= vacc + SAMPLE_RATE;
    end

    // ==================== 自检音（SELFTEST=1 时生效）====================
    // 用来区分“取数据坏了”还是“输出通路坏了”：
    //   自检音 = 间断的干净嘟嘟声 → 输出通路 OK，噪音来自 sbuf 里的数据；
    //   自检音也沙沙响            → 输出通路（vtick/跨域/aud_valid）本身有问题。
    reg  [15:0] st_ph;
    reg  [14:0] st_env;
    wire [15:0] st_tri = st_ph[15] ? (16'hFFFF - st_ph) : st_ph;
    wire        st_amp = ~st_env[14];

    always @(posedge vid_clk or posedge vid_rst) begin
        if (vid_rst) begin
            st_ph  <= 16'd0;
            st_env <= 15'd0;
        end else if (vtick) begin
            st_ph  <= st_ph + 16'd1365;     // 1365/65536 × 48kHz ≈ 1kHz
            st_env <= st_env + 15'd1;
        end
    end

    // ==================== 音频输出（vid 域）====================
    reg [1:0]  tog_sync;
    reg [23:0] h_l, h_r;             // 持稳样本（上一个有效样本）

    always @(posedge vid_clk or posedge vid_rst) begin
        if (vid_rst) begin
            tog_sync  <= 2'd0;
            h_l       <= 24'd0;
            h_r       <= 24'd0;
            aud_valid <= 1'b0;
            aud_left  <= 24'd0;
            aud_right <= 24'd0;
        end else begin
            tog_sync <= {tog_sync[0], s_toggle};
            // toggle 变化 = sd 域刚送来一个新样本（s_l/s_r 已稳定 ≥2 个视频时钟）
            if (tog_sync[1] ^ tog_sync[0]) begin
                h_l <= s_l;
                h_r <= s_r;
            end
            // 精确 48kHz 输出，一个节拍都不少
            if (vtick) begin
                aud_left  <= SELFTEST ? (st_amp ? {st_tri, 8'b0} : 24'd0) : h_l;
                aud_right <= SELFTEST ? (st_amp ? {st_tri, 8'b0} : 24'd0) : h_r;
                aud_valid <= 1'b1;
            end else begin
                aud_valid <= 1'b0;
            end
        end
    end

    // ==================== 采样环形缓冲的块 RAM 读写 ====================
    // 写口与读口放在同一个 always 块里、读口寄存，安路综合器才会推断成 BRAM；
    // 若把读值先做选择再寄存，或读写分处两个 always，就会退化成分布式 RAM
    // （512x32 会吃掉约 2754 个 LUT，这是上一版的实测值）。
    wire        sbuf_we  = (st == S_STR_DAT) && sec_read_data_valid && !draining
                           && (byte_cnt[1:0] == 2'd3);
    wire [31:0] sbuf_din = {sec_read_data, r_low, l16};
    wire        sbuf_re  = tick && !buf_empty;

    always @(posedge sd_clk or posedge sd_rst) begin
        if (sd_rst) begin
            s_l <= 24'd0;
            s_r <= 24'd0;
        end else begin
            if (SELFTEST == 2'd2) begin
                // 缓冲自检：用 48kHz 节拍把已知三角波写进缓冲（完全不碰 SD）
                if (tick && (buf_cnt <= 12'd3967)) begin
                    sbuf[wrp_cf] <= {cf_s16, cf_s16};
                    wrp_cf       <= wrp_cf + 12'd1;
                end
            end else if (sbuf_we) sbuf[wrp] <= sbuf_din;
            // 读口不加条件：rdp 只在 48kHz 节拍上推进，两次节拍之间读出的就是同一个样本，
            // 因此“每拍都读”与原来的“只在 tick 读”完全等价；而综合器要求读口无条件
            // 才肯推断成块 RAM（上一版加了 if(sbuf_re) 就被判成分布式 RAM）。
            s_l <= {sbuf[rdp][15:0],  8'b0};              // 16bit → 24bit
            s_r <= {sbuf[rdp][31:16], 8'b0};
        end
    end

endmodule
