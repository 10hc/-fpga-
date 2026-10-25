
module sd_card_bmp #(
    parameter integer CLK_FREQ_HZ       = 100_000_000,
    parameter [31:0]  SCAN_START_SECTOR = 32'd0,
    parameter [31:0]  SCAN_MAX_SECTOR   = 32'd131071,
    parameter [2:0]   SCAN_TARGET_COUNT = 3'd4
)(
    input                       clk,
    input                       rst,
    input                       key_next,
    input                       key_auto,
    input                       suspend,       // 视频模式：停掉图片自动播放/加载，别和视频抢带宽
    input                       single_buf,    // 诊断：单缓冲测试（新图写进当前正在显示的缓冲，用来分辨“写坏了”还是“切缓冲坏了”）
    output [3:0]                state_code,
    output [7:0]                dbg,           // 调试：加载/提交状态，给屏幕方块用
    output                      load_busy_o,   // 调试：正在加载一张图
    output                      wd_seen,       // 调试：加载超时被看门狗中止过
    input  [15:0]               bmp_width,
    input  [15:0]               bmp_height,
    output reg                  display_valid,

    input                       write_finish_toggle,
    output reg [1:0]            write_buf_idx,
    output reg [1:0]            disp_buf_idx,

    output                      write_req,
    input                       write_req_ack,
    output                      write_en,
    output [31:0]               write_data,
    output                      SD_nCS,
    output                      SD_DCLK,
    output                      SD_MOSI,
    input                       SD_MISO,

    // ===== WAV 播放器的扇区读通道（与图像请求仲裁：图像优先）=====
    input                       wav_sec_read,
    input  [31:0]               wav_sec_read_addr,
    output [7:0]                wav_sec_read_data,
    output                      wav_sec_read_data_valid,
    output                      wav_sec_read_end
);

wire key_next_press;
wire key_auto_press;

// 图像（bmp_read）的读请求
wire             bmp_sec_read;
wire [31:0]      bmp_sec_read_addr;
wire [7:0]       bmp_sec_read_data;
wire             bmp_sec_read_data_valid;
wire             bmp_sec_read_end;
// 仲裁后送给 SD 控制器
wire             sd_sec_read;
wire [31:0]      sd_sec_read_addr;
wire [7:0]       sd_sec_read_data;
wire             sd_sec_read_data_valid;
wire             sd_sec_read_end;
wire             bmp_data_wr_en;
wire [23:0]      bmp_data;
wire             sd_init_done;
wire             bmp_ready;
wire             scan_done;
wire             scan_found_valid;
wire [31:0]      scan_found_sector;
wire [2:0]       scan_found_total;

reg              scan_start_pulse;
reg              load_start_pulse;
reg [31:0]       load_sector;
reg              scan_kicked;
reg              first_image_committed;
reg              auto_play_en;
reg [31:0]       auto_cnt;
reg [2:0]        img_found_count;
reg [1:0]        img_idx;              // 当前真正显示中的图片编号
reg [1:0]        load_idx;             // 当前正在写入的图片编号
reg [1:0]        pending_buf_idx;      // 当前正在写入的目标缓冲区
reg [31:0]       img_sector0;
reg [31:0]       img_sector1;
reg [31:0]       img_sector2;
reg [31:0]       img_sector3;
reg              next_req_pending;
reg              load_busy;

// bmp_ready 先表示“源文件读取/送 FIFO 完成”；真正切显示要等 write_finish_toggle 同步后
reg              source_done_seen;
// 整帧写完事件：与 source_done_seen 谁先到都先记住，两个都到齐才提交（避免先后顺序问题）
reg              write_done_seen;

// 看门狗触发过的锁存（屏幕上用方块指示）：加载超时会被“中止”，不再拿没写完的缓冲区去显示
reg              wd_seen_r;
// single_buf 在 video_clk 域由 KEY3 长按切换，跨到 sd 时钟域用两级同步
reg [1:0]        single_buf_sync;

// 同步 mem_clk 域的 write_finish_toggle
reg [2:0]        wrfin_tgl_sync;
wire             write_finish_pulse;

wire auto_tick;
wire [1:0] next_from_current;

assign write_en   = bmp_data_wr_en;
assign write_data = {bmp_data[23:16], bmp_data[15:8], bmp_data[7:0], 8'b0};
assign auto_tick  = (auto_cnt == (CLK_FREQ_HZ - 1));
assign next_from_current = next_index_limited(img_idx, img_found_count);
assign write_finish_pulse = wrfin_tgl_sync[2] ^ wrfin_tgl_sync[1];

// 给顶层的诊断输出
assign load_busy_o = load_busy;
assign wd_seen     = wd_seen_r;

// KEY3 长按切换的“单缓冲测试模式”同步
wire single_buf_s = single_buf_sync[1];
always @(posedge clk or posedge rst) begin
    if (rst) single_buf_sync <= 2'b00;
    else     single_buf_sync <= {single_buf_sync[0], single_buf};
end

// 帧写完成脉冲的可见指示：拉长到约 0.3 秒，这样屏幕上能直接看到“有帧写完成”
reg [24:0] wfin_hold;
always @(posedge clk or posedge rst) begin
    if (rst)                        wfin_hold <= 25'd0;
    else if (write_finish_pulse)    wfin_hold <= 25'd30_000_000;
    else if (wfin_hold != 25'd0)    wfin_hold <= wfin_hold - 25'd1;
end
wire write_finish_lat = (wfin_hold != 25'd0);

// 加载看门狗：一次加载超过 2 秒还没提交就强制结束。
// 注意：一帧 921KB 在 25MHz SPI 下约需 0.25~0.5 秒，原来的 0.5 秒会把正常加载误判成卡死，
// 于是每次都在“差最后一点”时被放弃 → 图片永远提交不了（这曾经导致完全不出图）
localparam [31:0] LOAD_WD_CYCLES = 32'd1_000_000_000;   // 10 秒 @100MHz（加载期间音频已让路，正常 <1 秒）
reg [31:0] load_wd;
always @(posedge clk or posedge rst) begin
    if (rst)                           load_wd <= 32'd0;
    else if (!load_busy)               load_wd <= 32'd0;
    else if (load_wd < LOAD_WD_CYCLES) load_wd <= load_wd + 32'd1;
end
wire load_wd_fire = (load_wd >= LOAD_WD_CYCLES);

// 【踩过的坑，不要再加回来】
// 曾经在这里加过“源文件送完后等 1ms 就提交”（想摆脱对 write_finish 脉冲的依赖），
// 结果上电第一张图变成灰黑乱码。原因：bmp_read 的 ready = (state == ST_IDLE)，
// 它是“空闲可接收”而不是“送完了”，load_busy 一拉高它还是 1，
// 所以 source_done_seen 在加载刚开始就被置起，1ms 后提交的是几乎空的缓冲区。
// 结论：提交必须等真正的整帧写完事件 write_done_seen，看门狗只做兜底。

// 屏幕方块诊断（与顶层注释一一对应）：
// [7]display_valid [6]load_busy [5]source_done_seen [4]write_done_seen
// [3]scan_done [2]找到>=1张 [1]找到>=2张 [0]找到>=3张
assign dbg = {display_valid, load_busy, source_done_seen, write_done_seen,
              scan_done,
              (img_found_count >= 3'd1), (img_found_count >= 3'd2), (img_found_count >= 3'd3)};

key_press_debounce #(
    .CLK_FREQ_HZ (CLK_FREQ_HZ),
    .DEBOUNCE_MS (20)
) u_key_next (
    .clk        (clk),
    .rst        (rst),
    .button_in  (key_next),
    .press_pulse(key_next_press)
);

key_press_debounce #(
    .CLK_FREQ_HZ (CLK_FREQ_HZ),
    .DEBOUNCE_MS (20)
) u_key_auto (
    .clk        (clk),
    .rst        (rst),
    .button_in  (key_auto),
    .press_pulse(key_auto_press)
);

function [1:0] next_index_limited;
    input [1:0] cur;
    input [2:0] count;
    begin
        case (count)
            3'd0: next_index_limited = 2'd0;
            3'd1: next_index_limited = 2'd0;
            3'd2: next_index_limited = (cur == 2'd1) ? 2'd0 : (cur + 2'd1);
            3'd3: next_index_limited = (cur == 2'd2) ? 2'd0 : (cur + 2'd1);
            default: next_index_limited = (cur == 2'd3) ? 2'd0 : (cur + 2'd1);
        endcase
    end
endfunction

function [31:0] sector_lut;
    input [1:0] idx;
    begin
        case (idx)
            2'd0: sector_lut = img_sector0;
            2'd1: sector_lut = img_sector1;
            2'd2: sector_lut = img_sector2;
            2'd3: sector_lut = img_sector3;
            default: sector_lut = img_sector0;
        endcase
    end
endfunction

function [1:0] next_buf_lut;
    input [1:0] cur_disp_buf;
    input       valid_now;
    begin
        if (!valid_now)
            next_buf_lut = 2'd0;                 // 首图固定写 buffer0
        else if (cur_disp_buf == 2'd0)
            next_buf_lut = 2'd1;
        else
            next_buf_lut = 2'd0;
    end
endfunction

always @(posedge clk or posedge rst) begin
    if (rst) begin
        wrfin_tgl_sync        <= 3'b000;
        scan_start_pulse      <= 1'b0;
        load_start_pulse      <= 1'b0;
        load_sector           <= 32'd0;
        scan_kicked           <= 1'b0;
        first_image_committed <= 1'b0;
        auto_play_en          <= 1'b0;
        auto_cnt              <= 32'd0;
        img_found_count       <= 3'd0;
        img_idx               <= 2'd0;
        load_idx              <= 2'd0;
        pending_buf_idx       <= 2'd0;
        write_buf_idx         <= 2'd0;
        disp_buf_idx          <= 2'd0;
        img_sector0           <= 32'd0;
        img_sector1           <= 32'd0;
        img_sector2           <= 32'd0;
        img_sector3           <= 32'd0;
        next_req_pending      <= 1'b0;
        load_busy             <= 1'b0;
        source_done_seen      <= 1'b0;
        write_done_seen       <= 1'b0;
        display_valid         <= 1'b0;
        wd_seen_r             <= 1'b0;
    end else begin
        wrfin_tgl_sync   <= {wrfin_tgl_sync[1:0], write_finish_toggle};
        scan_start_pulse <= 1'b0;
        load_start_pulse <= 1'b0;

        if (!sd_init_done) begin
            scan_kicked           <= 1'b0;
            first_image_committed <= 1'b0;
            auto_play_en          <= 1'b0;
            auto_cnt              <= 32'd0;
            img_found_count       <= 3'd0;
            img_idx               <= 2'd0;
            load_idx              <= 2'd0;
            pending_buf_idx       <= 2'd0;
            write_buf_idx         <= 2'd0;
            disp_buf_idx          <= 2'd0;
            img_sector0           <= 32'd0;
            img_sector1           <= 32'd0;
            img_sector2           <= 32'd0;
            img_sector3           <= 32'd0;
            next_req_pending      <= 1'b0;
            load_busy             <= 1'b0;
            source_done_seen      <= 1'b0;
            write_done_seen       <= 1'b0;
            display_valid         <= 1'b0;
            wd_seen_r             <= 1'b0;
        end else begin
            // 扫描阶段缓存前 4 张图的起始 sector
            if (scan_found_valid) begin
                case (img_found_count)
                    3'd0: img_sector0 <= scan_found_sector;
                    3'd1: img_sector1 <= scan_found_sector;
                    3'd2: img_sector2 <= scan_found_sector;
                    3'd3: img_sector3 <= scan_found_sector;
                    default: ;
                endcase

                if (img_found_count < 3'd4)
                    img_found_count <= img_found_count + 3'd1;
            end

            // 记住两个“完成”事件：源文件送完 FIFO、整帧写完 SDRAM。
            // 谁先到都先记住，不再要求“同一时刻同时成立”（那是原来不出图的第二个原因）
            if (load_busy && bmp_ready)
                source_done_seen <= 1'b1;

            if (load_busy && write_finish_pulse)
                write_done_seen  <= 1'b1;

            // 真正整帧写完（write_done_seen）才提交新图并切换显示缓冲区。
            // 注意：source_done_seen 其实在加载一开始就被置起了（见文件上方“踩过的坑”），
            // 所以这里的实质判据是 write_done_seen，看门狗负责兜底。
            if (load_busy && source_done_seen && write_done_seen) begin
                load_busy             <= 1'b0;
                source_done_seen      <= 1'b0;
                write_done_seen       <= 1'b0;
                disp_buf_idx          <= pending_buf_idx;
                img_idx               <= load_idx;
                display_valid         <= 1'b1;
                first_image_committed <= 1'b1;
            end

            // 看门狗兜底：加载超时（正常 <1 秒，超时说明真卡住了）。
            // 【重要修正】超时只“中止加载”，绝不提交：原来超时会强行 commit，
            // 把一块根本没写完的 SDRAM 区当图片显示 → 屏幕上就是彩色乱码/条纹。
            if (load_busy && load_wd_fire && !(source_done_seen && write_done_seen)) begin
                load_busy        <= 1'b0;
                source_done_seen <= 1'b0;
                write_done_seen  <= 1'b0;
                wd_seen_r        <= 1'b1;
            end

            // 上电后自动发起一次“扫描前 4 张 BMP”
            if (!scan_kicked && bmp_ready) begin
                scan_start_pulse      <= 1'b1;
                scan_kicked           <= 1'b1;
                first_image_committed <= 1'b0;
                auto_play_en          <= 1'b0;
                auto_cnt              <= 32'd0;
                img_found_count       <= 3'd0;
                img_idx               <= 2'd0;
                load_idx              <= 2'd0;
                pending_buf_idx       <= 2'd0;
                write_buf_idx         <= 2'd0;
                disp_buf_idx          <= 2'd0;
                next_req_pending      <= 1'b0;
                display_valid         <= 1'b0;
                load_busy             <= 1'b0;
                source_done_seen      <= 1'b0;
                write_done_seen       <= 1'b0;
                wd_seen_r             <= 1'b0;
            end else begin
                // 忙的时候也只记 1 次“下一张”请求，不会累积成连跳两张
                if (key_next_press && scan_done && (img_found_count > 3'd0))
                    next_req_pending <= 1'b1;

                // 自动播放开/关
                if (key_auto_press && scan_done && (img_found_count > 3'd1)) begin
                    auto_play_en <= ~auto_play_en;
                    auto_cnt     <= 32'd0;
                end

                // 自动播放 1s 计数：只有当前没有写图任务时才计时
                if (scan_done && auto_play_en && display_valid && !load_busy && first_image_committed && (img_found_count > 3'd1)) begin
                    if (auto_tick)
                        auto_cnt <= 32'd0;
                    else
                        auto_cnt <= auto_cnt + 32'd1;
                end else begin
                    auto_cnt <= 32'd0;
                end

                // 视频模式：停掉自动播放并撤掉挂起的切图请求（这些会去抢 SD 读和 SDRAM 写通道）
                if (suspend) begin
                    auto_play_en     <= 1'b0;
                    next_req_pending <= 1'b0;
                    auto_cnt         <= 32'd0;
                end

                // 首图自动加载到 buffer0
                if (scan_done && !first_image_committed && bmp_ready && !load_busy && !suspend && (img_found_count != 3'd0)) begin
                    load_idx         <= 2'd0;
                    load_sector      <= img_sector0;
                    pending_buf_idx  <= 2'd0;
                    write_buf_idx    <= 2'd0;
                    load_start_pulse <= 1'b1;
                    load_busy        <= 1'b1;
                    source_done_seen <= 1'b0;
                    write_done_seen  <= 1'b0;
                    wd_seen_r        <= 1'b0;
                    next_req_pending <= 1'b0;
                    auto_cnt         <= 32'd0;
                end
                // 手动下一张优先：写到“非当前显示”的另一块 buffer
                else if (scan_done && bmp_ready && display_valid && !load_busy && next_req_pending && (img_found_count != 3'd0)) begin
                    load_idx         <= next_from_current;
                    load_sector      <= sector_lut(next_from_current);
                    pending_buf_idx  <= single_buf_s ? 2'd0 : next_buf_lut(disp_buf_idx, display_valid);
                    write_buf_idx    <= single_buf_s ? 2'd0 : next_buf_lut(disp_buf_idx, display_valid);
                    load_start_pulse <= 1'b1;
                    load_busy        <= 1'b1;
                    source_done_seen <= 1'b0;
                    write_done_seen  <= 1'b0;
                    wd_seen_r        <= 1'b0;
                    next_req_pending <= 1'b0;
                    auto_cnt         <= 32'd0;
                end
                // 自动播放下一张：同样写到“非当前显示”的另一块 buffer
                else if (scan_done && bmp_ready && display_valid && !load_busy && auto_play_en && auto_tick && (img_found_count > 3'd1)) begin
                    load_idx         <= next_from_current;
                    load_sector      <= sector_lut(next_from_current);
                    pending_buf_idx  <= single_buf_s ? 2'd0 : next_buf_lut(disp_buf_idx, display_valid);
                    write_buf_idx    <= single_buf_s ? 2'd0 : next_buf_lut(disp_buf_idx, display_valid);
                    load_start_pulse <= 1'b1;
                    load_busy        <= 1'b1;
                    source_done_seen <= 1'b0;
                    write_done_seen  <= 1'b0;
                    wd_seen_r        <= 1'b0;
                    auto_cnt         <= 32'd0;
                end
            end
        end
    end
end

bmp_read bmp_read_m0(
    .clk                    (clk),
    .rst                    (rst),
    .ready                  (bmp_ready),

    .scan_start             (scan_start_pulse),
    .scan_start_sector      (SCAN_START_SECTOR),
    .scan_max_sector        (SCAN_MAX_SECTOR),
    .scan_target_count      (SCAN_TARGET_COUNT),
    .scan_done              (scan_done),
    .scan_found_valid       (scan_found_valid),
    .scan_found_sector      (scan_found_sector),
    .scan_found_total       (scan_found_total),

    .load_start             (load_start_pulse),
    .load_sector            (load_sector),

    .sd_init_done           (sd_init_done),
    .state_code             (state_code),
    .bmp_width              (bmp_width),
    .bmp_height             (bmp_height),
    .write_req              (write_req),
    .write_req_ack          (write_req_ack),
    .sd_sec_read            (bmp_sec_read),
    .sd_sec_read_addr       (bmp_sec_read_addr),
    .sd_sec_read_data       (bmp_sec_read_data),
    .sd_sec_read_data_valid (bmp_sec_read_data_valid),
    .sd_sec_read_end        (bmp_sec_read_end),
    .bmp_data_wr_en         (bmp_data_wr_en),
    .bmp_data               (bmp_data)
);

// ===================== 扇区读仲裁：图像优先，WAV 用空档 =====================
// 一次事务 = 请求拉高 → read_end。事务进行中不切换授权，
// 所以绝不会打断 SD 卡正在执行的命令；图像路径的时序与原来完全一致，
// 最坏情况是图像请求多等一个 WAV 扇区（约 300us）。
reg granted_wav;
reg sec_busy;

always @(posedge clk or posedge rst) begin
    if (rst) begin
        granted_wav <= 1'b0;
        sec_busy    <= 1'b0;
    end else begin
        if (sd_sec_read_end)    sec_busy <= 1'b0;
        else if (sd_sec_read)   sec_busy <= 1'b1;

        if (!sec_busy)
            granted_wav <= wav_sec_read & ~bmp_sec_read;
    end
end

assign sd_sec_read      = granted_wav ? wav_sec_read      : bmp_sec_read;
assign sd_sec_read_addr = granted_wav ? wav_sec_read_addr : bmp_sec_read_addr;

// 数据只回送给被授权的一方
assign bmp_sec_read_data       = sd_sec_read_data;
assign bmp_sec_read_data_valid = sd_sec_read_data_valid & ~granted_wav;
assign bmp_sec_read_end        = sd_sec_read_end        & ~granted_wav;

assign wav_sec_read_data       = sd_sec_read_data;
assign wav_sec_read_data_valid = sd_sec_read_data_valid & granted_wav;
assign wav_sec_read_end        = sd_sec_read_end        & granted_wav;

sd_card_top sd_card_top_m0(
    .clk                    (clk),
    .rst                    (rst),
    .SD_nCS                 (SD_nCS),
    .SD_DCLK                (SD_DCLK),
    .SD_MOSI                (SD_MOSI),
    .SD_MISO                (SD_MISO),
    .sd_init_done           (sd_init_done),
    .sd_sec_read            (sd_sec_read),
    .sd_sec_read_addr       (sd_sec_read_addr),
    .sd_sec_read_data       (sd_sec_read_data),
    .sd_sec_read_data_valid (sd_sec_read_data_valid),
    .sd_sec_read_end        (sd_sec_read_end),
    .sd_sec_write           (1'b0),
    .sd_sec_write_addr      (32'd0),
    .sd_sec_write_data      (),
    .sd_sec_write_data_req  (),
    .sd_sec_write_end       ()
);

endmodule

module key_press_debounce #(
    parameter integer CLK_FREQ_HZ = 100_000_000,
    parameter integer DEBOUNCE_MS = 20
)(
    input  wire clk,
    input  wire rst,
    input  wire button_in,    // 默认：松开=1，按下=0
    output reg  press_pulse
);

localparam integer DEBOUNCE_CYCLES = (CLK_FREQ_HZ / 1000) * DEBOUNCE_MS;

reg button_sync0;
reg button_sync1;
reg button_stable;
reg [31:0] cnt;

always @(posedge clk or posedge rst) begin
    if (rst) begin
        button_sync0  <= 1'b1;
        button_sync1  <= 1'b1;
        button_stable <= 1'b1;
        cnt           <= 32'd0;
        press_pulse   <= 1'b0;
    end else begin
        button_sync0 <= button_in;
        button_sync1 <= button_sync0;
        press_pulse  <= 1'b0;

        if (button_sync1 == button_stable) begin
            cnt <= 32'd0;
        end else begin
            if (cnt >= DEBOUNCE_CYCLES - 1) begin
                if (button_stable && !button_sync1)
                    press_pulse <= 1'b1;
                button_stable <= button_sync1;
                cnt           <= 32'd0;
            end else begin
                cnt <= cnt + 32'd1;
            end
        end
    end
end

endmodule
