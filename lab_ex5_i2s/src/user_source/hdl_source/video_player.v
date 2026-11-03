// ============================================================
// video_player.v —— TF 卡视频播放（320x240 RGB565 → 2 倍放大 → 640x480）
//
// 文件格式（.VID，全部小端）：
//   扇区 0 : 头部 512 字节（0:"VID1" 4:宽u16 6:高u16 8:fps u16 10:帧数u32）
//   扇区 1 起 : 逐帧 RGB565，每帧 320*240*2 = 153600 字节 = 恰好 300 扇区
//
// ★ 关键设计：源行 640 字节与 512 字节扇区不对齐，所以以
//   **5 个扇区为一组**（5*512 = 2560 字节 = 1280 像素 = 恰好 4 条源行）
//   整组收完再输出，永远不在扇区中间切换状态。
//   每帧 300 扇区 / 5 = 60 组，60 × 4 行 = 240 行 ✓
//
// 输出：每条源行发 2 遍（垂直 2 倍），每像素发 2 次（水平 2 倍）
//       4 行 × 320 像素 × 2 × 2 = 5120 字/组，60 组 = 307200 字 = 一帧 ✓
//       限速 1 字 / 4 拍；write_ready 背压时保留待发像素
//
// 双缓冲：写 A 显示 B，写完（wr_finish）再切换；帧率由头部 fps 决定。
// ============================================================
module video_player #(
    parameter integer CLK_FREQ_HZ = 100_000_000,
    parameter [31:0]  SCAN_START  = 32'd0,
    parameter [31:0]  SCAN_MAX    = 32'd131071,
    parameter integer SRC_W       = 320,
    parameter integer SRC_H       = 240,
    parameter [23:0] WAIT_LIMIT   = 24'hFFFFFF
)(
    input  wire        clk,
    input  wire        rst,
    input  wire        enable,

    // 目录解析结果：file_valid=1 时直接从 file_sector 开始找 "VID1"，
    // 不再从扇区 0 逐扇区扫描（原来要扫 13 万扇区 ≈ 40~50 秒）。
    input  wire        file_valid,
    input  wire [31:0] file_sector,

    output reg         sec_read,
    output reg  [31:0] sec_read_addr,
    input  wire [7:0]  sec_read_data,
    input  wire        sec_read_data_valid,
    input  wire        sec_read_end,

    output reg         wr_req,
    input  wire        wr_req_ack,
    input  wire        write_ready,   // clk 域：本拍 FIFO 可以接受一个字
    output wire        wr_en,         // 本拍实际写入（pending && write_ready）
    output reg  [31:0] wr_data,
    input  wire        wr_finish,
    output wire        wr_abort,

    output reg         display_valid,
    output reg  [1:0]  disp_buf_idx,
    output reg  [1:0]  write_buf_idx,
    output wire        active,
    output wire [3:0]  st_dbg          // 调试：当前状态，给屏幕方块用
);

    // Register the shared SD bus locally before RAM/header decoding.
    // Data, valid and parser-end have identical latency, including same-cycle
    // end. Request release and byte counting must still use the RAW interface
    // below: the real sector sequencer accepts a HIGH request in WAIT.
    reg [7:0] sec_data_q;
    reg sec_valid_q, sec_end_q;
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            sec_data_q <= 0; sec_valid_q <= 0; sec_end_q <= 0;
        end else begin
            sec_data_q <= sec_read_data;
            sec_valid_q <= sec_read_data_valid;
            sec_end_q <= sec_read_end;
        end
    end

    // ---------------- 常量 ----------------
    localparam [3:0]  GROUP_SEC  = 4'd5;
    localparam [10:0] GROUP_PX   = 11'd1280;
    localparam [2:0]  GROUP_LINE = 3'd4;
    localparam [8:0]  GROUPS     = 9'd60;
    localparam [8:0]  SRC_W_LAST = SRC_W - 1;      // 319

    localparam [2:0] S_IDLE = 3'd0,
                     S_SCAN = 3'd1,
                     S_HDR  = 3'd2,
                     S_GRP  = 3'd3,
                     S_OUT  = 3'd4,
                     S_WAIT = 3'd5,
                     S_PACE = 3'd6,
                     S_HALT = 3'd7;

    // S_WAIT 兜底上限：24 bit 满量程 ≈ 167ms @100MHz（正常一帧写入只需 ~6ms）
    localparam [23:0] WAIT_MAX = WAIT_LIMIT;

    // ---------------- 寄存器（全部先声明）----------------
    reg [2:0]  st;
    reg [31:0] addr;
    reg [3:0]  grp_sec;
    reg [8:0]  grp_grp;
    reg [10:0] g_wp;
    reg [7:0]  hdrb;
    reg        byte_odd;
    reg [9:0]  hdr_cnt;
    reg [31:0] hdr32;
    reg [15:0] fps_l;
    reg [31:0] frame_left;
    reg [31:0] total_frames, data_start;
    reg [15:0] header_width, header_height;
    reg [9:0] sector_bytes;
    reg write_done_seen, pace_due;
    reg [1:0]  wr_buf;

    reg [1:0]  o_sl;
    reg [8:0]  o_px;
    reg        o_dup;
    reg        o_row;
    reg [1:0]  o_div;
    reg        pixel_pending;

    reg [31:0] pace_acc;        // 帧率累加器（避免除法器）
    reg [23:0] wait_cnt;        // S_WAIT 兜底计数（写完成脉冲丢失时不至于僵死）
    reg [2:0]  fin_sync;
    reg        wr_req_set;      // 状态机发出的“请求一次帧写”脉冲
    reg        wr_ready;        // 本帧的写请求已被应答，可以送像素了
    reg [3:0]  ack_dly;         // ack 之后的等待计数
    reg [1:0] ack_sync;

    // Four-phase level handshake: memory holds ACK until our request drops.
    always @(posedge clk or posedge rst) begin
        if (rst) ack_sync <= 2'b00;
        else ack_sync <= {ack_sync[0], wr_req_ack};
    end

    // 分组缓冲（推断 BRAM）
    reg [15:0] gbuf [0:1279];
    reg [15:0] pix565;

    // ---------------- 组合 ----------------
    // 帧率节拍改用“累加器”实现（数学上等价于 CLK_FREQ_HZ/fps 的分频）。
    // 【踩过的坑】千万别写 CLK_FREQ_HZ / fps_l —— 那会综合出一个组合除法器：
    // 实测这条路径 103.6ns（84 级逻辑、56 个加法器串联），100MHz 下 Slack -93.8ns。
    wire [31:0] pace_sum  = pace_acc + {16'd0, fps_l};
    wire        pace_tick = (pace_sum >= CLK_FREQ_HZ);
    wire        fin_pulse = fin_sync[2] ^ fin_sync[1];

    wire [10:0] rd_idx  = (o_sl * 11'd320) + o_px;
    wire [23:0] pix888  = {pix565[15:11], pix565[15:13],
                           pix565[10:5],  pix565[10:9],
                           pix565[4:0],   pix565[4:2]};

    assign active = (st != S_IDLE) && (st != S_HALT);
    assign wr_abort = (st == S_HALT);
    // The FIFO consumes wr_en directly on clk. Gate the registered pending
    // word with CURRENT ready, so a falling ready cannot lose a delayed strobe.
    // Only this accepted write advances the pixel/group/frame position.
    assign wr_en = pixel_pending && write_ready && !rst;

    // Separate synchronous RAM ports: no reset on the array or read data.
    // This infers block RAM instead of a large asynchronous LUT multiplexer.
    always @(posedge clk) begin
        if (!rst && st == S_GRP && sec_valid_q && byte_odd &&
            g_wp < GROUP_PX)
            gbuf[g_wp] <= {sec_data_q, hdrb};
        pix565 <= gbuf[rd_idx];
    end

    // ---------------- 帧率节拍（唯一驱动 pace_acc，状态机里不得再赋值）----------------
    // 每拍 acc += fps；累加超过 CLK_FREQ_HZ 就减掉一个周期 → 每秒正好 fps 个 tick。
    always @(posedge clk or posedge rst) begin
        if (rst)               pace_acc <= 32'd0;
        else if (st == S_IDLE || st == S_SCAN || st == S_HDR || st == S_HALT)
            pace_acc <= 32'd0;
        else if (pace_tick)    pace_acc <= pace_sum - CLK_FREQ_HZ;
        else                   pace_acc <= pace_sum;
    end

    // ---------------- wr_finish 同步（独立，只碰 fin_sync）----------------
    always @(posedge clk or posedge rst) begin
        if (rst) fin_sync <= 3'd0;
        else     fin_sync <= {fin_sync[1:0], wr_finish};
    end

    assign st_dbg = {1'b0, st};

    // ---------------- 写请求握手（唯一驱动 wr_req / wr_ready）----------------
    // frame_fifo_write 的协议：写请求必须一直保持到 write_req_ack 到来；
    // 收到 ack 后必须立刻撤销请求，否则它会停在 S_ACK、而且写 FIFO 一直被异步清零，
    // 后面送进去的像素会被吃掉，导致 write_finish 永远不来（图像切不过去、视频也出不来）。
    // 另外 FIFO 的清零是在请求撤销那一拍才解除，所以 ack 之后要再等几拍才允许送数据。
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            wr_req   <= 1'b0;
            wr_ready <= 1'b0;
            ack_dly  <= 4'd0;
        end else if (st == S_IDLE || st == S_HALT) begin
            wr_req   <= 1'b0;
            wr_ready <= 1'b0;
            ack_dly  <= 4'd0;
        end else begin
            if (wr_req && ack_sync[1]) begin
                wr_req  <= 1'b0;              // 收到应答：立刻撤销（与状态机状态无关）
                ack_dly <= 4'd12;             // 再等 12 拍，让 FIFO 清零彻底解除
            end else if (ack_dly != 4'd0) begin
                ack_dly <= ack_dly - 4'd1;
                if (ack_dly == 4'd1) wr_ready <= 1'b1;   // 之后才允许送像素
            end
            if (wr_req_set) begin             // 状态机请求一次新的帧写
                wr_req   <= 1'b1;
                wr_ready <= 1'b0;
                ack_dly  <= 4'd0;
            end
        end
    end

    // 跳转模式下只容忍文件首扇区后 3 个扇区（头部正常就在首扇区），
    // 找不到就放弃，保持图像模式；没有目录信息时才退回原来的全盘扫描。
    wire [31:0] scan_stop = file_valid ? (file_sector + 32'd3) : SCAN_MAX;

    // ---------------- 主状态机 ----------------
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            st            <= S_IDLE;
            sec_read      <= 1'b0;
            sec_read_addr <= 32'd0;
            addr          <= SCAN_START;
            grp_sec       <= 4'd0;
            grp_grp       <= 9'd0;
            g_wp          <= 11'd0;
            hdrb          <= 8'd0;
            byte_odd      <= 1'b0;
            hdr_cnt       <= 8'd0;
            hdr32         <= 32'd0;
            fps_l         <= 16'd25;
            frame_left    <= 32'd0;
            total_frames  <= 32'd0;
            data_start    <= 32'd0;
            header_width  <= 16'd0;
            header_height <= 16'd0;
            wr_buf        <= 2'd0;
            o_sl          <= 2'd0;
            o_px          <= 9'd0;
            o_dup         <= 1'b0;
            o_row         <= 1'b0;
            o_div         <= 2'd0;
            wr_req_set    <= 1'b0;
            pixel_pending <= 1'b0;
            wr_data       <= 32'd0;
            disp_buf_idx  <= 2'd0;
            write_buf_idx <= 2'd0;
            display_valid <= 1'b0;
            wait_cnt      <= 24'd0;
        end else begin
            wr_req_set  <= 1'b0;

            case (st)
            // ================= 等使能 =================
            S_IDLE: begin
                sec_read <= 1'b0;
                if (!enable) display_valid <= 1'b0;
                if (enable) begin
                    addr    <= file_valid ? file_sector : SCAN_START;
                    hdr_cnt <= 10'd0;
                    hdr32   <= 32'd0;
                    st      <= S_SCAN;
                end
            end

            // ================= 扫描 "VID1" =================
            S_SCAN: begin
                if (!sec_read && !enable) st <= S_IDLE;
                else if (!sec_read) begin
                    sec_read      <= 1'b1;
                    sec_read_addr <= addr;
                    hdr_cnt       <= 8'd0;
                    hdr32         <= 32'd0;
                end
                if (sec_valid_q) begin
                    if (hdr_cnt < 10'd4) hdr32[hdr_cnt*8 +: 8] <= sec_data_q;
                    hdr_cnt <= hdr_cnt + 10'd1;
                end
                if (sec_end_q) begin
                    sec_read <= 1'b0;
                    if (!enable) st <= S_IDLE;
                    else if (sector_bytes != 10'd512) st <= S_HALT;
                    else if (hdr32 == 32'h31444956) begin
                        // 头部就在命中魔数的这个扇区里，地址不加；
                        // 头部读完后再 +1，帧数据从下一个扇区开始
                        st   <= S_HDR;
                    end else if (addr < scan_stop) begin
                        addr <= addr + 32'd1;
                    end else begin
                        st <= S_IDLE;                          // 没找到，保持图像模式
                    end
                end
            end

            // ================= 头部扇区 =================
            S_HDR: begin
                if (!sec_read && !enable) st <= S_IDLE;
                else if (!sec_read) begin
                    sec_read      <= 1'b1;
                    sec_read_addr <= addr;
                    hdr_cnt       <= 8'd0;
                end
                if (sec_valid_q) begin
                    if (hdr_cnt == 10'd4)       header_width[7:0] <= sec_data_q;
                    else if (hdr_cnt == 10'd5)  header_width[15:8] <= sec_data_q;
                    else if (hdr_cnt == 10'd6)  header_height[7:0] <= sec_data_q;
                    else if (hdr_cnt == 10'd7)  header_height[15:8] <= sec_data_q;
                    else if (hdr_cnt == 10'd8)  fps_l[7:0]  <= sec_data_q;
                    else if (hdr_cnt == 10'd9)  fps_l[15:8] <= sec_data_q;
                    else if (hdr_cnt >= 10'd10 && hdr_cnt <= 10'd13)
                        frame_left[(hdr_cnt - 8'd10)*8 +: 8] <= sec_data_q;
                    hdr_cnt <= hdr_cnt + 10'd1;
                end
                if (sec_end_q) begin
                    sec_read      <= 1'b0;
                    addr          <= addr + 32'd1;
                    grp_sec       <= 4'd0;
                    grp_grp       <= 9'd0;
                    g_wp          <= 11'd0;
                    byte_odd      <= 1'b0;
                    o_sl          <= 2'd0;
                    o_px          <= 9'd0;
                    o_dup         <= 1'b0;
                    o_row         <= 1'b0;
                    o_div         <= 2'd0;
                    wr_buf        <= wr_buf + 2'd1;
                    write_buf_idx <= wr_buf + 2'd1;
                    wr_req_set    <= 1'b1;
                    st            <= S_GRP;
                    data_start    <= addr + 32'd1;
                    total_frames  <= frame_left;
                    if (!enable) begin
                        wr_req_set <= 1'b0;
                        st <= S_IDLE;
                    end else if (sector_bytes != 10'd512 || header_width != SRC_W ||
                                 header_height != SRC_H || SRC_W != 320 || SRC_H != 240 ||
                                 fps_l == 16'd0 || fps_l > 16'd60 || frame_left == 32'd0) begin
                        wr_req_set <= 1'b0;
                        st <= S_HALT;
                    end
                end
            end

            // ================= 收一组（5 扇区 = 1280 像素）=================
            S_GRP: begin
                if (!sec_read) begin
                    sec_read      <= 1'b1;
                    sec_read_addr <= addr;
                end

                if (sec_valid_q) begin
                    if (byte_odd == 1'b0) begin
                        hdrb     <= sec_data_q;             // 像素低字节
                        byte_odd <= 1'b1;
                    end else begin
                        g_wp       <= g_wp + 11'd1;
                        byte_odd   <= 1'b0;
                    end
                end

                if (sec_end_q) begin
                    sec_read <= 1'b0;
                    addr     <= addr + 32'd1;
                    grp_sec  <= grp_sec + 4'd1;
                    byte_odd <= 1'b0;                          // 512 字节 = 256 像素，必为偶数
                    if (grp_sec + 4'd1 >= GROUP_SEC) begin
                        grp_sec <= 4'd0;
                        g_wp    <= 11'd0;
                        o_sl    <= 2'd0;
                        o_px    <= 9'd0;
                        o_dup   <= 1'b0;
                        o_row   <= 1'b0;
                        // Prefetch pixel zero after resetting the address
                        // left over from the previous group.
                        o_div   <= 2'd3;
                        st      <= S_OUT;
                    end
                    if (sector_bytes != 10'd512) st <= S_HALT;
                end
            end

            // ================= 输出一组（5120 字）=================
            S_OUT: begin
                if (!wr_ready) begin
                    if (wait_cnt == WAIT_MAX) st <= S_HALT;
                    else wait_cnt <= wait_cnt + 24'd1;
                end else if (wr_en) begin
                    pixel_pending <= 1'b0;
                    // Two cooldown cycles plus staging keep 1 word / 4 clocks.
                    o_div <= 2'd2;
                    if (o_dup == 1'b0) begin
                        o_dup <= 1'b1;                         // 同一像素再发一次
                    end else begin
                        o_dup <= 1'b0;
                        if (o_px == SRC_W_LAST) begin
                            o_px <= 9'd0;
                            if (o_row == 1'b0) begin
                                o_row <= 1'b1;                 // 同一行再发一遍（垂直 2 倍）
                            end else begin
                                o_row <= 1'b0;
                                if (o_sl == GROUP_LINE - 3'd1) begin
                                    grp_grp <= grp_grp + 9'd1;
                                    if (grp_grp + 9'd1 >= GROUPS) begin
                                        st <= S_WAIT;          // 整帧发完
                                        wait_cnt <= 24'd0;     // 进 S_WAIT 时清兜底计数
                                    end else begin
                                        st <= S_GRP;
                                    end
                                end else begin
                                    o_sl <= o_sl + 2'd1;
                                end
                            end
                        end else begin
                            o_px <= o_px + 9'd1;
                        end
                    end
                end else if (write_ready && !pixel_pending) begin
                    if (o_div == 2'd0) begin
                        wr_data <= {pix888, 8'b0};
                        pixel_pending <= 1'b1;
                    end else begin
                        o_div <= o_div + 2'd1;
                    end
                end
            end

            // ================= 等帧写完成 =================
            // Timeout aborts the pending frame and retains the last good display.
            S_WAIT: begin
                if (write_done_seen || fin_pulse) begin
                    disp_buf_idx  <= write_buf_idx;            // 切到新帧
                    display_valid <= 1'b1;
                    wait_cnt      <= 24'd0;
                    st            <= enable ? S_PACE : S_IDLE;
                end else if (wait_cnt == WAIT_MAX) begin
                    wait_cnt      <= 24'd0;
                    st            <= S_HALT;
                end else begin
                    wait_cnt <= wait_cnt + 24'd1;
                end
            end

            // ================= 帧率节拍 =================
            S_PACE: begin
                if (!enable) st <= S_IDLE;
                else if (pace_due || pace_tick) begin
                    grp_grp <= 9'd0;
                    g_wp    <= 11'd0;
                    wr_buf  <= wr_buf + 2'd1;
                    write_buf_idx <= wr_buf + 2'd1;
                    wr_req_set  <= 1'b1;
                    if (frame_left > 32'd1) frame_left <= frame_left - 32'd1;
                    else begin
                        addr <= data_start;
                        frame_left <= total_frames;
                    end
                    st      <= S_GRP;
                end
            end

            S_HALT: begin
                sec_read <= 1'b0;
                if (!enable) st <= S_IDLE;
            end
            default: st <= S_HALT;
            endcase

            // End of the physical transaction must release its level request
            // immediately, before the SD controller returns to WAIT next edge.
            // Waiting for sec_end_q starts an unwanted read of the old LBA.
            // Parsing still consumes the aligned registered data/end pair.
            if (sec_read_end) sec_read <= 1'b0;
        end
    end

    // Record early completion and run the frame clock during data transfer.
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            pace_due <= 1'b0;
            write_done_seen <= 1'b0;
            sector_bytes <= 10'd0;
        end else begin
            if (st == S_IDLE || st == S_HDR || st == S_HALT || st == S_SCAN)
                pace_due <= 1'b0;
            else if (st == S_PACE && (pace_due || pace_tick)) pace_due <= 1'b0;
            else if (pace_tick) pace_due <= 1'b1;
            if (st == S_IDLE || st == S_HALT || wr_req_set) write_done_seen <= 1'b0;
            else if (fin_pulse) write_done_seen <= 1'b1;
            if (!sec_read) sector_bytes <= 10'd0;
            // Count on the physical interface so the value is complete when
            // sec_end_q is consumed after the request has already dropped.
            else if (sec_read_data_valid && sector_bytes != 10'h3ff)
                sector_bytes <= sector_bytes + 10'd1;
        end
    end
endmodule
