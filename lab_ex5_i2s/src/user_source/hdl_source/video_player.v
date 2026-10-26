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
//       限速 1 字 / 2 拍，保护帧写 FIFO
//
// 双缓冲：写 A 显示 B，写完（wr_finish）再切换；帧率由头部 fps 决定。
// ============================================================
module video_player #(
    parameter integer CLK_FREQ_HZ = 100_000_000,
    parameter [31:0]  SCAN_START  = 32'd0,
    parameter [31:0]  SCAN_MAX    = 32'd131071,
    parameter integer SRC_W       = 320,
    parameter integer SRC_H       = 240
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
    output reg         wr_en,
    output reg  [31:0] wr_data,
    input  wire        wr_finish,

    output reg         display_valid,
    output reg  [1:0]  disp_buf_idx,
    output reg  [1:0]  write_buf_idx,
    output wire        active,
    output wire [3:0]  st_dbg          // 调试：当前状态，给屏幕方块用
);

    // ---------------- 常量 ----------------
    localparam [3:0]  GROUP_SEC  = 4'd5;
    localparam [10:0] GROUP_PX   = 11'd1280;
    localparam [1:0]  GROUP_LINE = 2'd4;
    localparam [8:0]  GROUPS     = 9'd60;
    localparam [8:0]  SRC_W_LAST = SRC_W - 1;      // 319

    localparam [2:0] S_IDLE = 3'd0,
                     S_SCAN = 3'd1,
                     S_HDR  = 3'd2,
                     S_GRP  = 3'd3,
                     S_OUT  = 3'd4,
                     S_WAIT = 3'd5,
                     S_PACE = 3'd6;

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
    reg [1:0]  wr_buf;

    reg [1:0]  o_sl;
    reg [8:0]  o_px;
    reg        o_dup;
    reg        o_row;
    reg        o_div;

    reg [31:0] pace_acc;        // 帧率累加器（避免除法器）
    reg [2:0]  fin_sync;
    reg        wr_req_set;      // 状态机发出的“请求一次帧写”脉冲
    reg        wr_ready;        // 本帧的写请求已被应答，可以送像素了
    reg [3:0]  ack_dly;         // ack 之后的等待计数

    // 分组缓冲（推断 BRAM）
    reg [15:0] gbuf [0:1279];

    // ---------------- 组合 ----------------
    // 帧率节拍改用“累加器”实现（数学上等价于 CLK_FREQ_HZ/fps 的分频）。
    // 【踩过的坑】千万别写 CLK_FREQ_HZ / fps_l —— 那会综合出一个组合除法器：
    // 实测这条路径 103.6ns（84 级逻辑、56 个加法器串联），100MHz 下 Slack -93.8ns。
    wire [31:0] pace_sum  = pace_acc + {16'd0, fps_l};
    wire        pace_tick = (pace_sum >= CLK_FREQ_HZ);
    wire        fin_pulse = fin_sync[2] ^ fin_sync[1];

    wire [10:0] rd_idx  = (o_sl * 11'd320) + o_px;
    wire [15:0] pix565  = gbuf[rd_idx];
    wire [23:0] pix888  = {pix565[15:11], pix565[15:13],
                           pix565[10:5],  pix565[10:9],
                           pix565[4:0],   pix565[4:2]};

    assign active = enable && (st != S_IDLE);

    // ---------------- 帧率节拍（唯一驱动 pace_acc，状态机里不得再赋值）----------------
    // 每拍 acc += fps；累加超过 CLK_FREQ_HZ 就减掉一个周期 → 每秒正好 fps 个 tick。
    always @(posedge clk or posedge rst) begin
        if (rst)               pace_acc <= 32'd0;
        else if (st != S_PACE) pace_acc <= 32'd0;   // 只有 S_PACE 状态才累加
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
        end else if (!enable) begin
            wr_req   <= 1'b0;
            wr_ready <= 1'b0;
            ack_dly  <= 4'd0;
        end else begin
            if (wr_req && wr_req_ack) begin
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
            wr_buf        <= 2'd0;
            o_sl          <= 2'd0;
            o_px          <= 9'd0;
            o_dup         <= 1'b0;
            o_row         <= 1'b0;
            o_div         <= 1'b0;
            wr_req_set    <= 1'b0;
            wr_en         <= 1'b0;
            wr_data       <= 32'd0;
            disp_buf_idx  <= 2'd0;
            write_buf_idx <= 2'd0;
            display_valid <= 1'b0;
        end else begin
            wr_en       <= 1'b0;
            wr_req_set  <= 1'b0;

            case (st)
            // ================= 等使能 =================
            S_IDLE: begin
                sec_read <= 1'b0;
                if (enable) begin
                    addr    <= file_valid ? file_sector : SCAN_START;
                    hdr_cnt <= 10'd0;
                    hdr32   <= 32'd0;
                    st      <= S_SCAN;
                end
            end

            // ================= 扫描 "VID1" =================
            S_SCAN: begin
                if (!sec_read) begin
                    sec_read      <= 1'b1;
                    sec_read_addr <= addr;
                    hdr_cnt       <= 8'd0;
                    hdr32         <= 32'd0;
                end
                if (sec_read_data_valid) begin
                    if (hdr_cnt < 10'd4) hdr32[hdr_cnt*8 +: 8] <= sec_read_data;
                    hdr_cnt <= hdr_cnt + 10'd1;
                end
                if (sec_read_end) begin
                    sec_read <= 1'b0;
                    if (hdr32 == 32'h31444956) begin          // "VID1"（小端：V=56 I=49 D=44 1=31）
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
                if (!sec_read) begin
                    sec_read      <= 1'b1;
                    sec_read_addr <= addr;
                    hdr_cnt       <= 8'd0;
                end
                if (sec_read_data_valid) begin
                    if (hdr_cnt == 10'd8)       fps_l[7:0]  <= sec_read_data;
                    else if (hdr_cnt == 10'd9)  fps_l[15:8] <= sec_read_data;
                    else if (hdr_cnt >= 10'd10 && hdr_cnt <= 10'd13)
                        frame_left[(hdr_cnt - 8'd10)*8 +: 8] <= sec_read_data;
                    hdr_cnt <= hdr_cnt + 10'd1;
                end
                if (sec_read_end) begin
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
                    o_div         <= 1'b0;
                    wr_buf        <= wr_buf + 2'd1;
                    write_buf_idx <= wr_buf + 2'd1;
                    wr_req_set    <= 1'b1;
                    st            <= S_GRP;
                end
            end

            // ================= 收一组（5 扇区 = 1280 像素）=================
            S_GRP: begin
                if (!sec_read) begin
                    sec_read      <= 1'b1;
                    sec_read_addr <= addr;
                end

                if (sec_read_data_valid) begin
                    if (byte_odd == 1'b0) begin
                        hdrb     <= sec_read_data;             // 像素低字节
                        byte_odd <= 1'b1;
                    end else begin
                        gbuf[g_wp] <= {sec_read_data, hdrb};   // 小端：高字节+低字节
                        g_wp       <= g_wp + 11'd1;
                        byte_odd   <= 1'b0;
                    end
                end

                if (sec_read_end) begin
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
                        o_div   <= 1'b0;
                        st      <= S_OUT;
                    end
                end
            end

            // ================= 输出一组（5120 字）=================
            S_OUT: begin
                if (!wr_ready) begin
                    // 等写通道应答：FIFO 清零还没解除时送进去的像素会被吃掉
                end else if (o_div == 1'b0) begin
                    o_div   <= 1'b1;                           // 限速：1 字 / 2 拍
                    wr_en   <= 1'b1;
                    wr_data <= {pix888, 8'b0};

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
                                if (o_sl == GROUP_LINE - 2'd1) begin
                                    grp_grp <= grp_grp + 9'd1;
                                    if (grp_grp + 9'd1 >= GROUPS) begin
                                        st <= S_WAIT;          // 整帧发完
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
                end else begin
                    o_div <= 1'b0;
                end
            end

            // ================= 等帧写完成 =================
            S_WAIT: begin
                if (fin_pulse) begin
                    disp_buf_idx  <= write_buf_idx;            // 切到新帧
                    display_valid <= 1'b1;
                    st            <= S_PACE;   // 节拍累加器由上面的块在非 S_PACE 时清零
                end
            end

            // ================= 帧率节拍 =================
            S_PACE: begin
                if (pace_tick) begin
                    grp_grp <= 9'd0;
                    g_wp    <= 11'd0;
                    wr_buf  <= wr_buf + 2'd1;
                    write_buf_idx <= wr_buf + 2'd1;
                    wr_req_set  <= 1'b1;
                    if (frame_left > 32'd1) frame_left <= frame_left - 32'd1;
                    else                    frame_left <= 32'd0;   // 0 = 循环
                    st      <= S_GRP;
                end
            end

            default: st <= S_IDLE;
            endcase
        end
    end

endmodule