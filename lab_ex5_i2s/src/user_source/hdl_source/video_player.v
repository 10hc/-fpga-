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
    output wire        active
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

    reg [31:0] pace_cnt;
    reg [2:0]  fin_sync;

    // 分组缓冲（推断 BRAM）
    reg [15:0] gbuf [0:1279];

    // ---------------- 组合 ----------------
    wire [31:0] pace_max  = (fps_l == 16'd0) ? 32'd4_000_000 : (CLK_FREQ_HZ / fps_l);
    wire        pace_tick = (pace_cnt >= pace_max);
    wire        fin_pulse = fin_sync[2] ^ fin_sync[1];

    wire [10:0] rd_idx  = (o_sl * 11'd320) + o_px;
    wire [15:0] pix565  = gbuf[rd_idx];
    wire [23:0] pix888  = {pix565[15:11], pix565[15:13],
                           pix565[10:5],  pix565[10:9],
                           pix565[4:0],   pix565[4:2]};

    assign active = enable && (st != S_IDLE);

    // ---------------- 帧率节拍（唯一驱动 pace_cnt，状态机里不得再赋值）----------------
    always @(posedge clk or posedge rst) begin
        if (rst)               pace_cnt <= 32'd0;
        else if (st != S_PACE) pace_cnt <= 32'd0;   // 只有 S_PACE 状态才走计数
        else if (pace_tick)    pace_cnt <= 32'd0;
        else                   pace_cnt <= pace_cnt + 32'd1;
    end

    // ---------------- wr_finish 同步（独立，只碰 fin_sync）----------------
    always @(posedge clk or posedge rst) begin
        if (rst) fin_sync <= 3'd0;
        else     fin_sync <= {fin_sync[1:0], wr_finish};
    end

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
            wr_req        <= 1'b0;
            wr_en         <= 1'b0;
            wr_data       <= 32'd0;
            disp_buf_idx  <= 2'd0;
            write_buf_idx <= 2'd0;
            display_valid <= 1'b0;
        end else begin
            wr_en <= 1'b0;

            case (st)
            // ================= 等使能 =================
            S_IDLE: begin
                sec_read <= 1'b0;
                wr_req   <= 1'b0;
                if (enable) begin
                    addr    <= SCAN_START;
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
                    end else if (addr < SCAN_MAX) begin
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
                    wr_req        <= 1'b1;
                    st            <= S_GRP;
                end
            end

            // ================= 收一组（5 扇区 = 1280 像素）=================
            S_GRP: begin
                if (wr_req && wr_req_ack) wr_req <= 1'b0;

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
                if (o_div == 1'b0) begin
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
                    st            <= S_PACE;   // pace_cnt 由节拍块在非 S_PACE 时清零
                end
            end

            // ================= 帧率节拍 =================
            S_PACE: begin
                if (pace_tick) begin
                    grp_grp <= 9'd0;
                    g_wp    <= 11'd0;
                    wr_buf  <= wr_buf + 2'd1;
                    write_buf_idx <= wr_buf + 2'd1;
                    wr_req  <= 1'b1;
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