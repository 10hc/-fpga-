// ============================================================
// tb_video.v —— video_player.v 仿真测试平台（自包含，不需要工程其他模块）
//
// 做法：
//   1) 造 2 帧假 .VID 数据在内存里（像素值 = (y*320+x) & 0xFFFF）
//   2) 模拟 SD 扇区读：sec_read 拉高 → 送 512 字节 + data_valid + end
//   3) 模拟帧写通道：wr_req → ack；收 wr_en 数据，每帧收满 307200 字发 wr_finish
//   4) 逐字核对：输出第 n 个字应对应 源行=n/1280、源列=(n%640)/2 的像素 2 倍展开
// 期望：RESULT: frames=2 words=... ok=... bad=0
// ============================================================
`timescale 1ns/1ps

module tb_video;

    localparam integer CLK_HZ   = 100_000_000;
    localparam integer SRC_W    = 320;
    localparam integer SRC_H    = 240;
    localparam integer FRAMES   = 2;
    localparam integer SEC_PER_FRAME = 300;             // 153600 / 512
    localparam integer TOTAL_SEC     = 1 + FRAMES*SEC_PER_FRAME;
    localparam integer WORDS_FRAME   = 640*480;         // 307200
    localparam integer WORDS_GROUP   = 5120;
    // 仿真加速：头部 fps 写成 2000（真机是 25），节拍 0.5ms 而不是 40ms，
    // 否则跑 2 帧要 9 千万拍。节拍逻辑本身照样被完整执行到。
    localparam integer FPS_SIM       = 2000;
    // 整型参数不能做位选（Verilog-2001），转成 16 位再取字节
    localparam [15:0] SRC_W16   = SRC_W;
    localparam [15:0] SRC_H16   = SRC_H;
    localparam [15:0] FPS16     = FPS_SIM;
    localparam [15:0] FRAMES16  = FRAMES;

    reg clk = 0;
    reg rst = 1;
    reg enable = 0;
    always #5 clk = ~clk;                               // 100MHz

    // ---------------- DUT ----------------
    wire        sec_read;
    wire [31:0] sec_read_addr;
    reg  [7:0]  sec_read_data;
    reg         sec_read_data_valid;
    reg         sec_read_end;

    wire        wr_req;
    reg         wr_req_ack;
    wire        wr_en;
    wire [31:0] wr_data;
    reg         wr_finish;

    wire        display_valid;
    wire [1:0]  disp_buf_idx;
    wire [1:0]  write_buf_idx;
    wire        active;

    video_player #(
        .CLK_FREQ_HZ (CLK_HZ),
        .SCAN_START  (32'd0),
        .SCAN_MAX    (32'd131071),
        .SRC_W       (SRC_W),
        .SRC_H       (SRC_H)
    ) dut (
        .clk                 (clk),
        .rst                 (rst),
        .enable              (enable),
        .sec_read            (sec_read),
        .sec_read_addr       (sec_read_addr),
        .sec_read_data       (sec_read_data),
        .sec_read_data_valid (sec_read_data_valid),
        .sec_read_end        (sec_read_end),
        .wr_req              (wr_req),
        .wr_req_ack          (wr_req_ack),
        .wr_en               (wr_en),
        .wr_data             (wr_data),
        .wr_finish           (wr_finish),
        .display_valid       (display_valid),
        .disp_buf_idx        (disp_buf_idx),
        .write_buf_idx       (write_buf_idx),
        .active              (active)
    );

    // ---------------- 假 .VID 存储 ----------------
    reg [7:0] mem [0:TOTAL_SEC*512-1];

    function [15:0] pix_of(input integer x, input integer y);
        pix_of = (y*SRC_W + x) & 16'hFFFF;
    endfunction

    integer f, x, y, off, sec, bo, k;
    reg [15:0] pv;
    initial begin
        // 头部扇区
        mem[0] = "V"; mem[1] = "I"; mem[2] = "D"; mem[3] = "1";
        mem[4] = SRC_W16[7:0];   mem[5]  = SRC_W16[15:8];
        mem[6] = SRC_H16[7:0];   mem[7]  = SRC_H16[15:8];
        mem[8] = FPS16[7:0];     mem[9]  = FPS16[15:8];   // fps（仿真用 2000）
        mem[10] = FRAMES16[7:0]; mem[11] = 8'd0; mem[12] = 8'd0; mem[13] = 8'd0;
        for (k = 14; k < 512; k = k + 1) mem[k] = 8'd0;

        // 帧数据
        for (f = 0; f < FRAMES; f = f + 1)
            for (y = 0; y < SRC_H; y = y + 1)
                for (x = 0; x < SRC_W; x = x + 1) begin
                    off = (y*SRC_W + x) * 2;
                    sec = 1 + f*SEC_PER_FRAME + off/512;
                    bo  = off % 512;
                    pv  = pix_of(x, y);
                    mem[sec*512 + bo]     = pv[7:0];
                    mem[sec*512 + bo + 1] = pv[15:8];
                end
        $display("[TB] 假 .VID 就绪: %0d 帧, %0d 扇区", FRAMES, TOTAL_SEC);
    end

    // ---------------- SD 扇区读从机 ----------------
    // 注意：DUT 在 sec_read_end 之后才把 sec_read 拉低，中间有一拍
    //       sec_read 仍为高、streaming 已为 0。若用 (!streaming && sec_read)
    //       判断，会在同一拍又重读一遍同一扇区，往后整体错位 512 字节。
    //       所以必须检测 sec_read 的**上升沿**才启动。
    reg        streaming = 0;
    reg        sec_read_d = 0;
    reg [31:0] rd_base = 0;
    reg [9:0]  rd_byte = 0;                             // 0..511 发数据, 512 发 end

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            streaming            <= 1'b0;
            sec_read_d           <= 1'b0;
            rd_byte              <= 10'd0;
            sec_read_data        <= 8'd0;
            sec_read_data_valid  <= 1'b0;
            sec_read_end         <= 1'b0;
        end else begin
            sec_read_d          <= sec_read;
            sec_read_data_valid <= 1'b0;
            sec_read_end        <= 1'b0;
            if (!streaming && sec_read && !sec_read_d) begin   // 上升沿才算新请求
                rd_base   <= sec_read_addr << 9;        // *512
                rd_byte   <= 10'd0;
                streaming <= 1'b1;
            end else if (streaming) begin
                if (rd_byte == 10'd512) begin           // 512 字节发完，下一拍给 end
                    streaming    <= 1'b0;
                    sec_read_end <= 1'b1;
                end else begin
                    sec_read_data       <= mem[rd_base + rd_byte];
                    sec_read_data_valid <= 1'b1;
                    rd_byte             <= rd_byte + 10'd1;
                end
            end
        end
    end

    // ---------------- 帧写通道从机 + 逐字核对 ----------------
    integer    wr_cnt     = 0;
    integer    ok_cnt     = 0;
    integer    bad_cnt    = 0;
    integer    frame_cnt  = 0;
    integer    grp_word   = 0;
    integer    ol, opx, sy, sx;
    reg [15:0] exp565;
    reg [31:0] exp32;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            wr_req_ack <= 1'b0;
            wr_finish  <= 1'b0;
            wr_cnt     <= 0;
            ok_cnt     <= 0;
            bad_cnt    <= 0;
            frame_cnt  <= 0;
            grp_word   <= 0;
        end else begin
            wr_finish <= 1'b0;

            if (wr_req && !wr_req_ack) wr_req_ack <= 1'b1;   // 应答，保持到帧结束

            if (wr_en) begin
                // 由输出序号反推期望像素
                ol     = wr_cnt / 640;
                opx    = wr_cnt % 640;
                sy     = ol / 2;
                sx     = opx / 2;
                exp565 = (sy*SRC_W + sx) & 16'hFFFF;
                exp32  = {exp565[15:11], exp565[15:13],
                          exp565[10:5],  exp565[10:9],
                          exp565[4:0],   exp565[4:2], 8'b0};

                if (wr_data === exp32) ok_cnt  <= ok_cnt + 1;
                else begin
                    bad_cnt <= bad_cnt + 1;
                    if (bad_cnt < 5)
                        $display("[BAD] 帧%0d 字%0d 行%0d 列%0d: 得到 %h 期望 %h (源 %0d,%0d=%h)",
                                 frame_cnt, wr_cnt, ol, opx, wr_data, exp32, sx, sy, exp565);
                end

                wr_cnt   <= wr_cnt + 1;
                grp_word <= grp_word + 1;
                if (grp_word == WORDS_GROUP-1) grp_word <= 0;

                if (wr_cnt == WORDS_FRAME-1) begin
                    wr_cnt    <= 0;
                    grp_word  <= 0;
                    frame_cnt <= frame_cnt + 1;
                    wr_finish <= 1'b1;                       // 通知 DUT 本帧写完
                    wr_req_ack <= 1'b0;
                    $display("[TB] 第 %0d 帧收满 %0d 字 (display_valid=%b disp_buf=%0d write_buf=%0d)",
                             frame_cnt+1, WORDS_FRAME, display_valid, disp_buf_idx, write_buf_idx);
                end
            end
        end
    end

    // ---------------- 主流程 ----------------
    integer guard;
    initial begin
        // 不 dump VCD：一帧 30 万字+160 万拍，波形文件会到 GB 级；
        // 出错时下面会打印前 5 个错字的完整上下文，足够定位
        rst = 1; enable = 0;
        repeat (10) @(posedge clk);
        rst = 0;
        repeat (10) @(posedge clk);
        enable = 1;

        // 跑到收满 2 帧，或超时（2 帧大约 1.3M 拍）
        guard = 0;
        while (frame_cnt < FRAMES && guard < 4_000_000) begin
            @(posedge clk);
            guard = guard + 1;
        end

        $display("--------------------------------------------------");
        if (bad_cnt == 0 && frame_cnt >= FRAMES)
            $display("RESULT: PASS frames=%0d words=%0d ok=%0d bad=%0d cycles=%0d",
                     frame_cnt, ok_cnt + bad_cnt, ok_cnt, bad_cnt, guard);
        else
            $display("RESULT: FAIL frames=%0d words=%0d ok=%0d bad=%0d cycles=%0d",
                     frame_cnt, ok_cnt + bad_cnt, ok_cnt, bad_cnt, guard);
        $display("--------------------------------------------------");
        $finish;
    end

endmodule