`timescale 1ns/1ps
// tb_wav.v —— 验证 wav_audio：扫描 RIFF → 解析头 → 流式读 → 组装 → 输出
module tb_wav;

    reg sd_clk = 0, vid_clk = 0, rst = 1;
    always #5  sd_clk  = ~sd_clk;      // 100MHz（仅为加快仿真）
    always #5  vid_clk = ~vid_clk;

    reg  enable = 0;
    wire sec_read;
    wire [31:0] sec_read_addr;
    reg  [7:0]  sec_read_data;
    reg         sec_read_data_valid;
    reg         sec_read_end;
    wire        aud_valid;
    wire [23:0] aud_left, aud_right;
    wire        playing;

    wav_audio #(
        .SD_CLK_HZ   (1000),
        .VID_CLK_HZ  (1000),
        .SAMPLE_RATE (500),
        .SCAN_START  (32'd0),
        .SCAN_MAX    (32'd20)
    ) dut (
        .sd_clk(sd_clk), .sd_rst(rst),
        .vid_clk(vid_clk), .vid_rst(rst),
        .enable(enable),
        .sec_read(sec_read), .sec_read_addr(sec_read_addr),
        .sec_read_data(sec_read_data), .sec_read_data_valid(sec_read_data_valid),
        .sec_read_end(sec_read_end),
        .aud_valid(aud_valid), .aud_left(aud_left), .aud_right(aud_right),
        .playing(playing)
    );

    // ================= 假 SD 卡：按扇区号返回内容 =================
    integer i, req_cnt = 0;
    integer sample_cnt = 0;
    reg [15:0] l_exp, r_exp;
    reg [31:0] last_addr;

    // 扇区内容生成：0~4 垃圾；5 = WAV头；6 起 = 数据（已知递增样本）
    function [7:0] sector_byte(input [31:0] sec, input integer off);
        begin
            if (sec == 32'd5) begin
                // ---- 44 字节标准 WAV 头 ----
                case (off)
                    0: sector_byte = "R"; 1: sector_byte = "I";
                    2: sector_byte = "F"; 3: sector_byte = "F";
                    4: sector_byte = 8'h24;   // 文件大小（随便）
                    5: sector_byte = 8'h00; 6: sector_byte = 8'h00; 7: sector_byte = 8'h00;
                    8: sector_byte = "W"; 9: sector_byte = "A";
                   10: sector_byte = "V"; 11: sector_byte = "E";
                   12: sector_byte = "f"; 13: sector_byte = "m"; 14: sector_byte = "t"; 15: sector_byte = " ";
                   16: sector_byte = 8'd16; 17: sector_byte = 8'd0; 18: sector_byte = 8'd0; 19: sector_byte = 8'd0;
                   20: sector_byte = 8'd1;  21: sector_byte = 8'd0;      // PCM
                   22: sector_byte = 8'd2;  23: sector_byte = 8'd0;      // 2 声道
                   24: sector_byte = 8'h80; 25: sector_byte = 8'hBB;     // 48000 = 0xBB80
                   26: sector_byte = 8'd0;  27: sector_byte = 8'd0;
                   28: sector_byte = 8'h00; 29: sector_byte = 8'hEE;     // 字节率（随便）
                   30: sector_byte = 8'h02; 31: sector_byte = 8'd0;
                   32: sector_byte = 8'd4;  33: sector_byte = 8'd0;      // 块对齐
                   34: sector_byte = 8'd16; 35: sector_byte = 8'd0;      // 16bit
                   36: sector_byte = "d"; 37: sector_byte = "a";
                   38: sector_byte = "t"; 39: sector_byte = "a";
                   40: sector_byte = 8'h00; 41: sector_byte = 8'h10;     // data 大小 = 0x1000 = 4096
                   42: sector_byte = 8'd0;  43: sector_byte = 8'd0;
                   44: sector_byte = 8'h34; 45: sector_byte = 8'h12;     // 首个 L = 0x1234
                   46: sector_byte = 8'h78; 47: sector_byte = 8'h56;     // 首个 R = 0x5678
                   default: sector_byte = 8'h00;
                endcase
            end else if (sec >= 32'd6) begin
                // 数据扇区：每 4 字节一个 {R,L}，值递增便于核对
                sector_byte = (sec * 16 + off[7:0]);   // 任意可预测值
            end else begin
                sector_byte = 8'hFF;                   // 垃圾扇区
            end
        end
    endfunction

    // 模拟扇区读：sec_read 拉高后逐字节送出，最后给 end
    integer k;
    always @(posedge sd_clk) begin
        if (rst) begin
            sec_read_data_valid <= 1'b0;
            sec_read_end        <= 1'b0;
        end else begin
            sec_read_data_valid <= 1'b0;
            sec_read_end        <= 1'b0;
            if (sec_read && !sec_read_data_valid && !sec_read_end) begin
                // 收到请求：分 512 拍送出（用简单计数）
                if (k < 512) begin
                    sec_read_data       <= sector_byte(sec_read_addr, k);
                    sec_read_data_valid <= 1'b1;
                    k <= k + 1;
                end else begin
                    sec_read_end <= 1'b1;
                    k <= 0;
                    req_cnt = req_cnt + 1;
                    last_addr = sec_read_addr;
                end
            end
        end
    end

    // 检查输出样本
    always @(posedge vid_clk) begin
        if (!rst && aud_valid) begin
            sample_cnt = sample_cnt + 1;
            if (sample_cnt <= 3)
                $display("RESULT 样本%0d: L=%h R=%h", sample_cnt, aud_left, aud_right);
        end
    end

    initial begin
        k = 0;
        repeat (10) @(posedge sd_clk);
        rst = 0;
        repeat (5) @(posedge sd_clk);
        enable = 1;
        repeat (20000) @(posedge sd_clk);
        $display("RESULT: playing=%b 请求扇区数=%0d 最后地址=%0d 输出样本数=%0d",
                 playing, req_cnt, last_addr, sample_cnt);
        $stop;
    end
endmodule