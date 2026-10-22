// key_short_long.v
// 按键短按/长按检测（20ms 去抖；按住超过 LONG_MS 视为长按，长按只出一次脉冲）
// 短按脉冲在"松开"时产生；长按期间不会产生短按脉冲。
module key_short_long #(
    parameter integer CLK_FREQ_HZ = 25_000_000,
    parameter integer DEBOUNCE_MS = 20,
    parameter integer LONG_MS     = 1000
)(
    input  wire clk,
    input  wire rst,              // 高有效
    input  wire button,           // 松开=1，按下=0
    output reg  short_pulse,
    output reg  long_pulse
);
    localparam integer DEB = (CLK_FREQ_HZ / 1000) * DEBOUNCE_MS;
    localparam integer LNG = (CLK_FREQ_HZ / 1000) * LONG_MS;

    reg        b0, b1;
    reg        stable;
    reg        stable_d;
    reg [31:0] db_cnt;
    reg [31:0] hold_cnt;
    reg        long_done;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            b0 <= 1'b1; b1 <= 1'b1;
            stable <= 1'b1; stable_d <= 1'b1;
            db_cnt <= 32'd0; hold_cnt <= 32'd0;
            long_done <= 1'b0;
            short_pulse <= 1'b0; long_pulse <= 1'b0;
        end else begin
            b0 <= button;
            b1 <= b0;
            short_pulse <= 1'b0;
            long_pulse  <= 1'b0;

            // ---- 去抖 ----
            if (b1 == stable)
                db_cnt <= 32'd0;
            else if (db_cnt >= DEB - 1) begin
                stable <= b1;
                db_cnt <= 32'd0;
            end else
                db_cnt <= db_cnt + 32'd1;

            stable_d <= stable;

            // ---- 按住计时 ----
            if (!stable) begin
                if (!long_done) begin
                    if (hold_cnt >= LNG - 1) begin
                        long_pulse <= 1'b1;
                        long_done  <= 1'b1;
                    end else
                        hold_cnt <= hold_cnt + 32'd1;
                end
            end else begin
                hold_cnt <= 32'd0;
            end

            // ---- 松开沿：短按或清除长按标志 ----
            if (stable && !stable_d) begin
                if (long_done)
                    long_done <= 1'b0;
                else
                    short_pulse <= 1'b1;
            end
        end
    end
endmodule
