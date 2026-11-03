// JPEG byte stream -> 640x480 RGB/zero-pad frame words.
// RGB565 line storage matches the existing VID player's colour precision and
// saves a BRAM9K on EG4S20. Set LINE_RGB565=0 to retain full decoded RGB888.
// One 320x240 baseline image per start. All ports are in clk (target 50MHz).
// The consumer must commit a frame ONLY on done && error==0, after its write
// engine has drained. Pixels from a corrupt/truncated image are speculative.
// Input byte streaming and output streaming both use valid/ready handshakes.
module jpeg_video_pipe #(
    parameter [23:0] WATCHDOG_LIMIT=24'hffffff,
    parameter LINE_RGB565=1
) (
    input wire clk, rst, start,
    input wire in_valid, input wire [7:0] in_data, input wire in_last,
    output wire in_ready,
    output wire out_valid, input wire out_ready, output wire [31:0] out_data,
    output reg busy, done, output reg [15:0] error,
    output reg [18:0] words_sent
);
    wire dec_rst = rst || !busy;
    wire dec_in_ready, px_valid, px_ready, px_sof, px_eol;
    wire [15:0] px_x, px_y, img_w, img_h;
    wire [7:0] r,g,b;
    wire frame_start, frame_done;
    wire [12:0] dec_error;
    wire [2:0] unused_n;
    jpeg_decoder #(.FAST(0), .RASTER_OUT(1), .ROWBUF_BYTES(8192),
        .FANCY_UPSAMPLE(0), .CC_TURBO(0), .CHECKS(1)) decoder (
        .clk(clk), .rst(dec_rst), .in_valid(in_valid && busy),
        .in_data(in_data), .in_last(in_last), .in_ready(dec_in_ready),
        .out_fmt(2'd0), .px_valid(px_valid), .px_ready(px_ready),
        .px_x(px_x), .px_y(px_y), .px_n(unused_n),
        .px_c0(r), .px_c1(g), .px_c2(b), .px_sof(px_sof), .px_eol(px_eol),
        .img_w(img_w), .img_h(img_h), .frame_start(frame_start),
        .frame_done(frame_done), .err(dec_error));
    assign in_ready = busy && dec_in_ready;

    localparam [1:0] LOAD=0, PREFETCH=1, EMIT=2;
    reg [1:0] state;
    localparam LINE_WIDTH=LINE_RGB565 ? 16 : 24;
    reg [LINE_WIDTH-1:0] line_mem[0:319];
    reg [LINE_WIDTH-1:0] line_q;
    wire [23:0] line_rgb;
    generate if(LINE_RGB565)begin : g_565
        assign line_rgb={line_q[15:11],line_q[15:13],line_q[10:5],line_q[10:9],line_q[4:0],line_q[4:2]};
    end else begin : g_888
        assign line_rgb=line_q;
    end endgenerate
    reg [8:0] read_x, load_x;
    reg [7:0] load_y;
    reg dup_x, dup_y, decoded, drop_frame;
    reg [23:0] watchdog;
    wire accept_px = busy && px_valid && px_ready;
    wire accept_word = out_valid && out_ready;
    assign px_ready = busy && (drop_frame || state==LOAD);
    assign out_valid = busy && state==EMIT;
    assign out_data = {line_rgb,8'd0};
    always @(posedge clk) begin
        if (accept_px && !drop_frame && load_x<320)
            line_mem[load_x] <= LINE_RGB565 ? {r[7:3],g[7:2],b[7:3]} : {r,g,b};
        line_q <= line_mem[read_x];
    end
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            busy<=0;done<=0;error<=0;words_sent<=0;
            state<=LOAD;read_x<=0;load_x<=0;load_y<=0;
            dup_x<=0;dup_y<=0;decoded<=0;drop_frame<=0;watchdog<=0;
        end else begin
            done<=0;
            if (!busy) begin
                if (start) begin
                    busy<=1;error<=0;words_sent<=0;state<=LOAD;
                    read_x<=0;load_x<=0;load_y<=0;dup_x<=0;dup_y<=0;
                    decoded<=0;drop_frame<=0;watchdog<=0;
                end
            end else begin
                error <= error | {3'd0,dec_error};
                // Timeout pauses while output is explicitly backpressured.
                // Reset at the system boundary may abort any held output word.
                if (accept_px || accept_word || (in_valid && in_ready)) watchdog<=0;
                else if (!out_valid || out_ready) watchdog<=watchdog+1'b1;
                if (watchdog==WATCHDOG_LIMIT) begin
                    busy<=0;done<=1;error<=error | 16'h8000;state<=LOAD;
                end else begin
                    if (frame_start && (img_w!=320 || img_h!=240)) begin
                        drop_frame<=1;error<=error | 16'h2000;
                    end
                    if (frame_done) decoded<=1;
                    if (accept_px && !drop_frame) begin
                        if (px_x!=load_x || px_y!={8'd0,load_y} ||
                            px_sof!=(load_x==0 && load_y==0) || px_eol!=(load_x==319))
                            error<=error | {3'd0,dec_error} | 16'h4000;
                        if (load_x==319) begin
                            load_x<=0;load_y<=load_y+1'b1;
                            read_x<=0;dup_x<=0;dup_y<=0;state<=PREFETCH;
                        end else load_x<=load_x+1'b1;
                    end
                    if (state==PREFETCH) state<=EMIT;
                    if (accept_word) begin
                        words_sent<=words_sent+1'b1;
                        if (!dup_x) dup_x<=1;
                        else begin
                            dup_x<=0;
                            if (read_x==319) begin
                                read_x<=0;
                                if (!dup_y) begin dup_y<=1;state<=PREFETCH;end
                                else begin dup_y<=0;state<=LOAD;end
                            end else begin read_x<=read_x+1'b1;state<=PREFETCH;end
                        end
                    end
                    if (decoded && state==LOAD) begin
                        busy<=0;done<=1;
                        if (error==0 && words_sent!=307200) error<=16'h4000;
                    end
                end
            end
        end
    end
endmodule
