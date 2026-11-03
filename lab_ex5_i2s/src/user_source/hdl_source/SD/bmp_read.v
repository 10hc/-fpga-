module bmp_read(
    input                       clk,
    input                       rst,
    output                      ready,

    // 上电扫描：从 scan_start_sector 开始，顺序寻找前 scan_target_count 张 BMP
    input                       scan_start,
    input  [31:0]               scan_start_sector,
    input  [31:0]               scan_max_sector,
    input  [2:0]                scan_target_count,
    output reg                  scan_done,
    output reg                  scan_found_valid,
    output reg [31:0]           scan_found_sector,
    output reg [2:0]            scan_found_total,

    // 目录解析结果（跳转模式）：已知文件首扇区时直接访问，避免逐扇区暴力扫描。
    // jump_en=1 时只访问 jump_count 个候选扇区（来自 FAT32 根目录）。
    input                       jump_en,
    input  [2:0]                jump_count,
    input  [31:0]               jump_sector0,
    input  [31:0]               jump_sector1,
    input  [31:0]               jump_sector2,
    input  [31:0]               jump_sector3,

    // 按指定扇区加载一张图到 SDRAM
    input                       load_start,
    input  [31:0]               load_sector,

    input                       sd_init_done,
    output reg [3:0]            state_code,
    input  [15:0]               bmp_width,
    input  [15:0]               bmp_height,

    output reg                  write_req,
    input                       write_req_ack,

    output                      sd_sec_read,
    output [31:0]               sd_sec_read_addr,
    input  [7:0]                sd_sec_read_data,
    input                       sd_sec_read_data_valid,
    input                       sd_sec_read_end,

    input                       bmp_data_ready,
    input                       abort,
    output reg                  load_done,
    output reg                  load_error,
    output                      bmp_data_wr_en,
    output [23:0]               bmp_data
);

localparam ST_IDLE      = 4'd0;
localparam ST_SCAN      = 4'd1;
localparam ST_LOAD_HDR  = 4'd2;
localparam ST_LOAD_WAIT = 4'd3;
localparam ST_LOAD_DATA = 4'd4;

localparam ST_CHECK_SHAPE = 4'd5, ST_CHECK_SIZE = 4'd6,
           ST_CHECK_END = 4'd7, ST_CHECK_RESULT = 4'd8,
           ST_SCAN_COMMIT = 4'd9, ST_LOAD_COMMIT = 4'd10;
reg [3:0] state;
reg checking_scan;
reg complete_latched, basic_ok, header_ok;
reg [31:0] checked_height;
reg [12:0] row_stride;
reg [24:0] image_bytes;
reg [32:0] pixels_end, dib_end, last_sector;
reg [31:0] checked_sector_count;
reg [9:0]  rd_cnt;
reg header_req;
reg [31:0] header_addr;
reg [31:0] dib_size;
reg [15:0] planes;
reg cancel_header;
reg [1:0] ack_sync;
// Memory holds ACK until write_req is withdrawn; consume only local stage 2.
always @(posedge clk or posedge rst) begin
    if (rst) ack_sync <= 2'b00;
    else ack_sync <= {ack_sync[0], write_req_ack};
end

reg [7:0]  header_0;
reg [7:0]  header_1;
reg [31:0] file_len;
reg [31:0] pixel_offset;
reg [31:0] width;
reg [31:0] height;
reg [15:0] bit_count;
reg [31:0] compression;

reg [31:0] scan_sector;
reg [2:0]  jump_idx;                // 跳转模式：当前访问到第几个候选文件
reg [31:0] load_sector_latched;

wire header_match;
wire [31:0] file_sector_count;
wire [31:0] next_scan_sector_if_match;
wire [31:0] next_scan_sector_if_miss;

assign ready = (state == ST_IDLE);
wire [31:0] abs_height = height[31] ? (~height+32'd1) : height;
wire [31:0] header_sector = state == ST_SCAN ? scan_sector : load_sector_latched;
wire header_complete = rd_cnt + (sd_sec_read_data_valid ? 10'd1 : 10'd0) == 10'd512;
assign header_match = header_ok;
assign file_sector_count = checked_sector_count;
assign next_scan_sector_if_match = scan_sector + file_sector_count;
assign next_scan_sector_if_miss  = scan_sector + 32'd1;

// 跳转模式的候选扇区选择（纯 32 位四选一，不涉及运算，时序无压力）
wire [31:0] jump_sector_at = (jump_idx == 3'd0) ? jump_sector0 :
                             (jump_idx == 3'd1) ? jump_sector1 :
                             (jump_idx == 3'd2) ? jump_sector2 : jump_sector3;
wire [31:0] jump_sector_nx = (jump_idx == 3'd0) ? jump_sector1 :
                             (jump_idx == 3'd1) ? jump_sector2 :
                             (jump_idx == 3'd2) ? jump_sector3 : jump_sector0;
wire        jump_last      = (jump_idx + 3'd1 >= jump_count);

wire scale_req, scale_valid, scale_done, scale_error;
wire [31:0] scale_addr;
reg scale_start;
assign sd_sec_read = state == ST_LOAD_DATA ? scale_req : header_req;
assign sd_sec_read_addr = state == ST_LOAD_DATA ? scale_addr : header_addr;
assign bmp_data_wr_en = scale_valid && bmp_data_ready;
bmp_scale u_scale(
    .clk(clk), .rst(rst || !sd_init_done), .start(scale_start), .abort(abort),
    .src_w(width[11:0]), .src_h(abs_height[11:0]), .top_down(height[31]),
    .pixel_offset(pixel_offset), .base_sector(load_sector_latched),
    .out_ready(bmp_data_ready), .out_valid(scale_valid), .out_rgb(bmp_data),
    .busy(), .done(scale_done), .error(scale_error),
    .sd_req(scale_req), .sd_addr(scale_addr), .sd_data(sd_sec_read_data),
    .sd_valid(sd_sec_read_data_valid), .sd_end(sd_sec_read_end)
);

always @(posedge clk or posedge rst) begin
    if (rst) begin
        rd_cnt <= 10'd0;
    end else if ((state == ST_SCAN) || (state == ST_LOAD_HDR)) begin
        if (sd_sec_read_end)
            rd_cnt <= 10'd0;
        else if (sd_sec_read_data_valid && rd_cnt < 10'd513)
            rd_cnt <= rd_cnt + 10'd1;
    end else begin
        rd_cnt <= 10'd0;
    end
end

always @(posedge clk or posedge rst) begin
    if (rst) begin
        header_0     <= 8'd0;
        header_1     <= 8'd0;
        file_len     <= 32'd0;
        pixel_offset <= 32'd54;
        width        <= 32'd0;
        height       <= 32'd0;
        bit_count    <= 16'd0;
        compression  <= 32'd0;
        dib_size <= 0;
        planes <= 0;
    end else if (((state == ST_SCAN) || (state == ST_LOAD_HDR)) && sd_sec_read_data_valid) begin
        case (rd_cnt)
            10'd0 : header_0 <= sd_sec_read_data;
            10'd1 : header_1 <= sd_sec_read_data;

            10'd2 : file_len[7:0] <= sd_sec_read_data;
            10'd3 : file_len[15:8] <= sd_sec_read_data;
            10'd4 : file_len[23:16] <= sd_sec_read_data;
            10'd5 : file_len[31:24] <= sd_sec_read_data;

            10'd10: pixel_offset[7:0] <= sd_sec_read_data;
            10'd11: pixel_offset[15:8] <= sd_sec_read_data;
            10'd12: pixel_offset[23:16] <= sd_sec_read_data;
            10'd13: pixel_offset[31:24] <= sd_sec_read_data;

            10'd14: dib_size[7:0] <= sd_sec_read_data;
            10'd15: dib_size[15:8] <= sd_sec_read_data;
            10'd16: dib_size[23:16] <= sd_sec_read_data;
            10'd17: dib_size[31:24] <= sd_sec_read_data;
            10'd26: planes[7:0] <= sd_sec_read_data;
            10'd27: planes[15:8] <= sd_sec_read_data;

            10'd18: width[7:0] <= sd_sec_read_data;
            10'd19: width[15:8] <= sd_sec_read_data;
            10'd20: width[23:16] <= sd_sec_read_data;
            10'd21: width[31:24] <= sd_sec_read_data;

            10'd22: height[7:0] <= sd_sec_read_data;
            10'd23: height[15:8] <= sd_sec_read_data;
            10'd24: height[23:16] <= sd_sec_read_data;
            10'd25: height[31:24] <= sd_sec_read_data;

            10'd28: bit_count[7:0] <= sd_sec_read_data;
            10'd29: bit_count[15:8] <= sd_sec_read_data;

            10'd30: compression[7:0] <= sd_sec_read_data;
            10'd31: compression[15:8] <= sd_sec_read_data;
            10'd32: compression[23:16] <= sd_sec_read_data;
            10'd33: compression[31:24] <= sd_sec_read_data;
            default: ;
        endcase
    end
end

always @(posedge clk or posedge rst) begin
    if (rst) begin
        state             <= ST_IDLE;
        scale_start <= 0;
        load_done <= 0;
        load_error <= 0;
        cancel_header <= 0;
        checking_scan <= 0;
        complete_latched <= 0;
        basic_ok <= 0;
        header_ok <= 0;
        checked_height <= 0;
        row_stride <= 0;
        image_bytes <= 0;
        pixels_end <= 0;
        dib_end <= 0;
        last_sector <= 0;
        checked_sector_count <= 0;
        jump_idx <= 0;
        state_code        <= 4'd0;
        header_req       <= 1'b0;
        header_addr  <= 32'd0;
        write_req         <= 1'b0;
        scan_done         <= 1'b0;
        scan_found_valid  <= 1'b0;
        scan_found_sector <= 32'd0;
        scan_found_total  <= 3'd0;
        scan_sector       <= 32'd0;
        load_sector_latched <= 32'd0;
    end else if (!sd_init_done) begin
        state             <= ST_IDLE;
        scale_start <= 0;
        load_done <= 0;
        load_error <= 0;
        cancel_header <= 0;
        checking_scan <= 0;
        complete_latched <= 0;
        basic_ok <= 0;
        header_ok <= 0;
        checked_height <= 0;
        row_stride <= 0;
        image_bytes <= 0;
        pixels_end <= 0;
        dib_end <= 0;
        last_sector <= 0;
        checked_sector_count <= 0;
        state_code        <= 4'd0;
        header_req       <= 1'b0;
        header_addr  <= 32'd0;
        write_req         <= 1'b0;
        scan_done         <= 1'b0;
        scan_found_valid  <= 1'b0;
        scan_found_sector <= 32'd0;
        scan_found_total  <= 3'd0;
        scan_sector       <= 32'd0;
        load_sector_latched <= 32'd0;
    end else begin
        scan_found_valid <= 1'b0;
        scale_start <= 0;
        load_done <= 0;
        load_error <= 0;
        if (abort && (state == ST_LOAD_HDR ||
            (!checking_scan && state >= ST_CHECK_SHAPE))) cancel_header <= 1;

        case (state)
            ST_IDLE: begin
                state_code  <= 4'd1;
                header_req <= 1'b0;
                write_req   <= 1'b0;

                if (scan_start) begin
                    scan_done        <= 1'b0;
                    scan_found_total <= 3'd0;
                    jump_idx         <= 3'd0;
                    scan_sector      <= jump_en ? jump_sector0 : scan_start_sector;
                    header_addr <= jump_en ? jump_sector0 : scan_start_sector;
                    state            <= ST_SCAN;
                end else if (load_start) begin
                    cancel_header <= 0;
                    load_sector_latched <= load_sector;
                    header_addr    <= load_sector;
                    state               <= ST_LOAD_HDR;
                end
            end

            ST_SCAN: begin
                state_code  <= 4'd2;
                header_req <= 1'b1;

                if (sd_sec_read_end) begin
                    header_req <= 1'b0;

                    checking_scan <= 1;
                    complete_latched <= header_complete;
                    state <= ST_CHECK_SHAPE;
                end
            end

            ST_LOAD_HDR: begin
                state_code <= 4'd2;
                header_req <= 1;
                if (sd_sec_read_end) begin
                    header_req <= 0;
                    checking_scan <= 0;
                    complete_latched <= header_complete;
                    state <= ST_CHECK_SHAPE;
                end
            end

            // No memory write or next SD request can start until all checks
            // have crossed registers. Never false-path the validation chain.
            ST_CHECK_SHAPE: begin
                checked_height <= abs_height;
                row_stride <= ({1'b0,width[11:0]}*13'd3 + 13'd3) & 13'h1ffc;
                checked_sector_count <= (file_len >> 9) + (file_len[8:0] != 0);
                basic_ok <= complete_latched && header_0 == "B" && header_1 == "M" &&
                    bmp_width == 640 && bmp_height == 480 &&
                    width >= 1 && width <= 2048 && dib_size >= 40 &&
                    planes == 1 && bit_count == 24 && compression == 0;
                state <= ST_CHECK_SIZE;
            end
            ST_CHECK_SIZE: begin
                image_bytes <= row_stride * checked_height[11:0];
                dib_end <= {1'b0,dib_size} + 33'd14;
                state <= ST_CHECK_END;
            end
            ST_CHECK_END: begin
                pixels_end <= {1'b0,pixel_offset} + {8'd0,image_bytes};
                last_sector <= {1'b0,(checking_scan ? scan_sector : load_sector_latched)} +
                               {1'b0,checked_sector_count};
                state <= ST_CHECK_RESULT;
            end
            ST_CHECK_RESULT: begin
                header_ok <= basic_ok && checked_height >= 1 && checked_height <= 2048 &&
                    dib_end <= {1'b0,pixel_offset} && pixels_end <= {1'b0,file_len} &&
                    last_sector <= 33'h100000000;
                state <= checking_scan ? ST_SCAN_COMMIT : ST_LOAD_COMMIT;
            end
            ST_SCAN_COMMIT: begin
                // Default return to header issue; completion branches override.
                state <= ST_SCAN;
                    if (jump_en) begin
                        // ===== 跳转模式：只访问目录里列出的那些扇区，秒级完成 =====
                        jump_idx <= jump_idx + 3'd1;
                        if (header_match) begin
                            scan_found_valid  <= 1'b1;
                            scan_found_sector <= scan_sector;
                            scan_found_total  <= scan_found_total + 3'd1;
                        end
                        if (jump_last) begin
                            scan_done <= 1'b1;
                            state     <= ST_IDLE;
                        end else begin
                            header_addr <= jump_sector_nx;
                            scan_sector      <= jump_sector_nx;
                        end
                    end else if (header_match) begin
                        scan_found_valid  <= 1'b1;
                        scan_found_sector <= scan_sector;
                        scan_found_total  <= scan_found_total + 3'd1;

                        if ((scan_found_total + 3'd1 >= scan_target_count) || (next_scan_sector_if_match > scan_max_sector)) begin
                            scan_done        <= 1'b1;
                            state            <= ST_IDLE;
                            header_addr <= next_scan_sector_if_match;
                            scan_sector      <= next_scan_sector_if_match;
                        end else begin
                            header_addr <= next_scan_sector_if_match;
                            scan_sector      <= next_scan_sector_if_match;
                        end
                    end else begin
                        if (scan_sector >= scan_max_sector) begin
                            scan_done <= 1'b1;
                            state     <= ST_IDLE;
                        end else begin
                            header_addr <= next_scan_sector_if_miss;
                            scan_sector      <= next_scan_sector_if_miss;
                        end
                    end
            end
            ST_LOAD_COMMIT: begin
                    if (header_match && !cancel_header && !abort) begin
                        write_req <= 1'b1;
                        header_addr <= load_sector_latched;
                        state <= ST_LOAD_WAIT;
                    end else begin
                        state <= ST_IDLE;
                        load_error <= 1;
                        cancel_header <= 0;
                    end
            end

            ST_LOAD_WAIT: begin
                state_code <= 4'd3;
                if (abort) begin
                    write_req <= 0;
                    load_error <= 1;
                    state <= ST_IDLE;
                end else if (ack_sync[1]) begin
                    write_req <= 1'b0;
                    scale_start <= 1;
                    state     <= ST_LOAD_DATA;
                end
            end

            ST_LOAD_DATA: begin
                state_code  <= 4'd4;
                header_req <= 1'b0;
                if (scale_done || scale_error) begin
                    load_done <= scale_done;
                    load_error <= scale_error;
                    state <= ST_IDLE;
                end
            end

            default: begin
                state       <= ST_IDLE;
                state_code  <= 4'd1;
                header_req <= 1'b0;
                write_req   <= 1'b0;
            end
        endcase
    end
end

endmodule
