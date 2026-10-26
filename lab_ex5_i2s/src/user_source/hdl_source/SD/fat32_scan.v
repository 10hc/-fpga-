// ============================================================
// fat32_scan.v
// FAT32 根目录扫描器（只读、单分区）
//
// 作用：
//   1) 读 LBA0 的 BPB，解析 bytes/sector、sectors/cluster、
//      reserved sectors、FAT 数量、FAT 大小、根目录起始簇；
//   2) 沿根目录簇链读取 32 字节目录项，收集最多 4 个 BMP
//      和 1 个 WAV 的起始扇区；
//   3) 只处理 8.3 短文件名；LFN、目录、卷标自动跳过。
//
// 重要限制：
//   本模块只负责“目录解析”。BMP/WAV 读取器仍然按连续扇区
//   读取，因此文件必须连续存放（sync_to_sd.py 会按顺序写入）。
//   如后续要支持碎片文件，需要把起始簇信息扩展成簇链表读取。
// ============================================================
module fat32_scan #(
    parameter [2:0] MAX_BMP_COUNT = 3'd4
)(
    input  wire        clk,
    input  wire        rst,
    input  wire        start,
    input  wire        sd_init_done,

    output reg         busy,
    output reg         done,
    output reg         error,

    // SD 扇区读客户端
    output reg         sd_req,
    output reg  [31:0] sd_addr,
    input  wire [7:0]  sd_data,
    input  wire        sd_data_valid,
    input  wire        sd_end,

    // 扫描结果
    output reg  [2:0]  bmp_count,
    output reg  [31:0] bmp_sector0,
    output reg  [31:0] bmp_sector1,
    output reg  [31:0] bmp_sector2,
    output reg  [31:0] bmp_sector3,
    output reg         wav_valid,
    output reg  [31:0] wav_sector,
    output reg         vid_valid,
    output reg  [31:0] vid_sector
);

    localparam [3:0] ST_IDLE      = 4'd0;
    localparam [3:0] ST_BOOT      = 4'd1;
    localparam [3:0] ST_BOOT_CHECK= 4'd2;
    localparam [3:0] ST_ROOT_SETUP= 4'd3;
    localparam [3:0] ST_ROOT      = 4'd4;
    localparam [3:0] ST_ROOT_NEXT = 4'd5;
    localparam [3:0] ST_FAT       = 4'd6;
    localparam [3:0] ST_FAT_NEXT  = 4'd7;
    localparam [3:0] ST_FAT_CHECK = 4'd8;
    localparam [3:0] ST_FAT_OFFSET= 4'd9;
    localparam [3:0] ST_FAT_SECTOR= 4'd10;
    localparam [3:0] ST_FINISH    = 4'd11;
    localparam [3:0] ST_FAIL      = 4'd12;

    localparam [31:0] FAT_EOC_MIN = 32'h0FFFFFF8;
    localparam [31:0] FAT_BAD     = 32'h0FFFFFF7;
    localparam [31:0] TIMEOUT_MAX = 32'd500_000_000; // 100MHz 下约 5s

    reg [3:0]  state;

    // BPB
    reg [15:0] bytes_per_sector;
    reg [7:0]  sectors_per_cluster;
    reg [15:0] reserved_sectors;
    reg [7:0]  num_fats;
    reg [31:0] fat_size;
    reg [31:0] root_cluster;
    reg [7:0]  sig_low;
    reg        sig_ok;
    reg [31:0] boot_base;
    reg        retry_boot;
    reg [31:0] mbr_lba;
    reg        mbr_candidate;
    reg        mbr_found;

    // 布局
    reg [31:0] fat_start;
    reg [31:0] data_start;
    reg [2:0]  spc_shift;
    reg [31:0] dir_cluster;
    reg [31:0] next_cluster;
    reg [31:0] dir_offset;
    reg [31:0] dir_sector;
    reg [7:0]  dir_sec_left;

    // 目录项捕获
    reg [9:0]  entry_byte;
    reg [7:0]  ext0, ext1, ext2;
    reg [7:0]  attr;
    reg [15:0] cluster_hi;
    reg [15:0] cluster_lo;
    reg [31:0] file_size;
    reg        entry_end;
    reg        entry_deleted;
    reg        entry_pending;
    reg        entry_is_bmp_pending;
    reg        entry_is_wav_pending;
    reg        entry_is_vid_pending;
    reg [31:0] entry_offset_pending;

    // FAT 项捕获
    reg [31:0] fat_sector;
    reg [8:0]  fat_byte_idx;
    reg [9:0]  fat_byte_cnt;
    reg [7:0]  fat_b0, fat_b1, fat_b2, fat_b3;

    // 超时
    reg [31:0] timeout_cnt;

    reg [31:0] fat_area_calc;
    always @(*) begin
        // FAT32 的 num_fats 通常为 1 或 2；用移位/加法代替 32x8 乘法器。
        case (num_fats)
            8'd1: fat_area_calc = fat_size;
            8'd2: fat_area_calc = fat_size << 1;
            8'd3: fat_area_calc = fat_size + (fat_size << 1);
            8'd4: fat_area_calc = fat_size << 2;
            default: fat_area_calc = 32'd0;
        endcase
    end
    wire [31:0] data_start_calc = reserved_sectors + fat_area_calc;

    wire [31:0] entry_cluster_now = {cluster_hi, cluster_lo};
    wire [31:0] entry_size_now =
        {sd_data, file_size[23:16], file_size[15:8], file_size[7:0]};
    wire [31:0] entry_sector_calc = data_start + entry_offset_pending;
    wire [31:0] fat_value_w =
        ({fat_b3, fat_b2, fat_b1, fat_b0} & 32'h0FFFFFFF);
    wire bpb_valid =
        sig_ok &&
        (bytes_per_sector == 16'd512) &&
        (sectors_per_cluster != 8'd0) &&
        ((sectors_per_cluster & (sectors_per_cluster - 8'd1)) == 8'd0) &&
        (reserved_sectors != 16'd0) &&
        (num_fats >= 8'd1) && (num_fats <= 8'd4) &&
        (fat_size != 32'd0) &&
        (root_cluster >= 32'd2) && (root_cluster < FAT_BAD);

    wire is_bmp_ext =
        ((ext0 == 8'h42) || (ext0 == 8'h62)) &&
        ((ext1 == 8'h4D) || (ext1 == 8'h6D)) &&
        ((ext2 == 8'h50) || (ext2 == 8'h70));
    wire is_wav_ext =
        ((ext0 == 8'h57) || (ext0 == 8'h77)) &&
        ((ext1 == 8'h41) || (ext1 == 8'h61)) &&
        ((ext2 == 8'h56) || (ext2 == 8'h76));
    wire is_vid_ext =
        ((ext0 == 8'h56) || (ext0 == 8'h76)) &&
        ((ext1 == 8'h49) || (ext1 == 8'h69)) &&
        ((ext2 == 8'h44) || (ext2 == 8'h64));

    function [2:0] spc_shift_func;
        input [7:0] spc;
        begin
            case (spc)
                8'd1:   spc_shift_func = 3'd0;
                8'd2:   spc_shift_func = 3'd1;
                8'd4:   spc_shift_func = 3'd2;
                8'd8:   spc_shift_func = 3'd3;
                8'd16:  spc_shift_func = 3'd4;
                8'd32:  spc_shift_func = 3'd5;
                8'd64:  spc_shift_func = 3'd6;
                8'd128: spc_shift_func = 3'd7;
                default: spc_shift_func = 3'd0;
            endcase
        end
    endfunction

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state             <= ST_IDLE;
            busy              <= 1'b0;
            done              <= 1'b0;
            error             <= 1'b0;
            sd_req            <= 1'b0;
            sd_addr           <= 32'd0;
            bmp_count         <= 3'd0;
            bmp_sector0       <= 32'd0;
            bmp_sector1       <= 32'd0;
            bmp_sector2       <= 32'd0;
            bmp_sector3       <= 32'd0;
            wav_valid         <= 1'b0;
            wav_sector        <= 32'd0;
            vid_valid         <= 1'b0;
            vid_sector        <= 32'd0;
            bytes_per_sector  <= 16'd0;
            sectors_per_cluster <= 8'd0;
            reserved_sectors  <= 16'd0;
            num_fats          <= 8'd0;
            fat_size          <= 32'd0;
            root_cluster      <= 32'd0;
            sig_low           <= 8'd0;
            sig_ok            <= 1'b0;
            boot_base         <= 32'd0;
            retry_boot        <= 1'b0;
            mbr_lba           <= 32'd0;
            mbr_candidate     <= 1'b0;
            mbr_found         <= 1'b0;
            fat_start         <= 32'd0;
            data_start        <= 32'd0;
            spc_shift         <= 3'd0;
            dir_cluster       <= 32'd0;
            next_cluster      <= 32'd0;
            dir_offset        <= 32'd0;
            dir_sector        <= 32'd0;
            dir_sec_left      <= 8'd0;
            entry_byte        <= 10'd0;
            ext0              <= 8'd0;
            ext1              <= 8'd0;
            ext2              <= 8'd0;
            attr              <= 8'd0;
            cluster_hi        <= 16'd0;
            cluster_lo        <= 16'd0;
            file_size         <= 32'd0;
            entry_end         <= 1'b0;
            entry_deleted     <= 1'b0;
            entry_pending     <= 1'b0;
            entry_is_bmp_pending <= 1'b0;
            entry_is_wav_pending <= 1'b0;
            entry_is_vid_pending <= 1'b0;
            entry_offset_pending <= 32'd0;
            fat_sector        <= 32'd0;
            fat_byte_idx      <= 9'd0;
            fat_byte_cnt      <= 10'd0;
            fat_b0            <= 8'd0;
            fat_b1            <= 8'd0;
            fat_b2            <= 8'd0;
            fat_b3            <= 8'd0;
            timeout_cnt       <= 32'd0;
        end else if (!sd_init_done) begin
            state             <= ST_IDLE;
            busy              <= 1'b0;
            done              <= 1'b0;
            error             <= 1'b0;
            sd_req            <= 1'b0;
            bmp_count         <= 3'd0;
            wav_valid         <= 1'b0;
            timeout_cnt       <= 32'd0;
        end else begin
            // 超时保护：任何阶段超过 5s 没有扇区结束就失败
            if ((state != ST_IDLE) && (state != ST_FINISH) && (state != ST_FAIL)) begin
                if (sd_end)
                    timeout_cnt <= 32'd0;
                else if (timeout_cnt < TIMEOUT_MAX)
                    timeout_cnt <= timeout_cnt + 32'd1;
            end else begin
                timeout_cnt <= 32'd0;
            end

            case (state)
                ST_IDLE: begin
                    done  <= 1'b0;
                    error <= error;
                    sd_req <= 1'b0;
                    if (start && !busy) begin
                        busy          <= 1'b1;
                        done          <= 1'b0;
                        error         <= 1'b0;
                        bmp_count     <= 3'd0;
                        wav_valid     <= 1'b0;
            vid_valid     <= 1'b0;
                        vid_valid     <= 1'b0;
                        entry_byte    <= 10'd0;
                        timeout_cnt   <= 32'd0;
                        boot_base     <= 32'd0;
                        retry_boot    <= 1'b0;
                        mbr_lba       <= 32'd0;
                        mbr_candidate <= 1'b0;
                        mbr_found     <= 1'b0;
                        sd_req        <= 1'b1;
                        sd_addr       <= boot_base;
                        state         <= ST_BOOT;
                    end
                end

                ST_BOOT: begin
                    sd_req  <= 1'b1;
                    sd_addr <= boot_base;

                    if (sd_data_valid) begin
                        case (entry_byte)
                            10'd11: bytes_per_sector[7:0]  <= sd_data;
                            10'd12: bytes_per_sector[15:8] <= sd_data;
                            10'd13: sectors_per_cluster <= sd_data;
                            10'd14: reserved_sectors[7:0]  <= sd_data;
                            10'd15: reserved_sectors[15:8] <= sd_data;
                            10'd16: num_fats <= sd_data;
                            10'd32: fat_size[7:0]   <= sd_data;
                            10'd33: fat_size[15:8]  <= sd_data;
                            10'd34: fat_size[23:16] <= sd_data;
                            10'd35: fat_size[31:24] <= sd_data;
                            10'd44: root_cluster[7:0]   <= sd_data;
                            10'd45: root_cluster[15:8]  <= sd_data;
                            10'd46: root_cluster[23:16] <= sd_data;
                            10'd47: root_cluster[31:24] <= sd_data;
                            10'd510: sig_low <= sd_data;
                            10'd511: sig_ok <= (sd_data == 8'hAA) && (sig_low == 8'h55);
                            default: ;
                        endcase

                        // MBR 分区表：在 4 个 16 字节表项里找 FAT32(0x0B/0x0C)
                        if (!mbr_found &&
                            ((entry_byte == 10'd450) || (entry_byte == 10'd466) ||
                             (entry_byte == 10'd482) || (entry_byte == 10'd498))) begin
                            mbr_candidate <= (sd_data == 8'h0B) || (sd_data == 8'h0C);
                        end
                        if (!mbr_found && mbr_candidate) begin
                            case (entry_byte)
                                10'd454: mbr_lba[7:0]   <= sd_data;
                                10'd455: mbr_lba[15:8]  <= sd_data;
                                10'd456: mbr_lba[23:16] <= sd_data;
                                10'd457: begin
                                    mbr_lba[31:24] <= sd_data;
                                    mbr_found      <= 1'b1;
                                    mbr_candidate  <= 1'b0;
                                end
                                10'd470: mbr_lba[7:0]   <= sd_data;
                                10'd471: mbr_lba[15:8]  <= sd_data;
                                10'd472: mbr_lba[23:16] <= sd_data;
                                10'd473: begin
                                    mbr_lba[31:24] <= sd_data;
                                    mbr_found      <= 1'b1;
                                    mbr_candidate  <= 1'b0;
                                end
                                10'd486: mbr_lba[7:0]   <= sd_data;
                                10'd487: mbr_lba[15:8]  <= sd_data;
                                10'd488: mbr_lba[23:16] <= sd_data;
                                10'd489: begin
                                    mbr_lba[31:24] <= sd_data;
                                    mbr_found      <= 1'b1;
                                    mbr_candidate  <= 1'b0;
                                end
                                10'd502: mbr_lba[7:0]   <= sd_data;
                                10'd503: mbr_lba[15:8]  <= sd_data;
                                10'd504: mbr_lba[23:16] <= sd_data;
                                10'd505: begin
                                    mbr_lba[31:24] <= sd_data;
                                    mbr_found      <= 1'b1;
                                    mbr_candidate  <= 1'b0;
                                end
                                default: ;
                            endcase
                        end
                        entry_byte <= entry_byte + 10'd1;
                    end

                    if (sd_end) begin
                        sd_req <= 1'b0;
                        if (bpb_valid) begin
                            state <= ST_BOOT_CHECK;
                        end else if (!retry_boot && mbr_found && (mbr_lba != 32'd0)) begin
                            // LBA0 是 MBR，跳到第一个 FAT32 分区的 VBR 重新解析
                            retry_boot    <= 1'b1;
                            boot_base     <= mbr_lba;
                            entry_byte    <= 10'd0;
                            sig_ok        <= 1'b0;
                            mbr_found     <= 1'b0;
                            mbr_candidate <= 1'b0;
                            sd_req        <= 1'b1;
                            sd_addr       <= mbr_lba;
                            state         <= ST_BOOT;
                        end else begin
                            state <= ST_FAIL;
                        end
                    end
                end

                ST_BOOT_CHECK: begin
                    if (bpb_valid) begin
                        fat_start  <= boot_base + reserved_sectors;
                        data_start <= boot_base + data_start_calc;
                        spc_shift  <= spc_shift_func(sectors_per_cluster);
                        state      <= ST_ROOT_SETUP;
                    end else begin
                        state <= ST_FAIL;
                    end
                end

                ST_ROOT_SETUP: begin
                    dir_cluster  <= root_cluster;
                    dir_sector   <= data_start + ((root_cluster - 32'd2) << spc_shift);
                    dir_sec_left <= sectors_per_cluster;
                    state        <= ST_ROOT_NEXT;
                end

                ST_ROOT: begin
                    sd_req  <= 1'b1;
                    sd_addr <= dir_sector;

                    if (sd_data_valid) begin
                        case (entry_byte[4:0])
                            5'd0: begin
                                entry_end     <= (sd_data == 8'h00);
                                entry_deleted <= (sd_data == 8'hE5);
                            end
                            5'd8:  ext0 <= sd_data;
                            5'd9:  ext1 <= sd_data;
                            5'd10: ext2 <= sd_data;
                            5'd11: attr <= sd_data;
                            5'd20: cluster_hi[7:0]  <= sd_data;
                            5'd21: cluster_hi[15:8] <= sd_data;
                            5'd26: cluster_lo[7:0]  <= sd_data;
                            5'd27: cluster_lo[15:8] <= sd_data;
                            5'd28: file_size[7:0]   <= sd_data;
                            5'd29: file_size[15:8]  <= sd_data;
                            5'd30: file_size[23:16] <= sd_data;
                            5'd31: begin
                                file_size[31:24] <= sd_data;

                                if (!entry_end && !entry_deleted &&
                                    ((attr & 8'h18) == 8'h00) &&
                                    (attr != 8'h0F) &&
                                    (entry_cluster_now >= 32'd2) &&
                                    (entry_cluster_now < FAT_BAD) &&
                                    (entry_size_now > 32'd54)) begin
                                    if ((is_bmp_ext && (bmp_count < MAX_BMP_COUNT)) ||
                                        (is_wav_ext && !wav_valid) ||
                                        (is_vid_ext && !vid_valid)) begin
                                        entry_pending        <= 1'b1;
                                        entry_is_bmp_pending <= is_bmp_ext;
                                        entry_is_wav_pending <= is_wav_ext;
                                        entry_is_vid_pending <= is_vid_ext;
                                        entry_offset_pending <=
                                            (entry_cluster_now - 32'd2) << spc_shift;
                                    end
                                end
                            end
                            default: ;
                        endcase

                        entry_byte <= (entry_byte[4:0] == 5'd31) ? 10'd0 : (entry_byte + 10'd1);
                    end

                    // 目录项簇偏移已寄存，本拍只做 data_start + offset
                    if (entry_pending) begin
                        if (entry_is_bmp_pending) begin
                            case (bmp_count)
                                3'd0: bmp_sector0 <= entry_sector_calc;
                                3'd1: bmp_sector1 <= entry_sector_calc;
                                3'd2: bmp_sector2 <= entry_sector_calc;
                                3'd3: bmp_sector3 <= entry_sector_calc;
                                default: ;
                            endcase
                            bmp_count <= bmp_count + 3'd1;
                        end else if (entry_is_wav_pending && !wav_valid) begin
                            wav_sector <= entry_sector_calc;
                            wav_valid  <= 1'b1;
                        end else if (entry_is_vid_pending && !vid_valid) begin
                            vid_sector <= entry_sector_calc;
                            vid_valid  <= 1'b1;
                        end
                        entry_pending <= 1'b0;
                    end

                    if (sd_end) begin
                        sd_req <= 1'b0;
                        if (entry_end) begin
                            state <= ST_FINISH;
                        end else if (dir_sec_left > 8'd1) begin
                            dir_sec_left <= dir_sec_left - 8'd1;
                            dir_sector   <= dir_sector + 32'd1;
                            state        <= ST_ROOT_NEXT;
                        end else begin
                            fat_sector   <= fat_start + (dir_cluster >> 7);
                            fat_byte_idx <= {dir_cluster[6:0], 2'b00};
                            fat_byte_cnt <= 10'd0;
                            state        <= ST_FAT_NEXT;
                        end
                    end
                end

                ST_ROOT_NEXT: begin
                    sd_req     <= 1'b1;
                    sd_addr    <= dir_sector;
                    entry_byte <= 10'd0;
                    state      <= ST_ROOT;
                end

                ST_FAT_NEXT: begin
                    sd_req      <= 1'b1;
                    sd_addr     <= fat_sector;
                    fat_byte_cnt <= 10'd0;
                    state       <= ST_FAT;
                end

                ST_FAT: begin
                    sd_req  <= 1'b1;
                    sd_addr <= fat_sector;

                    if (sd_data_valid) begin
                        if ((fat_byte_cnt >= {1'b0, fat_byte_idx}) &&
                            (fat_byte_cnt < ({1'b0, fat_byte_idx} + 10'd4))) begin
                            case (fat_byte_cnt - {1'b0, fat_byte_idx})
                                10'd0: fat_b0 <= sd_data;
                                10'd1: fat_b1 <= sd_data;
                                10'd2: fat_b2 <= sd_data;
                                10'd3: fat_b3 <= sd_data;
                                default: ;
                            endcase
                        end
                        fat_byte_cnt <= fat_byte_cnt + 10'd1;
                    end

                    if (sd_end) begin
                        sd_req <= 1'b0;
                        state  <= ST_FAT_CHECK;
                    end
                end

                ST_FAT_CHECK: begin
                    if (fat_value_w >= FAT_EOC_MIN) begin
                        state <= ST_FINISH;
                    end else if ((fat_value_w < 32'd2) || (fat_value_w >= FAT_BAD)) begin
                        state <= ST_FAIL;
                    end else begin
                        dir_cluster  <= fat_value_w;
                        next_cluster <= fat_value_w;
                        state        <= ST_FAT_OFFSET;
                    end
                end

                ST_FAT_OFFSET: begin
                    dir_offset <= (next_cluster - 32'd2) << spc_shift;
                    state      <= ST_FAT_SECTOR;
                end

                ST_FAT_SECTOR: begin
                    dir_sector   <= data_start + dir_offset;
                    dir_sec_left <= sectors_per_cluster;
                    state        <= ST_ROOT_NEXT;
                end

                ST_FINISH: begin
                    sd_req <= 1'b0;
                    busy   <= 1'b0;
                    done   <= 1'b1;
                    error  <= (bmp_count == 3'd0) && !wav_valid;
                    state  <= ST_IDLE;
                end

                ST_FAIL: begin
                    sd_req <= 1'b0;
                    busy   <= 1'b0;
                    done   <= 1'b1;
                    error  <= 1'b1;
                    state  <= ST_IDLE;
                end

                default: begin
                    state  <= ST_FAIL;
                    sd_req <= 1'b0;
                end
            endcase

            if ((state != ST_IDLE) && (timeout_cnt >= TIMEOUT_MAX)) begin
                state  <= ST_FAIL;
                sd_req <= 1'b0;
            end
        end
    end

endmodule
