// RIFF PCM reader: exact byte boundaries, complete-sector staging, precise loop.
module wav_audio #(
    parameter integer SD_CLK_HZ    = 50_000_000,
    parameter integer VID_CLK_HZ   = 25_000_000,
    parameter integer SAMPLE_RATE  = 48_000,
    parameter [31:0]  SCAN_START   = 32'd0,
    parameter [31:0]  SCAN_MAX     = 32'd131071,
    // 自检音：1 = 不读 SD 数据，在视频域自己产生 1kHz 三角波 + 约 0.7s 通断
    // 0 = 正常播放 WAV；1 = 输出通路自检音；2 = 缓冲自检（写已知三角波，不读 SD）
    parameter  [1:0]  SELFTEST     = 2'd0,
    // 期望校验和：兼容旧工具：RIFF 起始 LBA+1 起 4 个物理扇区(2048B)求和取低16位
    parameter  [15:0] SUM_EXPECT   = 16'hF5E2
)(
    input  wire        sd_clk,
    input  wire        sd_rst,
    input  wire        vid_clk,
    input  wire        vid_rst,
    input  wire        enable,             // 例如首图显示后拉高

    // 目录解析结果：file_valid=1 时直接从 file_sector 开始找 RIFF，不再全盘扫描
    input  wire        file_valid,
    input  wire [31:0] file_sector,

    // ---- 扇区读接口（→ sd_card_bmp 的 WAV 通道）----
    output reg         sec_read,
    output reg  [31:0] sec_read_addr,
    input  wire [7:0]  sec_read_data,
    input  wire        sec_read_data_valid,
    input  wire        sec_read_end,

    // ---- 音频输出（vid_clk 域）----
    output reg         aud_valid,          // 每个样本 1 拍
    output reg  [23:0] aud_left,
    output reg  [23:0] aud_right,
    output wire        playing,
    output wire        active,
    output reg  [15:0] sum16,          // RIFF 首扇区之后 4 个物理扇区的字节和（兼容旧诊断）
    output wire        sum_ok
);


    // Only complete physical sectors are exposed to the parser. A failed read
    // is retried at the same LBA without changing parser/FIFO/checksum state.
    // Three cycles per byte isolate the BRAM output from parser decoding.
    reg [7:0] sector_mem [0:511];
    reg [7:0] sector_ram_q, sector_q;
    reg [9:0] rx_count;
    reg [8:0] parse_idx, read_offset, data_start_off;
    reg [31:0] addr, base_lba, data_start;
    reg [24:0] to_cnt;
    reg draining;
    wire [10:0] rx_total = {1'b0, rx_count} + (sec_read_data_valid ? 11'd1 : 11'd0);

    reg [31:0] sbuf [0:4095];
    reg [11:0] wrp, rdp, wrp_cf;
    reg [15:0] cf_ph;
    wire [15:0] cf_s16 = cf_ph[15] ? (16'hffff - cf_ph) : cf_ph;
    wire [11:0] wrp_eff = (SELFTEST == 2'd2) ? wrp_cf : wrp;
    wire [11:0] buf_cnt = wrp_eff - rdp;
    wire buf_empty = (wrp_eff == rdp);
    wire sbuf_re;
    wire buf_room = (buf_cnt != 12'd4095); // reserve one entry: full != empty

    localparam S_IDLE=4'd0, S_REQ=4'd1, S_RX=4'd2,
               S_LOAD=4'd3, S_PARSE=4'd4, S_BAD=4'd5,
               S_RIFF_LIMIT=4'd6, S_RIFF_LAST=4'd7, S_RIFF_LBA=4'd8,
               S_RIFF_CHECK=4'd9, S_CHUNK_SUM=4'd10, S_CHUNK_PAD=4'd11,
               S_CHUNK_CHECK=4'd12, S_CHUNK_APPLY=4'd13, S_HEADER_CHECK=4'd14,
               S_FETCH=4'd15;
    localparam P_RIFF=3'd0, P_CHUNK=3'd1, P_FMT=3'd2,
               P_SKIP=3'd3, P_PAD=3'd4, P_PCM=3'd5;
    reg [3:0] st, parse_resume;
    reg [2:0] parser;
    reg started, streaming, fmt_ok;
    reg [3:0] field_idx;
    reg [95:0] riff_hdr;
    reg [127:0] fmt_hdr;
    reg [31:0] chunk_id, chunk_size, chunk_left;
    reg chunk_pad;
    reg [31:0] file_pos, riff_end, riff_last, data_length, data_left;
    reg [1:0] pcm_phase;
    reg [15:0] l16;
    reg [7:0] r_low;
    reg [15:0] sum_acc, sec_sum;
    reg [2:0] sum_sec;

    wire [31:0] incoming_size = {sector_q, chunk_size[23:0]};
    // Each arithmetic step ends at a register; validation stalls the parser.
    // Widen before addition so malformed lengths cannot wrap past RIFF.
    reg [32:0] riff_limit_q, riff_last_q, last_lba_q;
    reg [32:0] header_end_q, payload_end_q, padded_end_q;
    reg riff_header_ok, chunk_fits;
    reg metadata_at_end, data_chunk_ok, fmt_chunk_ok, chunk_empty;
    reg [1:0] chunk_kind;
    reg [32:0] checksum_lba_q;
    reg checksum_sector;
    assign playing = streaming;
    assign active = (st != S_IDLE) && (st != S_BAD);
    assign sum_ok = (sum16 == SUM_EXPECT);

    always @(posedge sd_clk) begin
        if (st == S_RX && sec_read_data_valid && rx_count < 10'd512)
            sector_mem[rx_count[8:0]] <= sec_read_data;
        sector_ram_q <= sector_mem[parse_idx];
        sector_q <= sector_ram_q;
    end

    always @(posedge sd_clk or posedge sd_rst) begin
        if (sd_rst) begin
            st <= S_IDLE; parse_resume <= S_LOAD; parser <= P_RIFF;
            started <= 0; streaming <= 0; fmt_ok <= 0;
            sec_read <= 0; sec_read_addr <= 0;
            addr <= SCAN_START; base_lba <= SCAN_START;
            rx_count <= 0; parse_idx <= 0; read_offset <= 0;
            data_start <= 0; data_start_off <= 0;
            to_cnt <= 0; draining <= 0;
            field_idx <= 0; riff_hdr <= 0; fmt_hdr <= 0;
            chunk_id <= 0; chunk_size <= 0; chunk_left <= 0; chunk_pad <= 0;
            file_pos <= 0; riff_end <= 0; riff_last <= 0; data_length <= 0; data_left <= 0;
            riff_limit_q <= 0; riff_last_q <= 0; last_lba_q <= 0;
            header_end_q <= 0; payload_end_q <= 0; padded_end_q <= 0;
            riff_header_ok <= 0; chunk_fits <= 0;
            metadata_at_end <= 0; data_chunk_ok <= 0; fmt_chunk_ok <= 0;
            chunk_empty <= 0; chunk_kind <= 0;
            checksum_lba_q <= 0; checksum_sector <= 0;
            pcm_phase <= 0; l16 <= 0; r_low <= 0; wrp <= 0;
            sum_acc <= 0; sec_sum <= 0; sum_sec <= 0; sum16 <= 0;
        end else begin
            case (st)
                S_IDLE: begin
                    sec_read <= 0;
                    if (enable && SELFTEST == 2'd0) begin
                        if (!started) begin
                            addr <= file_valid ? file_sector : SCAN_START;
                            base_lba <= file_valid ? file_sector : SCAN_START;
                            started <= 1;
                        end
                        st <= S_REQ;
                    end
                end
                S_REQ: begin
                    checksum_lba_q <= {1'b0,base_lba} + 33'd1 + sum_sec;
                    if (!enable) st <= S_IDLE;
                    else begin
                        sec_read <= 1; sec_read_addr <= addr;
                        rx_count <= 0; sec_sum <= 0; to_cnt <= 0; draining <= 0;
                        st <= S_RX;
                    end
                end
                S_RX: begin
                    // LBA is stable for a whole sector. Finish its comparison
                    // well before the last byte/checksum update arrives.
                    checksum_sector <= {1'b0,addr} == checksum_lba_q;
                    if (sec_read_data_valid) begin
                        if (rx_count != 10'd1023) rx_count <= rx_count + 10'd1;
                        sec_sum <= sec_sum + {8'd0,sec_read_data};
                        to_cnt <= 0;
                    end else if (to_cnt == (draining ? 25'd4000000 : 25'd2000000)) begin
                        to_cnt <= 0;
                        if (draining) begin
                            sec_read <= 0; draining <= 0; st <= S_REQ;
                        end else draining <= 1;
                    end else to_cnt <= to_cnt + 25'd1;
                    if (sec_read_end) begin
                        sec_read <= 0;
                        if (!draining && rx_total == 11'd512) begin
                            parse_idx <= read_offset;
                            st <= S_FETCH;
                            // Keep the existing SUM_EXPECT convention. Retries
                            // and loops never count the same sector twice.
                            if (sum_sec < 3'd4 && checksum_sector) begin
                                sum_acc <= sum_acc + sec_sum +
                                    (sec_read_data_valid ? {8'd0,sec_read_data} : 16'd0);
                                sum_sec <= sum_sec + 3'd1;
                                if (sum_sec == 3'd3)
                                    sum16 <= sum_acc + sec_sum +
                                        (sec_read_data_valid ? {8'd0,sec_read_data} : 16'd0);
                            end
                        end else st <= S_REQ;
                    end
                end
                S_FETCH: st <= S_LOAD;
                S_LOAD: begin
                    metadata_at_end <= file_pos >= riff_last;
                    if (parser != P_PCM || pcm_phase != 2'd3 || buf_room)
                        st <= S_PARSE;
                end
                S_PARSE: begin
                    file_pos <= file_pos + 32'd1;
                    if (parse_idx == 9'd511) begin
                        addr <= addr + 32'd1;
                        read_offset <= 0;
                        parse_resume <= S_REQ;
                        st <= S_REQ;
                    end else begin
                        parse_idx <= parse_idx + 9'd1;
                        parse_resume <= S_FETCH;
                        st <= S_FETCH;
                    end
                    case (parser)
                        P_RIFF: begin
                            riff_hdr[field_idx*8 +: 8] <= sector_q;
                            if (field_idx == 4'd11) begin
                                field_idx <= 0;
                                st <= S_RIFF_LIMIT;
                            end else field_idx <= field_idx + 4'd1;
                        end
                        P_CHUNK: begin
                            if (field_idx < 4) chunk_id[field_idx*8 +: 8] <= sector_q;
                            else chunk_size[(field_idx-4)*8 +: 8] <= sector_q;
                            if (field_idx == 0) begin
                                header_end_q <= {1'b0,file_pos} + 33'd8;
                                field_idx <= 1;
                                st <= S_HEADER_CHECK;
                            end else if (field_idx == 7) begin
                                field_idx <= 0;
                                chunk_left <= incoming_size;
                                chunk_pad <= incoming_size[0];
                                st <= S_CHUNK_SUM;
                            end else field_idx <= field_idx + 4'd1;
                        end
                        P_FMT: begin
                            if (field_idx < 16) fmt_hdr[field_idx*8 +: 8] <= sector_q;
                            // The first 16 bytes form the PCM format. Extension
                            // bytes are opaque; tag 1 is required (not extensible).
                            if (field_idx == 15) begin
                                // Last format byte is arriving on this edge.
                                if (fmt_hdr[15:0] != 1 || fmt_hdr[31:16] != 2 ||
                                    fmt_hdr[63:32] != SAMPLE_RATE ||
                                    fmt_hdr[95:64] != SAMPLE_RATE*4 ||
                                    fmt_hdr[111:96] != 4 ||
                                    {sector_q,fmt_hdr[119:112]} != 16)
                                    st <= S_BAD;
                                else begin
                                    fmt_ok <= 1;
                                    parser <= chunk_left == 1 ? (chunk_pad ? P_PAD : P_CHUNK) : P_SKIP;
                                    field_idx <= 0;
                                end
                            end else field_idx <= field_idx + 4'd1;
                            chunk_left <= chunk_left - 32'd1;
                        end
                        P_SKIP: begin
                            chunk_left <= chunk_left - 32'd1;
                            if (chunk_left == 1) begin
                                parser <= chunk_pad ? P_PAD : P_CHUNK;
                                field_idx <= 0;
                            end
                        end
                        P_PAD: begin parser <= P_CHUNK; field_idx <= 0; end
                        P_PCM: begin
                            case (pcm_phase)
                                0: l16[7:0] <= sector_q;
                                1: l16[15:8] <= sector_q;
                                2: r_low <= sector_q;
                                3: wrp <= wrp + 12'd1;
                            endcase
                            pcm_phase <= pcm_phase + 2'd1;
                            data_left <= data_left - 32'd1;
                            if (data_left == 1) begin
                                // Ignore trailing metadata/sector slack. Re-read
                                // exactly the first PCM LBA and byte offset.
                                addr <= data_start; read_offset <= data_start_off;
                                data_left <= data_length; pcm_phase <= 0;
                                st <= S_REQ;
                            end
                        end
                        default: st <= S_BAD;
                    endcase
                    // RIFF without data stops even if metadata ends at byte 511.
                    if (parser != P_RIFF && parser != P_PCM &&
                        metadata_at_end) st <= S_BAD;
                end
                // No byte is consumed in these states. parse_resume records
                // whether the last header byte also exhausted its sector.
                S_RIFF_LIMIT: begin
                    riff_limit_q <= {1'b0,riff_hdr[63:32]} + 33'd8;
                    riff_header_ok <= riff_hdr[31:0] == 32'h46464952 &&
                                      riff_hdr[95:64] == 32'h45564157 &&
                                      riff_hdr[63:32] >= 32'd4;
                    st <= S_RIFF_LAST;
                end
                S_RIFF_LAST: begin
                    riff_last_q <= riff_limit_q - 33'd1;
                    st <= S_RIFF_LBA;
                end
                S_RIFF_LBA: begin
                    last_lba_q <= {1'b0,base_lba} + {9'd0,riff_last_q[32:9]};
                    st <= S_RIFF_CHECK;
                end
                S_RIFF_CHECK: begin
                    if (!riff_header_ok || riff_limit_q[32] || last_lba_q[32])
                        st <= S_BAD;
                    else begin
                        riff_end <= riff_limit_q[31:0];
                        riff_last <= riff_last_q[31:0];
                        parser <= P_CHUNK;
                        st <= parse_resume;
                    end
                end
                S_HEADER_CHECK: begin
                    st <= header_end_q > {1'b0,riff_end} ? S_BAD : parse_resume;
                end
                S_CHUNK_SUM: begin
                    // file_pos has already advanced to payload byte zero and
                    // chunk_size contains all four bytes, even at byte 511.
                    payload_end_q <= {1'b0,file_pos} + {1'b0,chunk_size};
                    st <= S_CHUNK_PAD;
                end
                S_CHUNK_PAD: begin
                    padded_end_q <= payload_end_q + {32'd0,chunk_pad};
                    st <= S_CHUNK_CHECK;
                end
                S_CHUNK_CHECK: begin
                    chunk_fits <= padded_end_q <= {1'b0,riff_end};
                    fmt_chunk_ok <= !fmt_ok && chunk_size >= 32'd16;
                    data_chunk_ok <= fmt_ok && chunk_size != 0 && chunk_size[1:0] == 0;
                    chunk_empty <= chunk_size == 0;
                    chunk_kind <= chunk_id == 32'h20746d66 ? 2'd1 :
                                  chunk_id == 32'h61746164 ? 2'd2 : 2'd0;
                    st <= S_CHUNK_APPLY;
                end
                S_CHUNK_APPLY: begin
                    st <= parse_resume;
                    if (!chunk_fits) st <= S_BAD;
                    else if (chunk_kind == 2'd1) begin // fmt
                        if (!fmt_chunk_ok) st <= S_BAD;
                        else begin parser <= P_FMT; fmt_hdr <= 0; end
                    end else if (chunk_kind == 2'd2) begin // data
                        if (!data_chunk_ok)
                            st <= S_BAD;
                        else begin
                            parser <= P_PCM; streaming <= 1;
                            data_length <= chunk_size; data_left <= chunk_size;
                            // Header consumption already advanced addr/index;
                            // a sector-boundary payload starts at offset zero.
                            data_start <= addr;
                            data_start_off <= parse_resume == S_REQ ? 9'd0 : parse_idx;
                            pcm_phase <= 0;
                        end
                    end else if (chunk_empty) parser <= P_CHUNK;
                    else parser <= P_SKIP;
                end
                S_BAD: begin
                    sec_read <= 0;
                    if ((!file_valid && base_lba < SCAN_MAX) || !enable) begin
                        if (enable) begin
                            base_lba <= base_lba + 32'd1; addr <= base_lba + 32'd1;
                            st <= S_REQ;
                        end else begin started <= 0; st <= S_IDLE; end
                        parser <= P_RIFF; file_pos <= 0; field_idx <= 0;
                        read_offset <= 0; fmt_ok <= 0; streaming <= 0;
                        sum_acc <= 0; sum_sec <= 0; sum16 <= 0;
                    end
                end
                default: st <= S_IDLE;
            endcase
        end
    end
    // ==================== 48kHz 节拍 + 握手发送（sd 域）====================
    reg  [31:0] acc;
    wire tick = ((acc + SAMPLE_RATE) >= SD_CLK_HZ);

    always @(posedge sd_clk or posedge sd_rst) begin
        if (sd_rst) acc <= 32'd0;
        else if (tick) acc <= acc + SAMPLE_RATE - SD_CLK_HZ;
        else           acc <= acc + SAMPLE_RATE;
    end

    // ==================== 样本搬运（sd 域 → 持稳寄存器）====================
    // 【2026-10-26 重写】原来的做法是“sd 域 48kHz 节拍 + 四相握手”：
    //   节拍产生在 sd 域；若上一拍样本还没被视频域取走，这一拍就被整拍丢掉；
    //   缓冲空时还补静音。结果送给 HDMI 的音频流 cadence 不稳、偶尔断供，
    //   接收端失锁 → 持续刺耳的沙沙声（测试音那条链在音频域精确 48kHz，所以干净）。
    // 现在：sd 域只负责“把样本搬进持稳寄存器，并用 toggle 通知”；
    //       输出节拍改在 **视频域** 用累加器精确产生 48kHz，一个节拍都不丢；
    //       缓冲空时保持上一个样本（不补静音，避免爆音）。
    reg        s_toggle;             // 每送出一个样本翻转一次（跨域通知用）
    reg [23:0] s_l, s_r;

    always @(posedge sd_clk or posedge sd_rst) begin
        if (sd_rst) begin
            s_toggle <= 1'b0;
            rdp      <= 12'd0;                         // rdp 唯一驱动点
        end else if (tick) begin
            if (sbuf_re) begin
                rdp <= rdp + 12'd1;
            end
            // 缓冲空（例如正在给图片加载让路）：保持上一个样本，
            // 节拍照走 —— 给 HDMI 的音频流一旦断供，接收端失锁就是持续噪音。
            s_toggle <= ~s_toggle;
        end
    end

    // ==================== 48kHz 输出节拍（视频域，精确）====================
    reg  [31:0] vacc;
    wire vtick = ((vacc + SAMPLE_RATE) >= VID_CLK_HZ);

    always @(posedge vid_clk or posedge vid_rst) begin
        if (vid_rst) vacc <= 32'd0;
        else if (vtick) vacc <= vacc + SAMPLE_RATE - VID_CLK_HZ;
        else            vacc <= vacc + SAMPLE_RATE;
    end

    // ==================== 自检音（SELFTEST=1 时生效）====================
    // 用来区分“取数据坏了”还是“输出通路坏了”：
    //   自检音 = 间断的干净嘟嘟声 → 输出通路 OK，噪音来自 sbuf 里的数据；
    //   自检音也沙沙响            → 输出通路（vtick/跨域/aud_valid）本身有问题。
    reg  [15:0] st_ph;
    reg  [14:0] st_env;
    wire [15:0] st_tri = st_ph[15] ? (16'hFFFF - st_ph) : st_ph;
    wire        st_amp = ~st_env[14];

    always @(posedge vid_clk or posedge vid_rst) begin
        if (vid_rst) begin
            st_ph  <= 16'd0;
            st_env <= 15'd0;
        end else if (vtick) begin
            st_ph  <= st_ph + 16'd1365;     // 1365/65536 × 48kHz ≈ 1kHz
            st_env <= st_env + 15'd1;
        end
    end

    // ==================== 音频输出（vid 域）====================
    reg [2:0]  tog_sync;
    reg [23:0] h_l, h_r;             // 持稳样本（上一个有效样本）

    always @(posedge vid_clk or posedge vid_rst) begin
        if (vid_rst) begin
            tog_sync  <= 3'd0;
            h_l       <= 24'd0;
            h_r       <= 24'd0;
            aud_valid <= 1'b0;
            aud_left  <= 24'd0;
            aud_right <= 24'd0;
        end else begin
            tog_sync <= {tog_sync[1:0], s_toggle};
            // 只消费同步链后两拍；第一级 tog_sync[0] 不参与数据采用。
            // s_l/s_r 持稳一个采样周期，采用时已经稳定至少两个视频时钟。
            if (tog_sync[2] ^ tog_sync[1]) begin
                h_l <= s_l;
                h_r <= s_r;
            end
            // 精确 48kHz 输出，一个节拍都不少
            if (vtick) begin
                aud_left  <= (SELFTEST == 2'd1) ? (st_amp ? {st_tri, 8'b0} : 24'd0) : h_l;
                aud_right <= (SELFTEST == 2'd1) ? (st_amp ? {st_tri, 8'b0} : 24'd0) : h_r;
                aud_valid <= 1'b1;
            end else begin
                aud_valid <= 1'b0;
            end
        end
    end


    wire sbuf_we = st == S_PARSE && parser == P_PCM && pcm_phase == 3;
    reg [31:0] sample_q;
    reg [11:0] sample_addr;
    reg sample_ready;
    assign sbuf_re = tick && !buf_empty && sample_ready && sample_addr == rdp;
    // Unconditional registered read keeps the RAM inference pattern. Publish
    // only a valid prefetched word, so reset/underflow never exposes RAM Xs.
    always @(posedge sd_clk) begin
        if (SELFTEST == 2'd2) begin
            if (!sd_rst && tick && buf_room) sbuf[wrp_cf] <= {cf_s16,cf_s16};
        end else if (sbuf_we) sbuf[wrp] <= {sector_q,r_low,l16};
        sample_q <= sbuf[rdp];
    end
    always @(posedge sd_clk or posedge sd_rst) begin
        if (sd_rst) begin
            sample_ready <= 0; sample_addr <= 0;
            s_l <= 0; s_r <= 0; wrp_cf <= 0; cf_ph <= 0;
        end else begin
            sample_addr <= rdp; sample_ready <= !buf_empty;
            if (sbuf_re) begin
                s_l <= {sample_q[15:0],8'd0};
                s_r <= {sample_q[31:16],8'd0};
            end
            if (SELFTEST == 2'd2 && tick && buf_room) begin
                wrp_cf <= wrp_cf + 12'd1;
                cf_ph <= cf_ph + 16'd1365;
            end
        end
    end
endmodule
