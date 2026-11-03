// Stereo PCM clock master and completed-frame presentation queue.
// Both queues are flushed together, even if either source clock is stopped.
// RAM contents/read-data are deliberately NOT reset: infer dual-clock BRAM.
module av_pcm_player #(
    parameter integer VID_CLK_HZ = 25000000,
    parameter integer SAMPLE_RATE = 48000,
    parameter integer PCM_ADDR_BITS = 12,
    // Zero selects one complete audio frame (stable samples_per_frame).
    parameter integer START_SAMPLES = 0
)(
    input wire sd_clk, sd_rst, vid_clk, vid_rst, enable,
    input wire [15:0] samples_per_frame,
    input wire pcm_valid,
    input wire [31:0] pcm_data,
    output wire pcm_ready, pcm_sector_ready,
    input wire frame_valid,
    input wire [1:0] frame_index,
    input wire [31:0] frame_sequence,
    output wire frame_ready,
    input wire frame_start,
    output reg display_valid,
    output reg [1:0] disp_buf_idx,
    output reg aud_valid,
    output reg [23:0] aud_left, aud_right,
    output reg underrun, overflow
);
    localparam integer PTR_BITS = PCM_ADDR_BITS + 1;
    localparam integer PCM_DEPTH = 2**PCM_ADDR_BITS;
    wire session_reset = sd_rst | vid_rst | !enable;
    (* async_reg = "true" *) reg [1:0] sd_release, vid_release;
    always @(posedge sd_clk or posedge session_reset)
        if (session_reset) sd_release <= 2'b11;
        else sd_release <= {sd_release[0], 1'b0};
    always @(posedge vid_clk or posedge session_reset)
        if (session_reset) vid_release <= 2'b11;
        else vid_release <= {vid_release[0], 1'b0};
    wire sd_clear = sd_release[1];
    wire vid_clear = vid_release[1];

    function [PTR_BITS-1:0] pcm_ungray;
        input [PTR_BITS-1:0] g;
        integer k;
        begin
            pcm_ungray[PTR_BITS-1] = g[PTR_BITS-1];
            for (k=PTR_BITS-2; k>=0; k=k-1)
                pcm_ungray[k] = pcm_ungray[k+1] ^ g[k];
        end
    endfunction

    reg [31:0] pcm_ram [0:PCM_DEPTH-1];
    reg [PTR_BITS-1:0] pcm_wr_bin, pcm_wr_gray;
    reg [PTR_BITS-1:0] pcm_rd_bin, pcm_rd_gray;
    (* async_reg = "true" *) reg [PTR_BITS-1:0] rd_gray_sd1, rd_gray_sd2;
    (* async_reg = "true" *) reg [PTR_BITS-1:0] wr_gray_vid1, wr_gray_vid2;
    // Register decoded read progress before the sector occupancy subtractor.
    // This pointer is one SD edge older than rd_gray_sd2, so reservations can
    // only become more conservative; newly written words are counted now.
    reg [PTR_BITS-1:0] rd_bin_sd;
    wire [PTR_BITS-1:0] pcm_used_sd = pcm_wr_bin - rd_bin_sd;
    wire [PTR_BITS-1:0] pcm_used_vid = pcm_ungray(wr_gray_vid2) - pcm_rd_bin;
    wire [PTR_BITS-1:0] pcm_free_sd = PCM_DEPTH - pcm_used_sd;
    wire pcm_empty = (pcm_rd_gray == wr_gray_vid2);
    // Standard async-FIFO registered full flag. Inverting the top TWO Gray
    // bits represents the same RAM address one complete FIFO lap ahead.
    // Neither Gray decoding nor occupancy arithmetic feeds the RAM write EN.
    localparam [PTR_BITS-1:0] PCM_FULL_MASK =
        (1 << PCM_ADDR_BITS) | (1 << (PCM_ADDR_BITS-1));
    reg pcm_full;
    assign pcm_ready = !sd_clear && !pcm_full;
    assign pcm_sector_ready = !sd_clear && (PCM_DEPTH >= 128) &&
                              (pcm_used_sd <= PCM_DEPTH - 128);
    wire pcm_push = pcm_valid && pcm_ready;
    wire [PTR_BITS-1:0] pcm_wr_next = pcm_wr_bin + 1'b1;
    wire [PTR_BITS-1:0] pcm_wr_next_gray = (pcm_wr_next >> 1) ^ pcm_wr_next;
    wire [PTR_BITS-1:0] pcm_wr_gray_after_push = pcm_push ? pcm_wr_next_gray : pcm_wr_gray;
    wire [PTR_BITS-1:0] pcm_rd_next = pcm_rd_bin + 1'b1;
    always @(posedge sd_clk)
        if (pcm_push) pcm_ram[pcm_wr_bin[PCM_ADDR_BITS-1:0]] <= pcm_data;

    always @(posedge sd_clk or posedge sd_clear) begin
        if (sd_clear) begin
            pcm_wr_bin <= 0; pcm_wr_gray <= 0;
            rd_gray_sd1 <= 0; rd_gray_sd2 <= 0;
            rd_bin_sd <= 0; pcm_full <= 0;
            overflow <= 0;
        end else begin
            rd_gray_sd1 <= pcm_rd_gray;
            rd_gray_sd2 <= rd_gray_sd1;
            rd_bin_sd <= pcm_ungray(rd_gray_sd2);
            pcm_full <= (pcm_wr_gray_after_push == (rd_gray_sd2 ^ PCM_FULL_MASK));
            if (pcm_push) begin
                pcm_wr_bin <= pcm_wr_next;
                pcm_wr_gray <= pcm_wr_next_gray;
            end
            if (pcm_valid && !pcm_ready) overflow <= 1;
        end
    end

    // Each synchronous RAM read carries its FULL address (including wrap).
    // A pop invalidates the old q until a read of the new head has returned.
    reg [31:0] pcm_q;
    reg [PTR_BITS-1:0] pcm_q_addr;
    reg pcm_q_valid;
    always @(posedge vid_clk)
        pcm_q <= pcm_ram[pcm_rd_bin[PCM_ADDR_BITS-1:0]];
    wire pcm_head_ready = !pcm_empty && pcm_q_valid && (pcm_q_addr == pcm_rd_bin);

    // Two-entry asynchronous completed-frame FIFO: {sequence, buffer index}.
    reg [33:0] meta_ram [0:1];
    reg [1:0] meta_wr_bin, meta_wr_gray, meta_rd_bin, meta_rd_gray;
    (* async_reg = "true" *) reg [1:0] meta_rd_sd1, meta_rd_sd2;
    (* async_reg = "true" *) reg [1:0] meta_wr_vid1, meta_wr_vid2;
    wire meta_full = (meta_wr_gray == ~meta_rd_sd2);
    wire meta_empty = (meta_rd_gray == meta_wr_vid2);
    assign frame_ready = !sd_clear && !meta_full;
    wire [1:0] meta_wr_next = meta_wr_bin + 2'd1;
    wire [1:0] meta_rd_next = meta_rd_bin + 2'd1;
    always @(posedge sd_clk)
        if (frame_valid && frame_ready)
            meta_ram[meta_wr_bin[0]] <= {frame_sequence, frame_index};
    always @(posedge sd_clk or posedge sd_clear) begin
        if (sd_clear) begin
            meta_wr_bin <= 0; meta_wr_gray <= 0;
            meta_rd_sd1 <= 0; meta_rd_sd2 <= 0;
        end else begin
            meta_rd_sd1 <= meta_rd_gray;
            meta_rd_sd2 <= meta_rd_sd1;
            if (frame_valid && frame_ready) begin
                meta_wr_bin <= meta_wr_next;
                meta_wr_gray <= (meta_wr_next >> 1) ^ meta_wr_next;
            end
        end
    end
    reg [33:0] meta_q;
    reg [1:0] meta_q_addr;
    reg meta_q_valid;
    always @(posedge vid_clk) meta_q <= meta_ram[meta_rd_bin[0]];
    wire meta_head_ready = !meta_empty && meta_q_valid && (meta_q_addr == meta_rd_bin);

    // Cadence belongs to the video clock, NOT to a playback session or VS.
    // aud_valid announces a sample every tick, including silence and underrun.
    reg [31:0] sample_acc;
    wire [32:0] sample_sum = {1'b0, sample_acc} + SAMPLE_RATE;
    wire sample_tick = (sample_sum >= VID_CLK_HZ);
    always @(posedge vid_clk or posedge vid_rst) begin
        if (vid_rst) begin sample_acc <= 0; aud_valid <= 0; end
        else begin
            if (sample_tick) sample_acc <= sample_sum - VID_CLK_HZ;
            else sample_acc <= sample_sum;
            aud_valid <= sample_tick;
        end
    end

    (* async_reg = "true" *) reg [15:0] spf_vid1, spf_vid2;
    reg started;
    reg [15:0] sample_count;
    reg [31:0] audio_sequence;
    wire pcm_pop = started && sample_tick && pcm_head_ready && !vid_clear;
    wire sequence_step = pcm_pop && (spf_vid2 != 0) &&
                         (sample_count == spf_vid2 - 16'd1);
    wire [31:0] sequence_at_edge = audio_sequence + sequence_step;
    // Signed modular comparison; producer is always less than 2^31 ahead.
    wire [31:0] head_sequence_delta = sequence_at_edge - meta_q[33:2];
    wire [31:0] start_threshold = (START_SAMPLES == 0) ? {16'd0, spf_vid2} : START_SAMPLES;
    wire start_now = !started && frame_start && meta_head_ready &&
                     pcm_head_ready && (spf_vid2 != 0) &&
                     // Explicit prefill overrides allow streaming packets
                     // larger than the FIFO; dynamic prefill must fit in full.
                     ((START_SAMPLES != 0) || (spf_vid2 <= PCM_DEPTH)) &&
                     (pcm_used_vid >= start_threshold);
    wire meta_pop = !vid_clear && frame_start && meta_head_ready &&
                    (start_now || (started && !head_sequence_delta[31]));

    always @(posedge vid_clk or posedge vid_clear) begin
        if (vid_clear) begin
            pcm_rd_bin <= 0; pcm_rd_gray <= 0;
            wr_gray_vid1 <= 0; wr_gray_vid2 <= 0;
            pcm_q_addr <= 0; pcm_q_valid <= 0;
            meta_rd_bin <= 0; meta_rd_gray <= 0;
            meta_wr_vid1 <= 0; meta_wr_vid2 <= 0;
            meta_q_addr <= 0; meta_q_valid <= 0;
            spf_vid1 <= 0; spf_vid2 <= 0;
            started <= 0; sample_count <= 0; audio_sequence <= 0;
            display_valid <= 0; disp_buf_idx <= 0;
            aud_left <= 0; aud_right <= 0; underrun <= 0;
        end else begin
            wr_gray_vid1 <= pcm_wr_gray;
            wr_gray_vid2 <= wr_gray_vid1;
            meta_wr_vid1 <= meta_wr_gray;
            meta_wr_vid2 <= meta_wr_vid1;
            pcm_q_addr <= pcm_rd_bin; pcm_q_valid <= !pcm_empty;
            meta_q_addr <= meta_rd_bin; meta_q_valid <= !meta_empty;
            spf_vid1 <= samples_per_frame;
            spf_vid2 <= spf_vid1;
            if (start_now) started <= 1;
            if (meta_pop) begin
                meta_rd_bin <= meta_rd_next;
                meta_rd_gray <= (meta_rd_next >> 1) ^ meta_rd_next;
                display_valid <= 1;
                disp_buf_idx <= meta_q[1:0];
            end
            if (sample_tick) begin
                if (!started) begin aud_left <= 0; aud_right <= 0; end
                else if (pcm_pop) begin
                    aud_left <= {pcm_q[15:0], 8'b0};
                    aud_right <= {pcm_q[31:16], 8'b0};
                    pcm_rd_bin <= pcm_rd_next;
                    pcm_rd_gray <= (pcm_rd_next >> 1) ^ pcm_rd_next;
                    if (sequence_step) begin
                        sample_count <= 0;
                        audio_sequence <= audio_sequence + 32'd1;
                    end else sample_count <= sample_count + 16'd1;
                end else underrun <= 1;
            end
        end
    end
endmodule
