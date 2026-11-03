`timescale 1ns/1ps

// Four real samples/frame configurations run concurrently. Clocks are
// accelerated in wall time; the 25,000,000/48,000 cadence ratio is unchanged.
module av_pcm_case #(
    parameter integer SPF = 1920,
    parameter integer CASE_ID = 0,
    parameter integer ADDR_BITS = 11
)(output reg done = 0, output reg passed = 0);
    localparam integer DEPTH = 2**ADDR_BITS;
    localparam integer START = (ADDR_BITS == 11) ? 64 : 4;
    reg sd_clk = 0, vid_clk = 0;
    always #3 sd_clk = ~sd_clk;
    always #5 vid_clk = ~vid_clk;
    reg sd_rst = 1, vid_rst = 1, enable = 0;
    reg [15:0] samples_per_frame = SPF;
    reg pcm_valid = 0, frame_valid = 0, frame_start = 0;
    reg [31:0] pcm_data = 0, frame_sequence = 0;
    reg [1:0] frame_index = 0;
    wire pcm_ready, pcm_sector_ready, frame_ready, display_valid;
    wire [1:0] disp_buf_idx;
    wire aud_valid, underrun, overflow;
    wire [23:0] aud_left, aud_right;
    av_pcm_player #(.PCM_ADDR_BITS(ADDR_BITS), .START_SAMPLES(START)) dut (
        .sd_clk(sd_clk), .sd_rst(sd_rst), .vid_clk(vid_clk), .vid_rst(vid_rst),
        .enable(enable), .samples_per_frame(samples_per_frame),
        .pcm_valid(pcm_valid), .pcm_data(pcm_data), .pcm_ready(pcm_ready),
        .pcm_sector_ready(pcm_sector_ready), .frame_valid(frame_valid),
        .frame_index(frame_index), .frame_sequence(frame_sequence),
        .frame_ready(frame_ready), .frame_start(frame_start),
        .display_valid(display_valid), .disp_buf_idx(disp_buf_idx),
        .aud_valid(aud_valid), .aud_left(aud_left), .aud_right(aud_right),
        .underrun(underrun), .overflow(overflow)
    );

    task fail;
        input [511:0] reason;
        begin
            $display("FAIL case=%0d spf=%0d time=%0t: %0s", CASE_ID, SPF, $time, reason);
            $finish;
        end
    endtask
    function [31:0] sample_word;
        input integer n;
        reg [15:0] l, r;
        begin
            l = (n * 73 + 16'h81a3) ^ (CASE_ID * 251);
            r = (n * 151 + 16'h4e29) ^ (CASE_ID * 997);
            sample_word = {r,l};
        end
    endfunction
    reg [31:0] expected_pcm [0:65535];
    reg [33:0] expected_meta [0:1023];
    integer written = 0, consumed = 0, meta_written = 0, presented = 0;
    integer epoch = 0, next_word = 0;
    integer cadence_acc = 0, ticks = 0, underflow_ticks = 0;
    integer prior_ptr, prior_meta, prior_count, prior_seq;
    reg prior_started, expected_tick, boundary;
    reg [23:0] held_left, held_right;
    reg [31:0] word_expected;
    reg [31:0] expected_sequence, delta;
    reg [1:0] prior_display_idx;
    reg prior_display_valid;
    integer seed = 12345;
    integer random_value;
    reg session_saw_full = 0;
    integer full_resume_writes = 0;
    initial begin
        if ($value$plusargs("SEED=%d", seed)) begin end
        seed = seed + CASE_ID * 719;
    end
    always @(posedge sd_clk) begin
        if (dut.sd_clear) session_saw_full = 0;
        if (enable && !dut.sd_clear) begin
            // Independent accepted-stream occupancy, not the DUT's decoder.
            // Delayed read progress may withhold permission, never overgrant.
            if (pcm_ready && written-consumed >= DEPTH)
                fail("write permission while physically full");
            if (pcm_sector_ready && DEPTH-(written-consumed) < 128)
                fail("sector permission exceeds real free capacity");
            if (dut.pcm_used_sd < written-consumed)
                fail("pipelined sector occupancy is not conservative");
            if (!pcm_ready) session_saw_full = 1;
            if (pcm_valid && pcm_ready) begin
                if (session_saw_full) full_resume_writes = full_resume_writes + 1;
                expected_pcm[written] = pcm_data;
                written = written + 1;
            end
            if (frame_valid && frame_ready) begin
                expected_meta[meta_written] = {frame_sequence,frame_index};
                meta_written = meta_written + 1;
            end
            if (dut.pcm_used_sd > DEPTH) fail("write occupancy exceeds physical depth");
            if (pcm_sector_ready && dut.pcm_free_sd < 128)
                fail("sector reservation overcommits FIFO");
        end
    end

    // Check each video edge, including disabled sessions: aud_valid never loses
    // phase; RAM output, ordering, media count and display deadlines agree with
    // the independently recorded accepted producer stream.
    always @(posedge vid_clk) begin
        expected_tick = 0;
        if (vid_rst) cadence_acc = 0;
        else begin
            cadence_acc = cadence_acc + 48000;
            if (cadence_acc >= 25000000) begin
                cadence_acc = cadence_acc - 25000000;
                expected_tick = 1;
                ticks = ticks + 1;
            end
        end
        prior_ptr = dut.pcm_rd_bin;
        prior_meta = dut.meta_rd_bin;
        prior_started = dut.started;
        prior_count = dut.sample_count;
        prior_seq = dut.audio_sequence;
        boundary = frame_start;
        held_left = aud_left; held_right = aud_right;
        prior_display_idx = disp_buf_idx; prior_display_valid = display_valid;
        #1;
        if (aud_valid !== expected_tick) fail("48k cadence changed phase or missed tick");
        if (!enable || dut.vid_clear) begin
            if (display_valid || aud_left !== 0 || aud_right !== 0)
                fail("disabled session leaked audio/display");
        end else begin
            if (dut.pcm_rd_bin != prior_ptr) begin
                if (!expected_tick || !prior_started) fail("consumption outside running sample tick");
                if (consumed >= written) fail("read unwritten sample");
                word_expected = expected_pcm[consumed];
                if (aud_left !== {word_expected[15:0],8'h00} ||
                    aud_right !== {word_expected[31:16],8'h00})
                    fail("PCM sample order, stereo channels or 24-bit scaling mismatch");
                consumed = consumed + 1;
            end else if (prior_started && expected_tick) begin
                if (aud_left !== held_left || aud_right !== held_right)
                    fail("underrun did not hold last sample");
                if (!underrun) fail("missing sticky underrun");
                underflow_ticks = underflow_ticks + 1;
            end
            if (dut.audio_sequence !== (consumed / SPF) ||
                dut.sample_count !== (consumed % SPF))
                fail("media timeline differs from real samples consumed");
            if (dut.meta_rd_bin != prior_meta) begin
                if (!boundary || presented >= meta_written)
                    fail("metadata displayed without refresh boundary or accepted frame");
                if (disp_buf_idx !== expected_meta[presented][1:0])
                    fail("metadata FIFO reordered buffer index");
                expected_sequence = consumed / SPF;
                delta = expected_sequence - expected_meta[presented][33:2];
                if (prior_started && delta[31]) fail("video ran ahead of audio sequence");
                if (!prior_started && consumed != 0) fail("initial display started after audio");
                presented = presented + 1;
            end else if (display_valid !== prior_display_valid || disp_buf_idx !== prior_display_idx)
                fail("display changed without popping frame at boundary");
            if (!expected_tick && (aud_left !== held_left || aud_right !== held_right))
                fail("audio changed between sample ticks");
        end
    end

    task send_pcm;
        input integer count;
        input integer random_gaps;
        integer k, gap;
        begin
            for (k=0; k<count; k=k+1) begin
                if (random_gaps) begin
                    random_value = $random(seed) & 32'h7fffffff;
                    gap = random_value % 5;
                    repeat (gap) @(negedge sd_clk);
                end
                @(negedge sd_clk);
                // This interface flags every valid&&!ready as overflow;
                // reserve one slot BEFORE asserting a producer strobe.
                while (!pcm_ready) @(negedge sd_clk);
                pcm_valid = 1;
                pcm_data = sample_word(next_word + epoch * 16384);
                @(posedge sd_clk);
                while (!pcm_ready) @(posedge sd_clk);
                @(negedge sd_clk);
                pcm_valid = 0;
                next_word = next_word + 1;
            end
        end
    endtask
    task send_frame;
        input [31:0] seq;
        input [1:0] idx;
        begin
            @(negedge sd_clk);
            frame_sequence = seq; frame_index = idx; frame_valid = 1;
            @(posedge sd_clk);
            while (!frame_ready) @(posedge sd_clk);
            @(negedge sd_clk); frame_valid = 0;
        end
    endtask
    task refresh;
        begin
            @(negedge vid_clk); frame_start = 1;
            @(negedge vid_clk); frame_start = 0;
        end
    endtask
    task flush_session;
        begin
            @(negedge sd_clk); enable = 0; pcm_valid = 0; frame_valid = 0;
            repeat (5) @(negedge vid_clk);
            written = 0; consumed = 0; meta_written = 0; presented = 0;
            next_word = 0; epoch = epoch + 1;
            if (dut.pcm_wr_bin != 0 || dut.pcm_rd_bin != 0 ||
                dut.meta_wr_bin != 0 || dut.meta_rd_bin != 0 || underrun || overflow)
                fail("flush failed to clear pointers and sticky flags");
            @(negedge sd_clk); enable = 1;
            repeat (8) @(negedge sd_clk);
            repeat (8) @(negedge vid_clk);
        end
    endtask
    integer frozen_count, frozen_seq, target, i;
    initial begin
        repeat (5) @(negedge vid_clk);
        sd_rst = 0; vid_rst = 0;
        repeat (1100) @(negedge vid_clk);
        if (ticks < 2) fail("disabled cadence stopped");
        enable = 1;
        repeat (8) @(negedge vid_clk);

        // Fill exactly to full with display stalled. Last 127 slots must not
        // admit a sector, yet still accept individual samples.
        send_frame(0, 1); send_frame(1, 2);
        if (frame_ready) fail("two-entry metadata FIFO did not backpressure");
        if (DEPTH >= 128) begin
            send_pcm(DEPTH-128, 0);
            if (!pcm_ready || !pcm_sector_ready) fail("sector threshold at free=128");
            send_pcm(1, 0);
            if (!pcm_ready || pcm_sector_ready) fail("sector threshold at free=127");
            send_pcm(127, 0);
        end else begin
            if (pcm_sector_ready) fail("small FIFO advertised a sector");
            send_pcm(DEPTH, 0);
        end
        repeat (5) @(negedge sd_clk);
        if (pcm_ready || pcm_sector_ready || overflow) fail("full FIFO/backpressure incorrect");
        if (display_valid || consumed != 0) fail("write completion started display/audio");
        @(negedge sd_clk); pcm_valid = 1; pcm_data = 32'hdeadbeef;
        repeat (3) @(negedge sd_clk);
        pcm_valid = 0;
        if (!overflow || written != DEPTH) fail("rejected write not sticky or corrupted occupancy");
        flush_session();

        // Both startup gates matter: refresh without enough PCM stays blank;
        // prefilled PCM alone also stays silent until frame_start.
        samples_per_frame = 0;
        send_frame(0, 1);
        send_pcm(START-1, 1);
        repeat (8) @(negedge vid_clk); refresh();
        if (display_valid) fail("startup ignored START_SAMPLES threshold");
        send_pcm(1, 0);
        repeat (1100) @(negedge vid_clk);
        if (display_valid || consumed != 0 || aud_left != 0 || aud_right != 0)
            fail("prefill started before refresh boundary");
        refresh();
        if (display_valid) fail("startup ignored zero/unsettled samples_per_frame");
        samples_per_frame = SPF;
        repeat (8) @(negedge vid_clk);
        refresh();
        if (!display_valid || disp_buf_idx != 1) fail("startup failed to present frame zero");
        send_frame(1, 2);
        wait (consumed == START);
        repeat (1700) @(negedge vid_clk);
        if (!underrun) fail("drained FIFO did not report underrun");
        frozen_count = dut.sample_count; frozen_seq = dut.audio_sequence;
        repeat (3) begin
            refresh(); repeat (600) @(negedge vid_clk);
        end
        if (dut.sample_count != frozen_count || dut.audio_sequence != frozen_seq || disp_buf_idx != 1)
            fail("underrun advanced audio/video media time");

        // Refill under random SD stalls; two complete source frames must be
        // consumed without skips. Refresh continuously so next-seq lookahead
        // must present frame 1 on the SAME edge as its last preceding sample.
        target = (ADDR_BITS == 11) ? 2*SPF+73 : 3*SPF+73;
        fork
            send_pcm(target-START, 1);
            begin
                while (consumed < target) begin
                    @(negedge vid_clk); frame_start = 1;
                end
                @(negedge vid_clk); frame_start = 0;
            end
            begin
                wait (consumed == SPF);
                #2;
                if (disp_buf_idx != 2 || presented != 2)
                    fail("tick/refresh coincidence delayed eligible frame by one refresh");
                send_frame(2, 3);
                wait (consumed == 2*SPF);
                #2;
                if (disp_buf_idx != 3 || presented != 3)
                    fail("frame 2 not synchronized with actual audio sequence");
            end
        join
        if (overflow) fail("compliant backpressure caused overflow");
        if (full_resume_writes == 0) fail("missing full-to-read-to-write recovery coverage");
        if (underflow_ticks == 0) fail("underrun coverage missing");
        repeat (1100) @(negedge vid_clk);

        // Leave stale PCM and metadata queued, then enter a fresh seq0 session.
        send_pcm(START+7, 0); send_frame(99, 0);
        flush_session();
        refresh();
        if (display_valid) fail("old metadata leaked on reentry");
        send_pcm(START, 1);
        repeat (8) @(negedge vid_clk); refresh();
        if (display_valid || consumed != 0) fail("PCM started without metadata");
        send_frame(0, 2);
        repeat (8) @(negedge vid_clk); refresh();
        wait (consumed == START);
        if (disp_buf_idx != 2 || presented != 1) fail("reentry retained old buffer/timeline");
        if (ADDR_BITS < 7 && pcm_sector_ready) fail("small FIFO advertised impossible sector capacity");
        passed = 1; done = 1;
        $display("CASE_PASS case=%0d spf=%0d depth=%0d ticks=%0d", CASE_ID, SPF, DEPTH, ticks);
    end
endmodule

// Production defaults: publish metadata before PCM, then emulate a 61ms
// video fetch with no audio supply. Actual ratio is 25MHz / 48kHz.
module av_pcm_production_case #(parameter integer SPF=3200)(
    output reg done=0, output reg passed=0
);
    reg sd_clk=0, vid_clk=0;
    always #3 sd_clk=~sd_clk;
    always #5 vid_clk=~vid_clk;
    reg sd_rst=1, vid_rst=1, enable=0;
    reg pcm_valid=0, frame_valid=0, frame_start=0;
    reg [31:0] pcm_data=0, frame_sequence=0;
    reg [1:0] frame_index=1;
    reg [15:0] samples_per_frame=SPF;
    wire pcm_ready, pcm_sector_ready, frame_ready, display_valid;
    wire [1:0] disp_buf_idx;
    wire aud_valid, underrun, overflow;
    wire [23:0] aud_left,aud_right;
    // Intentionally no parameter overrides: exercise shipped 4096/dynamic.
    av_pcm_player dut (
        .sd_clk(sd_clk),.sd_rst(sd_rst),.vid_clk(vid_clk),.vid_rst(vid_rst),
        .enable(enable),.samples_per_frame(samples_per_frame),
        .pcm_valid(pcm_valid),.pcm_data(pcm_data),.pcm_ready(pcm_ready),
        .pcm_sector_ready(pcm_sector_ready),.frame_valid(frame_valid),
        .frame_index(frame_index),.frame_sequence(frame_sequence),.frame_ready(frame_ready),
        .frame_start(frame_start),.display_valid(display_valid),.disp_buf_idx(disp_buf_idx),
        .aud_valid(aud_valid),.aud_left(aud_left),.aud_right(aud_right),
        .underrun(underrun),.overflow(overflow)
    );
    task fail;
        input [511:0] reason;
        begin $display("PRODUCTION_FAIL spf=%0d: %0s",SPF,reason); $finish; end
    endtask
    function [31:0] word_at;
        input integer n;
        reg [15:0] l,r;
        begin l=16'h9876+n*17; r=16'h3210-n*59; word_at={r,l}; end
    endfunction
    integer writes=0, reads=0, n, old_ptr, refresh_count=0;
    reg auto_refresh=0;
    reg [31:0] expected;
    always @(negedge vid_clk) begin
        if (auto_refresh) begin
            frame_start=(refresh_count==0);
            if (refresh_count==416666) refresh_count=0;
            else refresh_count=refresh_count+1;
        end
    end
    always @(posedge vid_clk) begin
        old_ptr=dut.pcm_rd_bin;
        #1;
        if (!dut.vid_clear && dut.pcm_rd_bin!=old_ptr) begin
            expected=word_at(reads);
            if (aud_left!=={expected[15:0],8'b0} || aud_right!=={expected[31:16],8'b0})
                fail("default-config PCM ordering mismatch");
            reads=reads+1;
            if (reads>writes) fail("consumed unwritten data");
        end
        if (!dut.vid_clear && (underrun || overflow)) fail("61ms fetch exhausted buffer or overflowed");
        if (disp_buf_idx==2 && reads<SPF) fail("display advanced ahead of media samples");
    end
    task fill;
        input integer count;
        integer k;
        begin
            for(k=0;k<count;k=k+1) begin
                @(negedge sd_clk);
                while(!pcm_ready) @(negedge sd_clk);
                pcm_valid=1; pcm_data=word_at(writes);
                @(posedge sd_clk); writes=writes+1;
                @(negedge sd_clk); pcm_valid=0;
            end
        end
    endtask
    task publish;
        input [31:0] seq;
        input [1:0] idx;
        begin
            @(negedge sd_clk); frame_valid=1; frame_sequence=seq; frame_index=idx;
            @(posedge sd_clk); while(!frame_ready) @(posedge sd_clk);
            @(negedge sd_clk); frame_valid=0;
        end
    endtask
    task refresh;
        begin
            @(negedge vid_clk); frame_start=1;
            @(negedge vid_clk); frame_start=0;
        end
    endtask
    initial begin
        repeat(5) @(negedge vid_clk); sd_rst=0; vid_rst=0; enable=1;
        repeat(8) @(negedge vid_clk);
        if (dut.PCM_ADDR_BITS!=12 || dut.START_SAMPLES!=0) fail("production defaults changed");
        publish(0,1);
        fill(SPF-1);
        repeat(8) @(negedge vid_clk); refresh();
        if (display_valid || reads!=0) fail("dynamic threshold started below one full audio frame");
        fill(1);
        // Oversized header config must not start even with a queued frame.
        samples_per_frame=4097;
        repeat(8) @(negedge vid_clk); refresh();
        if (display_valid) fail("unsupported spf above FIFO depth started");
        samples_per_frame=SPF;
        repeat(8) @(negedge vid_clk); refresh();
        if (!display_valid || disp_buf_idx!=1) fail("dynamic full-frame startup stalled");
        auto_refresh=1;
        // 1,525,000 pixels / 25MHz = 61ms, before the next frame's PCM.
        repeat(1525000) @(negedge vid_clk);
        publish(1,2);
        fill(SPF);
        wait(reads>=SPF+73 && disp_buf_idx==2);
        if (dut.audio_sequence!=1) fail("production media sequence mismatch");
        enable=0;
        repeat(8) @(negedge vid_clk);
        if (display_valid || aud_left!=0 || aud_right!=0) fail("production flush failed");
        passed=1; done=1;
        $display("PRODUCTION_PASS spf=%0d depth=4096 startup=dynamic reads=%0d",SPF,reads);
    end
endmodule

module tb_av_pcm_player;
    wire [6:0] done, passed;
    reg testdone = 0, test_passed = 0;
    av_pcm_case #(.SPF(1920), .CASE_ID(0)) c0(done[0],passed[0]);
    av_pcm_case #(.SPF(2000), .CASE_ID(1)) c1(done[1],passed[1]);
    av_pcm_case #(.SPF(2400), .CASE_ID(2)) c2(done[2],passed[2]);
    av_pcm_case #(.SPF(3200), .CASE_ID(3)) c3(done[3],passed[3]);
    av_pcm_case #(.SPF(29), .CASE_ID(4), .ADDR_BITS(4)) c4(done[4],passed[4]);
    av_pcm_production_case #(.SPF(3200)) p15(done[5],passed[5]);
    av_pcm_production_case #(.SPF(4000)) p12(done[6],passed[6]);
    initial begin
        wait (&done);
        testdone = 1; test_passed = &passed;
        $display("AV_PCM_TESTDONE PASS=%0d", test_passed);
        $finish;
    end
    initial begin
        #60000000;
        $display("AV_PCM_TIMEOUT done=%b passed=%b", done, passed);
        $finish;
    end
endmodule
