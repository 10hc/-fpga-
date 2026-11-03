`timescale 1ns/1ps
// Full-size VID2 integration, real level-sensitive SD sequencer, no SDRAM model.
// Run via run_av_video_checks.tcl. All failures leave explicit result flags.
module tb_av_video #(parameter integer REAL_RATE=0, PCM_BITS=12);
    localparam integer SD_HZ=REAL_RATE?100000000:40000000;
    localparam integer VID_HZ=REAL_RATE?25000000:10000000;
    localparam [31:0] BASE=32'd47984;
    reg clk=0, vid_clk=0, rst=1, enable=0, session_clear=1;
    always #5 clk=~clk;
    always #20 vid_clk=~vid_clk;
    reg test_done=0, test_passed=0;
    integer fps=20, scenario=0, badfield=0, spf, audio_sectors, stride;
    integer seed=20261031, sessions=0;
    integer gap_clocks=0, byte_gap=0;
    integer words=0, completed=0, meta_count=0, pcm_count=0;
    integer emitted=0, consumed_sequence, presented_sequence=-1;
    wire req, wr_req, wr_en, abort, active, has_audio, legacy_display;
    wire [31:0] lba, pixel, pcm_data, frame_sequence;
    wire [15:0] samples_per_frame;
    wire [3:0] state_dbg;
    wire [1:0] write_idx, frame_index, display_idx;
    wire pcm_valid, pcm_ready, sector_ready, frame_valid, meta_ready;
    wire display_valid, aud_valid, underrun, overflow;
    wire [23:0] aud_left, aud_right;
    reg ack=0, finish_toggle=0, ready=1;
    reg hold_space=0, hold_meta=0, pause_card=0;
    reg frame_start=0;
    wire [7:0] sd_data;
    wire sd_valid, sd_end, initialized, cmd_req, block_req;
    wire [47:0] cmd;
    reg cmd_ack=0, block_valid=0, block_ack=0;
    reg [7:0] block_data=0;
    video_player #(.CLK_FREQ_HZ(SD_HZ),.WAIT_LIMIT(24'd200000)) dut (
        .clk(clk),.rst(rst),.enable(enable),.file_valid(1'b1),.file_sector(BASE),
        .sec_read(req),.sec_read_addr(lba),.sec_read_data(sd_data),
        .sec_read_data_valid(sd_valid),.sec_read_end(sd_end),
        .wr_req(wr_req),.wr_req_ack(ack),.write_ready(ready),.wr_en(wr_en),
        .wr_data(pixel),.wr_finish(finish_toggle),.wr_abort(abort),
        .display_valid(legacy_display),.disp_buf_idx(),.write_buf_idx(write_idx),
        .active(active),.st_dbg(state_dbg),.has_audio(has_audio),
        .audio_samples_per_frame(samples_per_frame),.pcm_valid(pcm_valid),
        .pcm_data(pcm_data),.pcm_sector_ready(sector_ready && !hold_space),
        .frame_valid(frame_valid),.frame_index(frame_index),
        .frame_sequence(frame_sequence),.frame_ready(meta_ready && !hold_meta));
    av_pcm_player #(.VID_CLK_HZ(VID_HZ),.PCM_ADDR_BITS(PCM_BITS),.START_SAMPLES(REAL_RATE?0:128)) audio (
        .sd_clk(clk),.sd_rst(rst),.vid_clk(vid_clk),.vid_rst(rst),
        .enable(enable && has_audio),.samples_per_frame(samples_per_frame),
        .pcm_valid(pcm_valid),.pcm_data(pcm_data),.pcm_ready(pcm_ready),
        .pcm_sector_ready(sector_ready),.frame_valid(frame_valid && !hold_meta),
        .frame_index(frame_index),.frame_sequence(frame_sequence),.frame_ready(meta_ready),
        .frame_start(frame_start),.display_valid(display_valid),.disp_buf_idx(display_idx),
        .aud_valid(aud_valid),.aud_left(aud_left),.aud_right(aud_right),
        .underrun(underrun),.overflow(overflow));
    sd_card_sec_read_write #(.SPI_LOW_SPEED_DIV(0)) sector (
        .clk(clk),.rst(rst),.sd_init_done(initialized),.sd_sec_read(req),
        .sd_sec_read_addr(lba),.sd_sec_read_data(sd_data),
        .sd_sec_read_data_valid(sd_valid),.sd_sec_read_end(sd_end),
        .sd_sec_write(1'b0),.sd_sec_write_addr(32'd0),.sd_sec_write_data(8'd0),
        .sd_sec_write_data_req(),.sd_sec_write_end(),.spi_clk_div(),
        .cmd_req(cmd_req),.cmd_req_ack(cmd_ack),.cmd_req_error(1'b0),.cmd(cmd),
        .cmd_r1(),.cmd_data_len(),.block_read_req(block_req),
        .block_read_valid(block_valid),.block_read_data(block_data),
        .block_read_req_ack(block_ack),.block_write_req(),.block_write_data(),
        .block_write_data_rd(1'b0),.block_write_req_ack(1'b0));

    task fail;
        input [8*120-1:0] message;
        begin
            $display("AV_VIDEO_FAIL fps=%0d scenario=%0d bad=%0d: %0s",fps,scenario,badfield,message);
            test_done=1; test_passed=0; $finish;
        end
    endtask
    function [15:0] source_pixel;
        input integer f,n;
        begin source_pixel=(n*73) ^ (n>>7) ^ (f*16'hb6d3); end
    endfunction
    function [31:0] sample_value;
        input integer f,n;
        reg [15:0] l,r;
        begin
            l=16'h1000 ^ (f*16'h2100) ^ n;
            r=16'he731 ^ (n*3) ^ (f*16'h1200);
            sample_value={r,l};
        end
    endfunction
    function [7:0] file_byte;
        input integer address,offset;
        integer f,s,n;
        reg [15:0] p;
        reg [31:0] a;
        begin
            file_byte=0;
            if(address==BASE) begin
                case(offset)
                    0:file_byte="V";1:file_byte="I";2:file_byte="D";
                    3:file_byte=(scenario==6)?"1":"2";
                    4:file_byte=8'h40;5:file_byte=1;6:file_byte=8'hf0;
                    8:file_byte=fps;10:file_byte=2;
                    14:file_byte=audio_sectors;
                    16:file_byte=8'h80;17:file_byte=8'hbb;
                    20:file_byte=spf%256;21:file_byte=spf/256;
                    22:file_byte=2;23:file_byte=16;
                    24:file_byte=8'h2c;25:file_byte=1;26:file_byte=1;
                endcase
                if(scenario==1) case(badfield)
                    0:if(offset==4)file_byte=0;
                    1:if(offset==6)file_byte=0;
                    2:if(offset==8)file_byte=23;
                    3:if(offset==10)file_byte=0;
                    4:if(offset==14)file_byte=audio_sectors+1;
                    5:if(offset==16)file_byte=0;
                    6:if(offset==20)file_byte=(spf+1)%256;
                    7:if(offset==22)file_byte=1;
                    8:if(offset==23)file_byte=24;
                    9:if(offset==24)file_byte=0;
                    10:if(offset==26)file_byte=0;
                    11:if(offset==27)file_byte=1;
                endcase
            end else begin
                f=(address-BASE-1)/stride;
                s=(address-BASE-1)%stride;
                if(s<300) begin
                    p=source_pixel(f,s*256+offset/2);
                    file_byte=offset[0]?p[15:8]:p[7:0];
                end else begin
                    n=(s-300)*128+offset/4;
                    a=(n<spf)?sample_value(f,n):32'hdeadc0de;
                    file_byte=a[(offset%4)*8 +: 8];
                end
            end
        end
    endfunction

    integer commands=0, payload_commands=0, command_delay=0;
    integer byte_state=0, byte_index=0, current_lba=0, sector_length=512;
    integer audio_commands=0, video_commands=0, expected_lba;
    // Request acceptance is the real controller's level-sensitive WAIT state.
    // No request edge detector: a stale HIGH causes a duplicate and fails below.
    always @(posedge clk) begin
        cmd_ack<=0;
        if(rst || session_clear) begin
            commands<=0;payload_commands<=0;audio_commands<=0;video_commands<=0;
        end
        if(rst) command_delay<=0;
        else begin
            if(!cmd_req)command_delay<=0;
            else if(command_delay==3)begin cmd_ack<=1;command_delay<=4;end
            else if(command_delay<3)command_delay<=command_delay+1;
            if(cmd_ack && cmd_req && cmd[45:40]==17)begin
                expected_lba=commands<2?BASE:BASE+1+(payload_commands%(2*stride));
                if(cmd[39:8]!==expected_lba)fail("duplicate, skipped, or wrong sector LBA");
                commands<=commands+1;current_lba<=cmd[39:8];sector_length<=512;
                if(commands>=2) begin
                    payload_commands<=payload_commands+1;
                    if(((cmd[39:8]-BASE-1)%stride)>=300)begin
                        audio_commands<=audio_commands+1;
                        if(meta_count<=((payload_commands/stride)))fail("audio read before metadata acceptance");
                        if(scenario==2 && audio_commands==0)sector_length<=63;
                    end else video_commands<=video_commands+1;
                end
            end
            // Check reservation on the acceptance edge, before command latency.
            if(sector.state==17 && req && lba>BASE &&
               ((lba-BASE-1)%stride)>=300 && enable && (!sector_ready || hold_space))
                fail("audio sector request without 128 free samples");
        end
    end
    always @(posedge clk) begin
        block_valid<=0;block_ack<=0;
        if(rst)begin byte_state<=0;byte_index<=0;byte_gap<=0;end
        else case(byte_state)
            0:if(block_req)begin byte_state<=1;byte_index<=0;byte_gap<=0;end
            1:if(byte_gap==0)begin
                // Deliberately unconditional: SD never pauses for write_ready.
                if(byte_index<sector_length)begin
                    block_data<=file_byte(current_lba,byte_index);block_valid<=1;
                end
                if(byte_index==sector_length+1)begin block_ack<=1;byte_state<=2;end
                else begin byte_index<=byte_index+1;byte_gap<=gap_clocks;end
            end else byte_gap<=byte_gap-1;
            2:if(!block_req)byte_state<=0;
        endcase
    end

    integer stalls=0, early_finishes=0, normal_finishes=0;
    integer total_pixels=0, total_pcm=0, total_present=0, total_aud=0;
    integer pcm_cycle=0, previous_pcm_cycle=-100;
    integer max_pcm_gap=0, first_meta_cycle=0, last_meta_cycle=0;
    reg [1:0] indices[0:63];
    reg [15:0] expected565;
    reg [31:0] expected_pixel;
    reg [31:0] random_bits=32'hf12a76c9;
    reg was_meta_stalled=0;
    reg [33:0] stalled_meta;
    always @(negedge clk) begin
        random_bits={random_bits[30:0],random_bits[31]^random_bits[21]^random_bits[1]^random_bits[0]};
        ready=(random_bits[3:0]!=0 && random_bits[8:4]!=0);
        if(enable && !ready)stalls=stalls+1;
    end
    always @(posedge clk) begin
        ack<=wr_req; pcm_cycle<=pcm_cycle+1;
        if(session_clear)begin
            words<=0;completed<=0;meta_count<=0;pcm_count<=0;was_meta_stalled<=0;
            previous_pcm_cycle<=-100;
        end else begin
            if(abort && scenario!=1 && scenario!=2)fail("unexpected player halt");
            if(wr_en) begin
                if(!ready)fail("wr_en asserted against ready");
                expected565=source_pixel(completed%2,(words/1280)*320+(words%640)/2);
                expected_pixel={expected565[15:11],expected565[15:13],expected565[10:5],
                                expected565[10:9],expected565[4:0],expected565[4:2],8'd0};
                if(pixel!==expected_pixel)begin
                    $display("pixel frame=%0d word=%0d got=%h expected=%h",completed,words,pixel,expected_pixel);
                    fail("RGB565 expansion, duplicate, or ordering error");
                end
                total_pixels<=total_pixels+1;
                if(words==300000 && completed%2==0)begin
                    finish_toggle<=~finish_toggle;early_finishes<=early_finishes+1;
                end
                if(words==307199)begin
                    words<=0;completed<=completed+1;
                    if(completed%2==1)begin finish_toggle<=~finish_toggle;normal_finishes<=normal_finishes+1;end
                end else words<=words+1;
            end
            if(was_meta_stalled && enable && (!frame_valid || {frame_sequence,frame_index}!==stalled_meta))
                fail("metadata changed while backpressured");
            was_meta_stalled<=frame_valid && !(meta_ready && !hold_meta);
            stalled_meta<={frame_sequence,frame_index};
            if(frame_valid && meta_ready && !hold_meta)begin
                if(frame_sequence!==meta_count)fail("frame sequence reset or skipped across EOF loop");
                if(completed!=meta_count+1 || words!=0)fail("metadata before complete video write");
                if(frame_index!==write_idx)fail("metadata buffer index mismatch");
                indices[meta_count%64]<=frame_index;
                if(meta_count==0)first_meta_cycle<=pcm_cycle;
                last_meta_cycle<=pcm_cycle;
                if(REAL_RATE)$display("AV_RATE_FRAME seq=%0d cycle=%0d pcm=%0d emitted=%0d",meta_count,pcm_cycle,pcm_count,emitted);
                meta_count<=meta_count+1;
            end
            if(pcm_valid)begin
                if(!enable)fail("PCM emitted while disabled");
                if(!pcm_ready)fail("PCM FIFO overflow against reservation");
                if(pcm_cycle-previous_pcm_cycle<4)fail("PCM output faster than one sample per four clocks");
                if(pcm_count>0 && pcm_cycle-previous_pcm_cycle>max_pcm_gap)
                    max_pcm_gap<=pcm_cycle-previous_pcm_cycle;
                previous_pcm_cycle<=pcm_cycle;
                if(scenario==2)fail("short unverified audio sector leaked PCM");
                if(pcm_data!==sample_value((pcm_count/spf)%2,pcm_count%spf))begin
                    $display("pcm n=%0d got=%h expected=%h",pcm_count,pcm_data,sample_value((pcm_count/spf)%2,pcm_count%spf));
                    fail("PCM sample, channel order, or padding error");
                end
                if(meta_count<=pcm_count/spf)fail("PCM before metadata");
                pcm_count<=pcm_count+1;total_pcm<=total_pcm+1;
            end
            if(overflow)fail("av_pcm FIFO overflow");
        end
    end

    integer frame_acc=0, cadence_acc=0, aud_count=0, present_count=0;
    integer start_buffer=0, underflow_ticks=0;
    reg prior_display=0;
    reg [1:0] prior_idx=0;
    reg [23:0] previous_left=0, previous_right=0;
    reg [31:0] expected_sample;
    reg expected_tick, at_frame_start;
    // Exact rational 60Hz read window, independent of source video fps.
    always @(negedge vid_clk)begin
        if(rst)begin frame_acc=0;frame_start=0;end
        else if(frame_acc+60>=VID_HZ)begin frame_acc=frame_acc+60-VID_HZ;frame_start=1;end
        else begin frame_acc=frame_acc+60;frame_start=0;end
    end
    always @(posedge vid_clk)begin
        at_frame_start=frame_start;
        if(rst)cadence_acc=0;
        else begin
            expected_tick=(cadence_acc+48000>=VID_HZ);
            cadence_acc=cadence_acc+48000;
            if(expected_tick)cadence_acc=cadence_acc-VID_HZ;
        end
        #1;
        if(!rst)begin
            if(aud_valid!==expected_tick)fail("48k cadence accumulated drift or missing tick");
            if(aud_valid)begin aud_count=aud_count+1;total_aud=total_aud+1;end
        end
        if(session_clear)begin
            emitted=0;present_count=0;presented_sequence=-1;
            prior_display=0;prior_idx=0;previous_left=0;previous_right=0;
        end else if(enable && has_audio)begin
            if((display_valid!=prior_display || display_idx!=prior_idx) && !at_frame_start)
                fail("display changed outside frame_start");
            if(display_valid && (!prior_display || display_idx!=prior_idx))begin
                if(present_count>=meta_count)fail("display without complete metadata");
                if(display_idx!==indices[present_count%64])fail("display buffer ordering error");
                // Presentation can coincide with the sample completing an audio
                // frame. Include that edge's independently verified sample.
                expected_sample=sample_value((emitted/spf)%2,emitted%spf);
                consumed_sequence=(emitted+((aud_valid &&
                    {aud_right,aud_left}==={expected_sample[31:16],8'd0,expected_sample[15:0],8'd0})?1:0))/spf;
                if(present_count>consumed_sequence)begin
                    $display("present=%0d emitted=%0d pcm=%0d",present_count,emitted,pcm_count);
                    fail("video displayed ahead of audio consumed sequence");
                end
                if(present_count==0)begin
                    start_buffer=pcm_count-emitted;
                    if(start_buffer<128)fail("first frame before audio preload");
                    if(REAL_RATE && start_buffer<spf)fail("real-rate start before complete first audio frame preload");
                end
                presented_sequence=present_count;present_count=present_count+1;total_present=total_present+1;
            end
            if(aud_valid)begin
                if(!display_valid)begin
                    if(aud_left!==0 || aud_right!==0)fail("non-silent output before initial preload");
                end else if(aud_left!=0 || aud_right!=0)begin
                    expected_sample=sample_value((emitted/spf)%2,emitted%spf);
                    if({aud_right,aud_left}==={expected_sample[31:16],8'd0,expected_sample[15:0],8'd0})
                        emitted=emitted+1;
                    else if(aud_left===previous_left && aud_right===previous_right)begin
                        // Initial start edge may still be silent; underrun must hold the head.
                        if(!underrun && emitted!=0)fail("duplicated audio sample without underrun");
                        underflow_ticks=underflow_ticks+1;
                    end else fail("audible sample lost, reordered, or channels swapped");
                end
                previous_left=aud_left;previous_right=aud_right;
            end
            prior_display=display_valid;prior_idx=display_idx;
        end
    end

    task start_session;
        begin
            session_clear=1;
            repeat(12)@(negedge clk);
            session_clear=0;enable=1;sessions=sessions+1;
        end
    endtask
    task stop_session;
        integer n, stopped_commands;
        begin
            @(negedge clk);enable=0;pause_card=0;hold_meta=0;
            n=0;
            while((active || req || block_req) && n<10000)begin @(negedge clk);n=n+1;end
            if(active || req || block_req)fail("disable deadlocked or did not drain current sector");
            repeat(40)@(negedge clk);
            if(display_valid || pcm_valid || frame_valid)fail("disable did not clear presentation/PCM/metadata");
            if(aud_left!==0 || aud_right!==0)fail("disable did not clear audio output");
            stopped_commands=commands;
            repeat(100)@(negedge clk);
            if(commands!=stopped_commands)fail("new SD transaction after disable");
        end
    endtask
    integer before_commands, n;
    initial begin
        if($value$plusargs("FPS=%d",fps))begin end
        if($value$plusargs("SCENARIO=%d",scenario))begin end
        if($value$plusargs("BAD=%d",badfield))begin end
        if($value$plusargs("GAP=%d",gap_clocks))begin end
        if($value$plusargs("SEED=%d",seed))random_bits=seed;
        spf=48000/fps;audio_sectors=(spf+127)/128;
        if(scenario==6)audio_sectors=0;
        stride=300+audio_sectors;
        repeat(8)@(negedge clk);rst=0;
        force sector.por_cnt=20'd1000000;
        wait(initialized);release sector.por_cnt;
        start_session;
        if(scenario==1)begin
            wait(abort);
            if(wr_req || wr_en || pcm_count || meta_count || payload_commands)fail("malformed header started payload");
            stop_session;
        end else if(scenario==2)begin
            wait(abort);
            if(pcm_count!=0 || meta_count!=1 || audio_commands!=1)fail("short sector did not halt atomically");
            stop_session;
        end else if(scenario==3)begin
            hold_space=1;
            wait(meta_count==1 && state_dbg==9);
            repeat(100)@(negedge clk);
            if(audio_commands!=0)fail("space wait issued audio read");
            stop_session;hold_space=0;
        end else if(scenario==4)begin
            wait(audio_commands==1 && byte_state==1 && byte_index==40);
            stop_session;
        end else if(scenario==7 || scenario==8)begin
            // No induced stalls/underflow: four complete real-rate frames.
            wait(pcm_count>=4*spf);
            wait(emitted>=4*spf);
            $display("AV_RATE_RESULT fps=%0d gap=%0d depth=%0d max_pcm_gap_cycles=%0d meta_span_cycles=%0d underflow_ticks=%0d underrun=%0d",
                     fps,gap_clocks,2**PCM_BITS,max_pcm_gap,last_meta_cycle-first_meta_cycle,underflow_ticks,underrun);
            if(scenario==7 && (underrun || underflow_ticks!=0))fail("real-rate steady playback underrun");
            if(scenario==8 && !underrun)fail("diagnostic did not reproduce expected bandwidth underrun");
            // Stop during the next audio block, avoiding a partial video exit.
            wait(state_dbg==9 || state_dbg==10);stop_session;
        end else if(scenario==6)begin
            wait(legacy_display);
            if(has_audio || pcm_count || meta_count || audio_commands)fail("VID1 falsely enabled audio");
            stop_session;
        end else begin
            // Hold first metadata to prove stable valid/ready and no early PCM.
            hold_meta=1;wait(frame_valid);
            before_commands=commands;
            repeat(99)@(negedge clk);
            if(commands!=before_commands || pcm_count!=0)fail("metadata backpressure did not block audio");
            hold_meta=0;
            wait(present_count>=3 && emitted>=3*spf+160 && meta_count>=4);
            if(early_finishes<2 || normal_finishes<1 || stalls<1000)fail("insufficient write completion/backpressure coverage");
            // Pause at a video group boundary. Producer cannot replenish PCM;
            // all 48k ticks continue and underrun must preserve sample sequence.
            @(negedge clk);ready=0;
            force ready=1'b0;
            wait(underrun);
            n=emitted;
            repeat(12000)@(negedge clk);
            if(emitted!=n)fail("underrun skipped samples or advanced audio master");
            release ready;
            wait(emitted>=n+128);
            // Exit inside the next actual PCM emission, not at a frame boundary.
            wait(pcm_valid);stop_session;
            start_session;
            wait(present_count>=1 && emitted>=128);
            if(meta_count!=1 || completed!=1)fail("reenter did not restart at frame zero");
            wait(pcm_valid);stop_session;
        end
        test_done=1;test_passed=1;
        $display("AV_VIDEO_PASS fps=%0d scenario=%0d bad=%0d sessions=%0d pixels=%0d pcm=%0d present=%0d ticks=%0d hold=%0d",
                 fps,scenario,badfield,sessions,total_pixels,total_pcm,total_present,total_aud,underflow_ticks);
        $finish;
    end
    initial begin #900000000;fail("watchdog timeout");end
endmodule
