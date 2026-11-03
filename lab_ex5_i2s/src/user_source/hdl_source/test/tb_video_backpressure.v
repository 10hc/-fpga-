`timescale 1ns/1ps
// Direct FIFO semantics: wr_en is consumed on this edge without another ready
// gate. Unique pixels detect loss, duplication and ordering across 5-sector groups.
module tb_video_backpressure;
    reg clk=0;
    always #5 clk=~clk;
    reg rst=1, enable=0, write_ready=0;
    reg test_passed=0;
    wire sec_read, wr_req, wr_en, wr_abort, display_valid, active;
    wire [31:0] sec_addr, wr_data;
    wire [1:0] disp_idx, write_idx;
    wire [3:0] state_dbg;
    reg [7:0] sec_data=0;
    reg sec_valid=0, sec_end=0, wr_ack=0, wr_finish=0;
    video_player #(.CLK_FREQ_HZ(1_000_000), .WAIT_LIMIT(24'd2000)) dut (
        .clk(clk), .rst(rst), .enable(enable),
        .file_valid(1'b1), .file_sector(32'd100),
        .sec_read(sec_read), .sec_read_addr(sec_addr),
        .sec_read_data(sec_data), .sec_read_data_valid(sec_valid),
        .sec_read_end(sec_end), .wr_req(wr_req), .wr_req_ack(wr_ack),
        .write_ready(write_ready), .wr_en(wr_en), .wr_data(wr_data),
        .wr_finish(wr_finish), .wr_abort(wr_abort),
        .display_valid(display_valid), .disp_buf_idx(disp_idx),
        .write_buf_idx(write_idx), .active(active), .st_dbg(state_dbg)
    );

    function [15:0] source_pixel;
        input integer frame_number, pixel_number;
        begin
            source_pixel=((pixel_number*73) ^ (pixel_number>>7) ^
                          (frame_number*16'hb6d3)) & 16'hffff;
        end
    endfunction
    function [7:0] file_byte;
        input integer address, offset;
        integer word_number;
        reg [15:0] pixel;
        begin
            file_byte=0;
            if(address==100) begin
                case(offset)
                    0:file_byte="V"; 1:file_byte="I"; 2:file_byte="D"; 3:file_byte="1";
                    4:file_byte=8'h40; 5:file_byte=1; 6:file_byte=8'hf0;
                    8:file_byte=25; 10:file_byte=2;
                endcase
            end else begin
                word_number=((address-101)%300)*256+offset/2;
                pixel=source_pixel((address-101)/300,word_number);
                file_byte=offset[0] ? pixel[15:8] : pixel[7:0];
            end
        end
    endfunction

    integer card_state=0, card_addr=0, byte_idx=0;
    integer requests=0, payload_requests=0, bytes_received=0;
    reg previous_read=0;
    always @(posedge clk) begin
        if(rst) begin
            card_state<=0; sec_valid<=0; sec_end<=0; previous_read<=0;
            requests<=0; payload_requests<=0; bytes_received<=0;
        end else begin
            previous_read<=sec_read; sec_valid<=0; sec_end<=0;
            case(card_state)
                0:if(sec_read && !previous_read) begin
                    if(requests<2) begin
                        if(sec_addr!=100) $fatal(1,"Header address mismatch");
                    end else begin
                        if(sec_addr!=101+(payload_requests%600))
                            $fatal(1,"Sector skipped/repeated: %0d payload=%0d",sec_addr,payload_requests);
                        payload_requests<=payload_requests+1;
                    end
                    requests<=requests+1; card_addr<=sec_addr;
                    byte_idx<=0; card_state<=1;
                end
                1:begin
                    // SD cannot be paused by FIFO ready once a sector starts.
                    sec_data<=file_byte(card_addr,byte_idx); sec_valid<=1;
                    bytes_received<=bytes_received+1;
                    if(byte_idx==511) card_state<=2;
                    else byte_idx<=byte_idx+1;
                end
                2:begin sec_end<=1; card_state<=0; end
            endcase
        end
    end

    integer words=0, frames=0, commits=0;
    reg [1:0] previous_disp=0;
    integer source_index;
    reg [15:0] expected565;
    reg [31:0] expected32;
    always @(posedge clk) begin
        if(rst) begin
            wr_ack<=0; wr_finish<=0; words<=0; frames<=0;
            commits<=0; previous_disp<=0;
        end else begin
            wr_ack<=wr_req;
            previous_disp<=disp_idx;
            if(disp_idx!=previous_disp) begin
                if(frames<=commits) $fatal(1,"Published before all pixels accepted");
                commits<=commits+1;
            end
            if(wr_abort) $fatal(1,"Long ready stall caused abort");
            // Deliberately NO ready gate at this FIFO sink.
            if(wr_en) begin
                if(!write_ready) $fatal(1,"Write strobe while not ready");
                source_index=(words/1280)*320+(words%640)/2;
                expected565=source_pixel(frames%2,source_index);
                expected32={expected565[15:11],expected565[15:13],
                            expected565[10:5],expected565[10:9],
                            expected565[4:0],expected565[4:2],8'd0};
                if(wr_data!==expected32)
                    $fatal(1,"Lost/duplicated pixel: frame=%0d word=%0d got=%h expected=%h",
                           frames,words,wr_data,expected32);
                if(words==307199) begin
                    words<=0; frames<=frames+1; wr_finish<=~wr_finish;
                end else words<=words+1;
            end
        end
    end

    integer seed=1, seed_arg=1, stall_left=0, random_stalls=0, held_cycles=0;
    integer ready_cycles=0, i;
    reg [31:0] random_bits;
    reg [7:0] directed_done=0;
    reg [3:0] stalled_divs=0;
    integer directed_words[0:7];
    reg reset_probe=0;
    initial begin
        if($value$plusargs("SEED=%d",seed_arg)) seed=seed_arg;
        directed_words[0]=0; directed_words[1]=1; directed_words[2]=639;
        directed_words[3]=1279; directed_words[4]=5119;
        directed_words[5]=307199; directed_words[6]=5120; directed_words[7]=307198;
    end
    always @(negedge clk) begin
        if(rst) begin write_ready=0; stall_left=0; end
        else if(reset_probe) write_ready=0;
        else begin
            random_bits=$random(seed);
            if(stall_left>0) begin
                stall_left=stall_left-1; write_ready=0;
            end else begin
                write_ready=1;
                // Ready drops AFTER a word is staged, before the consuming edge.
                if(dut.pixel_pending) begin
                    for(i=0;i<8;i=i+1)
                        if(frames==0 && words==directed_words[i] && !directed_done[i]) begin
                            stall_left=8192+i*257; directed_done[i]=1;
                        end
                    if(stall_left==0 && random_bits[10:0]==0) begin
                        stall_left=2048+random_bits[23:11]%8192;
                        random_stalls=random_stalls+1;
                    end
                end
                // Pause while a sector is already streaming too.
                if(requests==3 && card_state==1 && byte_idx==17 && ready_cycles==0) begin
                    stall_left=12000; ready_cycles=1;
                end
                if(stall_left>0) write_ready=0;
                if(state_dbg==4 && !dut.pixel_pending && random_bits[4:0]==0) begin
                    stall_left=1+random_bits[7:5]; write_ready=0;
                end
            end
        end
    end

    reg [15:0] held_position;
    reg [31:0] held_data, held_address;
    reg [8:0] held_group;
    reg held_pending;
    always @(posedge clk) begin
        if(!rst && state_dbg==4 && !write_ready) begin
            held_position={dut.o_sl,dut.o_px,dut.o_dup,dut.o_row,dut.o_div,1'b0};
            held_data=wr_data; held_address=dut.addr;
            held_group=dut.grp_grp; held_pending=dut.pixel_pending;
            held_cycles=held_cycles+1;
            stalled_divs[dut.o_div]=1'b1;
            #1;
            if({dut.o_sl,dut.o_px,dut.o_dup,dut.o_row,dut.o_div,1'b0}!==held_position ||
               wr_data!==held_data || dut.addr!==held_address ||
               dut.grp_grp!==held_group || dut.pixel_pending!==held_pending ||
               state_dbg!=4 || sec_read)
                $fatal(1,"Stall changed pending data/counters or requested next sectors");
        end
    end

    integer stopped_requests;
    initial begin
        repeat(8) @(negedge clk); rst=0; enable=1;
        wait(frames==2 && words>10000);
        // Disable during a pending long stall must still drain the accepted frame.
        wait(dut.pixel_pending && !write_ready);
        @(negedge clk); enable=0;
        wait(!active);
        repeat(20) @(negedge clk);
        if(frames!=3 || commits!=3 || words!=0 ||
           payload_requests!=900 || bytes_received!=902*512 ||
           directed_done!=8'hff || random_stalls<20 || held_cycles<100000 ||
           (stalled_divs & 4'b1101)!=4'b1101)
            $fatal(1,"Coverage/count failure: frames=%0d commits=%0d words=%0d payload=%0d bytes=%0d directed=%h random=%0d held=%0d divs=%b",
                   frames,commits,words,payload_requests,bytes_received,directed_done,random_stalls,held_cycles,stalled_divs);
        stopped_requests=requests;
        repeat(2000) @(negedge clk);
        if(requests!=stopped_requests || wr_en || wr_req || sec_read)
            $fatal(1,"Disabled player did not stop");
        $display("PASS 3 frames / 921600 exact pixels, EOF loop, disable drain, sector and group boundaries");
        $display("PASS seed=%0d random_long_stalls=%0d held_cycles=%0d directed=%h",
                 seed_arg,random_stalls,held_cycles,directed_done);
        // Reset must discard a held word rather than emit it after reset.
        // Start a fresh session, including the sector-order scoreboard.
        @(negedge clk); rst=1; enable=0;
        repeat(4) @(negedge clk);
        rst=0; enable=1;
        wait(dut.pixel_pending);
        reset_probe=1;
        @(negedge clk);
        @(negedge clk); rst=1; enable=0;
        #1;
        if(wr_en) $fatal(1,"Reset leaked a pending write");
        repeat(4) @(negedge clk);
        if(dut.pixel_pending || wr_data!=0) $fatal(1,"Reset did not clear pending word");
        $display("ALL VIDEO BACKPRESSURE TESTS PASSED");
        test_passed=1;
        $finish;
    end
    initial begin #200000000; $fatal(1,"Backpressure test timed out: frame=%0d words=%0d",frames,words); end
endmodule
