`timescale 1ns/1ps
module tb_grid_mapping;
    reg clk=0, rst=1;
    always #4 clk=~clk;
    reg req=0, cancel=0, enable_effect=1, write_busy=0;
    reg [1:0] effect=1, read_index=1;
    reg [9:0] offset=0, fifo_used=0;
    reg [20:0] old_base=0, new_base=307200;
    wire ack, finish, rd_en;
    wire [20:0] addr;
    reg test_done=0, test_passed=0;
    integer words=0, row, col, expected_addr, latched_offset=0;
    integer n, direction;
    reg check_grid=1;
    frame_fifo_read #(.BURST_SIZE(256)) reader (
        .rst(rst),.mem_clk(clk),.Sdr_init_done(1'b1),
        .Sdr_init_ref_vld(1'b0),.Sdr_busy(1'b0),.Sdr_rd_en(1'b0),
        .App_wr_busy(write_busy),.O_rd_busy(),.App_rd_en(rd_en),.App_rd_addr(addr),
        .read_req(req),.read_req_ack(ack),.read_finish(finish),
        .read_addr_0(21'd0),.read_addr_1(21'd307200),
        .read_addr_2(21'd614400),.read_addr_3(21'd921600),
        .read_addr_index(read_index),.read_len(21'd307200),
        .fifo_aclr(),.wrusedw(fifo_used),
        .trans_abort(cancel),.trans_en(enable_effect),
        .trans_effect(effect),.trans_offset(offset),
        .trans_old_base(old_base),.trans_new_base(new_base),
        .dbg_state(),.dbg_wrbusy(),.dbg_req2(),.dbg_init()
    );
    always @(posedge clk) if(!rst && rd_en) begin
        row=words/640; col=words%640;
        expected_addr=read_index*307200+words;
        if(check_grid)
            expected_addr=((row/16+col/16)<latched_offset ? new_base:old_base)+words;
        if(addr !== expected_addr[20:0])
            $fatal(1,"GRID ADDRESS mismatch threshold=%0d word=%0d row=%0d col=%0d got=%0d expected=%0d",latched_offset,words,row,col,addr,expected_addr);
        words=words+1;
    end
    task start_frame;
        begin
            @(negedge clk); words=0; latched_offset=offset; req=1;
            wait(ack); @(negedge clk); req=0;
            wait(rd_en);
        end
    endtask
    task end_frame;
        begin
            wait(finish); @(negedge clk);
            if(words!=307200) $fatal(1,"Wrong frame length %0d",words);
            repeat(4) @(negedge clk);
        end
    endtask
    initial begin
        repeat(8) @(negedge clk); rst=0;
        for(direction=0;direction<2;direction=direction+1) begin
            old_base=direction*307200; new_base=(1-direction)*307200;
            for(n=0;n<=69;n=n+1) begin
                offset=n; start_frame;
                // Change the live controls after the frame has latched them.
                wait(words>=32); @(negedge clk); offset=69-n;
                // Stall between bursts without changing pixel position.
                fifo_used=500; write_busy=1;
                repeat(400) @(negedge clk);
                fifo_used=0; write_busy=0;
                end_frame;
            end
            $display("PASS grid all 70 thresholds, 307200 addresses each, direction=%0d",direction);
        end
        offset=34; start_frame;
        wait(words>=27); @(negedge clk); cancel=1; check_grid=0;
        repeat(8) @(negedge clk); cancel=0;
        end_frame;
        for(n=0;n<4;n=n+1) begin
            read_index=n; cancel=1; start_frame; end_frame;
        end
        $display("PASS grid cancellation and all four linear video buffers");
        test_passed=1; test_done=1;
        $display("ALL GRID MAPPING CHECKS PASSED"); $finish;
    end
    initial begin #1000000000; $fatal(1,"Grid mapping timed out"); end
endmodule
