`timescale 1ns/1ps
module tb_grid_sdram;
    reg clk=0, rst=1;
    always #4 clk=~clk;
    wire clk_sft;
    assign #2 clk_sft=clk;
    wire app_en;
    wire [20:0] app_addr;
    wire ready, busy, ref_vld, data_valid;
    reg req=0;
    wire ack, finish;
    reg [9:0] threshold=0;
    reg [20:0] old_base=0, new_base=307200;
    frame_fifo_read #(.BURST_SIZE(256)) reader (
        .rst(rst),.mem_clk(clk),.Sdr_init_done(ready),
        .Sdr_init_ref_vld(ref_vld),.Sdr_busy(busy),.Sdr_rd_en(data_valid),
        .App_wr_busy(1'b0),.O_rd_busy(),.App_rd_en(app_en),.App_rd_addr(app_addr),
        .read_req(req),.read_req_ack(ack),.read_finish(finish),
        .read_addr_0(21'd0),.read_addr_1(21'd307200),
        .read_addr_2(21'd614400),.read_addr_3(21'd921600),
        .read_addr_index(2'd1),.read_len(21'd307200),.fifo_aclr(),.wrusedw(10'd0),
        .trans_abort(1'b0),.trans_en(1'b1),.trans_effect(2'd1),
        .trans_offset(threshold),.trans_old_base(old_base),.trans_new_base(new_base),
        .dbg_state(),.dbg_wrbusy(),.dbg_req2(),.dbg_init()
    );
    wire ram_clk, ras_n, cas_n, we_n;
    wire [1:0] bank;
    wire [10:0] addr;
    wire [3:0] dm;
    wire [31:0] dq, data;

    reg test_done=0, test_passed=0;
    sdr_as_ram #(.self_refresh_open(1)) dut (
        .Sdr_clk(clk),.Sdr_clk_sft(clk_sft),.Rst(rst),
        .Sdr_init_done(ready),.Sdr_init_ref_vld(ref_vld),.Sdr_busy(busy),
        .App_ref_req(1'b0),.App_wr_en(1'b0),.App_wr_addr(21'd0),
        .App_wr_dm(4'd0),.App_wr_din(32'd0),
        .App_rd_en(app_en),.App_rd_addr(app_addr),
        .Sdr_rd_en(data_valid),.Sdr_rd_dout(data),
        .SDRAM_CLK(ram_clk),.SDR_RAS(ras_n),.SDR_CAS(cas_n),
        .SDR_WE(we_n),.SDR_BA(bank),.SDR_ADDR(addr),.SDR_DM(dm),.SDR_DQ(dq)
    );
    reg [20:0] requested[0:307199];
    reg [10:0] active_row[0:3];
    reg [3:0] bank_open=0;
    integer issued=0, commands=0, actual_addr, errors=0, expected_addr, k;
    integer returned=0, cycle=0, last_request=0, last_return=0, max_pending=0;
    always @(posedge clk) begin
        cycle=cycle+1;
        if(app_en) last_request=cycle;
        if(data_valid) begin returned=returned+1; last_return=cycle; end
        if(issued-returned>max_pending) max_pending=issued-returned;
    end
    always @(posedge clk) if(!rst && app_en) begin
        expected_addr=(((issued/640)/16+(issued%640)/16)<threshold ? new_base:old_base)+issued;
        if(app_addr!==expected_addr[20:0]) $fatal(1,"Reader source address mismatch");
        requested[issued]=app_addr; issued=issued+1;
    end
    always @(posedge ram_clk) if(!rst) begin
        case({ras_n,cas_n,we_n})
            3'b000: $display("MRS %h",addr);
            3'b011: begin active_row[bank]=addr; bank_open[bank]=1; end
            3'b010: if(addr[10]) bank_open=0; else bank_open[bank]=0;
            3'b101: begin
                if(!bank_open[bank]) $fatal(1,"READ against closed SDRAM bank");
                // Vendor controller stripes groups of four words across banks;
                // MRS 0x032 selects burst length four and CAS latency three.
                actual_addr={active_row[bank],addr[7:2],bank,addr[1:0]};
                if(commands*4>=issued) $fatal(1,"Unrequested READ command");
                for(k=0;k<4;k=k+1) begin
                    if(actual_addr+k!==requested[commands*4+k])
                        $fatal(1,"Physical SDRAM READ burst mismatch word=%0d actual=%0d expected=%0d",commands*4+k,actual_addr+k,requested[commands*4+k]);
                end
                commands=commands+1;
            end
        endcase
    end
    integer direction, n;
    initial begin
        repeat(10) @(negedge clk); rst=0;
        wait(ready && !busy); repeat(30) @(negedge clk);
        for(direction=0;direction<2;direction=direction+1) begin
            old_base=direction*307200; new_base=(1-direction)*307200;
            for(n=0;n<5;n=n+1) begin
                case(n) 0:threshold=0; 1:threshold=1; 2:threshold=34;
                    3:threshold=68; 4:threshold=69; endcase
                issued=0; commands=0; returned=0; max_pending=0;
                @(negedge clk); req=1;
                wait(ack); @(negedge clk); req=0;
                wait(finish); repeat(50) @(negedge clk);
                if(issued!=307200 || issued!=commands*4 || issued!=returned)
                    $fatal(1,"SDRAM lost frame words: issued=%0d commands=%0d returned=%0d",issued,commands,returned);
                $display("PASS reader -> vendor SDRAM direction=%0d threshold=%0d words=%0d READ commands=%0d max_pending=%0d tail=%0d",direction,threshold,returned,commands,max_pending,last_return-last_request);
            end
        end
        test_passed=1; test_done=1;
        $display("ALL GRID SDRAM CHECKS PASSED"); $finish;
    end
    initial begin #50000000; $fatal(1,"Grid SDRAM test timed out"); end
endmodule
