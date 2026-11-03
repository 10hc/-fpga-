`timescale 1ns/1ps
// Real default-step controller; no accelerated parameters or forced DUT state.
module tb_trans_ctrl;
    reg clk=0, rst=1, switch_pulse=0, frame_done=0, abort=0;
    wire [1:0] trans_effect;
    wire trans_en;
    wire [9:0] trans_offset;
    wire [8:0] fade_factor;
    reg test_done=0, test_passed=0;
    integer errors=0, checks=0, i, fx;
    reg [21:0] held;
    always #5 clk=~clk;
    trans_ctrl dut (
        .clk(clk), .rst(rst), .switch_pulse(switch_pulse),
        .frame_done(frame_done), .abort(abort), .trans_effect(trans_effect),
        .trans_en(trans_en), .trans_offset(trans_offset), .fade_factor(fade_factor)
    );
    task check;
        input condition;
        input [8*120-1:0] message;
        begin
            checks=checks+1;
            if (condition !== 1'b1) begin
                errors=errors+1;
                $display("FAIL trans_ctrl t=%0t %0s fx=%0d en=%b off=%0d fade=%0d",
                         $time,message,trans_effect,trans_en,trans_offset,fade_factor);
            end
        end
    endtask
    task expect_state;
        input [1:0] effect;
        input enabled;
        input [9:0] offset;
        input [8:0] fade;
        begin
            check({trans_effect,trans_en,trans_offset,fade_factor} ===
                  {effect,enabled,offset,fade},"exact effect/en/offset/fade");
        end
    endtask
    task hold_frame;
        begin
            held={trans_effect,trans_en,trans_offset,fade_factor};
            repeat (7) begin
                @(negedge clk);
                check({trans_effect,trans_en,trans_offset,fade_factor} === held,
                      "parameters stable inside frame");
            end
        end
    endtask
    task one_frame;
        begin
            hold_frame;
            frame_done=1;
            @(negedge clk); frame_done=0;
        end
    endtask
    task do_switch;
        begin
            @(negedge clk); switch_pulse=1;
            @(negedge clk); switch_pulse=0;
        end
    endtask
    task complete_effect;
        input integer effect;
        integer n, expected_offset;
        begin
            expect_state(effect,1'b1,10'd0,9'd256);
            if (effect == 1 || effect == 0) begin
                for (n=1; n <= (effect == 1 ? 35 : 80); n=n+1) begin
                    one_frame;
                    expected_offset=effect == 1 ? (n*2 > 69 ? 69 : n*2) : n*8;
                    expect_state(effect,1'b1,expected_offset,9'd256);
                end
                // GRID displays progress 69 for a full frame before disabling.
                hold_frame; one_frame;
                expect_state(effect,1'b0,effect == 1 ? 10'd69 : 10'd640,9'd256);
            end else begin
                for (n=1; n <= 16; n=n+1) begin
                    one_frame; expect_state(2'd2,n < 16,10'd0,256-n*16);
                end
                for (n=1; n <= 16; n=n+1) begin
                    one_frame; expect_state(2'd2,1'b0,10'd0,n*16);
                end
            end
            $display("COVER trans_ctrl complete fx=%0d",effect);
        end
    endtask
    initial begin
        repeat (5) @(negedge clk);
        expect_state(2'd0,1'b0,10'd0,9'd256);
        rst=0; hold_frame;
        // Two full rotations include GRID first and wraparound.
        for (i=0; i < 6; i=i+1) begin
            fx=(i+1)%3; do_switch; complete_effect(fx);
        end
        for (i=0; i < 3; i=i+1) begin
            fx=(i+1)%3; do_switch; one_frame;
            expect_state(fx,1'b1,fx == 1 ? 10'd2 : (fx == 0 ? 10'd8 : 10'd0),
                         fx == 2 ? 9'd240 : 9'd256);
            abort=1; @(negedge clk);
            expect_state(fx,1'b0,10'd640,9'd256);
            one_frame;
            expect_state(fx,1'b0,10'd640,9'd256);
            abort=0; hold_frame;
            $display("COVER trans_ctrl abort fx=%0d",fx);
        end
        test_passed=(errors == 0); test_done=1;
        $display("TRANS_CTRL_RESULT passed=%b checks=%0d errors=%0d",test_passed,checks,errors);
        $finish;
    end
    initial begin
        #2000000;
        errors=errors+1; test_passed=0; test_done=1;
        $display("FAIL trans_ctrl TIMEOUT"); $finish;
    end
endmodule
