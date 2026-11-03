`timescale 1ns/1ps
// Actual image_transition_ctrl from top_tf_hdmi_audio.v, default trans_ctrl.
// No forced DUT state or copied production implementation.
module tb_grid_transition_ctrl;
    reg clk=0, rst=1, inhibit=0, image_valid=0, frame_done=0;
    reg [1:0] image_index=0;
    reg transition_req_toggle=0;
    wire transition_abort, trans_en, transition_busy, transition_done_toggle;
    wire [1:0] trans_effect, old_index, new_index;
    wire [9:0] trans_offset;
    wire [8:0] fade_factor;
    reg test_done=0, test_passed=0;
    integer errors=0, checks=0, frames=0, full_effects=0, cancellations=0;
    integer rotation, target, route, n, fx;
    reg ack_before;
    reg [1:0] previous_index;

    always #5 clk=~clk;
    image_transition_ctrl dut (
        .clk(clk), .rst(rst), .inhibit(inhibit), .image_valid(image_valid),
        .image_index(image_index), .frame_done(frame_done),
        .transition_abort(transition_abort), .trans_en(trans_en),
        .trans_effect(trans_effect), .trans_offset(trans_offset), .fade_factor(fade_factor),
        .old_index(old_index), .new_index(new_index),
        .transition_req_toggle(transition_req_toggle), .transition_busy(transition_busy),
        .transition_done_toggle(transition_done_toggle)
    );
    task check;
        input condition;
        input [8*140-1:0] message;
        begin
            checks=checks+1;
            if (condition !== 1'b1) begin
                errors=errors+1;
                $display("FAIL image_ctrl t=%0t %0s fx=%0d en=%b off=%0d fade=%0d busy=%b req/done=%b/%b old/new=%0d/%0d",
                         $time,message,trans_effect,trans_en,trans_offset,fade_factor,
                         transition_busy,transition_req_toggle,transition_done_toggle,old_index,new_index);
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
    task pending;
        begin
            check(transition_busy === 1'b1,"request remains busy; old buffer still owned");
            check(transition_done_toggle === ack_before,"done must not toggle before release frame");
        end
    endtask
    task hold_frame;
        reg [25:0] held;
        reg held_done, held_busy;
        integer k;
        begin
            held={trans_effect,trans_en,trans_offset,fade_factor,old_index,new_index};
            held_done=transition_done_toggle; held_busy=transition_busy;
            for (k=0; k < 5+frames%7; k=k+1) begin
                @(negedge clk);
                check({trans_effect,trans_en,trans_offset,fade_factor,old_index,new_index} === held,
                      "effect and buffer mapping stable inside frame");
                check(transition_done_toggle === held_done && transition_busy === held_busy,
                      "ownership stable without frame_done");
            end
        end
    endtask
    task one_frame;
        begin
            hold_frame;
            frame_done=1;
            @(negedge clk); frame_done=0;
            frames=frames+1;
        end
    endtask
    task release_next_frame;
        begin
            pending;
            // Includes the entire first fully-new frame after effect completion.
            hold_frame; pending; one_frame;
            check(transition_busy === 1'b0,"next completed frame releases old buffer");
            check(transition_done_toggle === transition_req_toggle,"done acknowledges exact request toggle");
            one_frame;
            check(transition_busy === 1'b0 && transition_done_toggle === transition_req_toggle,
                  "no repeated/spurious acknowledgement");
        end
    endtask
    task request_image;
        input [1:0] index;
        input integer effect; // -1: session's first image, without an effect.
        begin
            previous_index=new_index; ack_before=transition_done_toggle;
            @(negedge clk);
            image_index=index; image_valid=1;
            transition_req_toggle=~transition_req_toggle;
            // Descriptor/request CDC and startup cannot finish without a frame.
            repeat (12) begin
                @(negedge clk);
                check(transition_done_toggle === ack_before,"CDC/startup cannot acknowledge without a frame");
            end
            pending;
            check(new_index === index,"new descriptor adopted");
            if (effect < 0)
                check(old_index === index && trans_en === 1'b0 && fade_factor === 9'd256,
                      "first image primes both mappings without an effect");
            else begin
                check(old_index === previous_index,"old descriptor retained until release");
                expect_state(effect,1'b1,10'd0,9'd256);
            end
        end
    endtask
    task reset_and_prime;
        begin
            @(negedge clk);
            rst=1; inhibit=0; image_valid=0; image_index=0;
            frame_done=0; transition_req_toggle=0;
            repeat (5) @(negedge clk);
            expect_state(2'd0,1'b0,10'd0,9'd256);
            check(transition_busy === 1'b0 && transition_done_toggle === 1'b0,
                  "reset clears request ownership");
            rst=0;
            repeat (8) @(negedge clk);
            request_image(2'd0,-1);
            check(trans_effect === 2'd0,"first image preserves reset SLIDE selection");
            release_next_frame;
        end
    endtask
    task complete_effect;
        input integer effect;
        integer f, expected_offset, start_errors;
        begin
            start_errors=errors;
            expect_state(effect,1'b1,10'd0,9'd256);
            if (effect == 1 || effect == 0) begin
                for (f=1; f <= (effect == 1 ? 35 : 80); f=f+1) begin
                    one_frame;
                    expected_offset=effect == 1 ? (f*2 > 69 ? 69 : f*2) : f*8;
                    expect_state(effect,1'b1,expected_offset,9'd256); pending;
                end
                // GRID 68 is not all-new; 69 stays enabled for a full frame.
                hold_frame; pending; one_frame;
                expect_state(effect,1'b0,effect == 1 ? 10'd69 : 10'd640,9'd256);
                pending; // SAME frame edge dropping trans_en cannot release.
            end else begin
                for (f=1; f <= 16; f=f+1) begin
                    one_frame;
                    expect_state(2'd2,f < 16,10'd0,256-f*16);
                    pending; // Falling trans_en at black is not completion.
                end
                for (f=1; f <= 16; f=f+1) begin
                    one_frame; expect_state(2'd2,1'b0,10'd0,f*16);
                    pending; // Also covers SAME edge fade first reaches 256.
                end
            end
            release_next_frame;
            check(old_index === previous_index && new_index === image_index,
                  "completion preserves buffer descriptors");
            full_effects=full_effects+1;
            $display("COVER image_ctrl complete fx=%0d req=%b errors_added=%0d",
                     effect,transition_req_toggle,errors-start_errors);
        end
    endtask
    task cancel_and_return;
        input integer mode;
        integer b, start_errors;
        reg [1:0] retained_effect;
        begin
            retained_effect=trans_effect; start_errors=errors;
            @(negedge clk);
            // Independent abort paths: inhibit/video, invalid descriptor,
            // and valid index 2, which belongs to video rather than images.
            if (mode == 0) inhibit=1;
            else if (mode == 1) image_valid=0;
            else image_index=2;
            repeat (10) @(negedge clk);
            check(transition_abort === 1'b1,"mode entry cancels active effect");
            expect_state(retained_effect,1'b0,10'd640,9'd256);
            pending; // Abort alone must not release an outstanding memory read.
            release_next_frame;
            // Video ownership remains asserted as buffers cycle 0..3 twice.
            inhibit=1;
            repeat (6) @(negedge clk);
            for (b=0; b < 8; b=b+1) begin
                @(negedge clk); image_index=b%4;
                repeat (8) @(negedge clk);
                one_frame;
                check(transition_abort === 1'b1,"video inhibited for every buffer 0..3");
                expect_state(retained_effect,1'b0,10'd640,9'd256);
                check(transition_busy === 1'b0 && transition_done_toggle === transition_req_toggle,
                      "video buffer changes cannot create a request or acknowledgement");
            end
            // A changed descriptor on return is first-image priming, not a
            // transition from the stale descriptor before video was entered.
            @(negedge clk); inhibit=0; image_valid=1;
            request_image(2'd1,-1);
            check(transition_abort === 1'b0 && trans_effect === retained_effect,
                  "return first image does not rotate/start an effect");
            release_next_frame;
            request_image(2'd0,(retained_effect+1)%3);
            complete_effect((retained_effect+1)%3);
            cancellations=cancellations+1;
            $display("COVER image_ctrl cancel/return fx=%0d route=%0d errors_added=%0d",
                     retained_effect,mode,errors-start_errors);
        end
    endtask

    // Edge monitor also catches transient changes between task samples.
    reg pre_rst, pre_frame, pre_abort, pre_switch, pre_done, pre_busy;
    reg [21:0] pre_parameters;
    always @(posedge clk) begin
        pre_rst=rst; pre_frame=frame_done; pre_abort=transition_abort;
        pre_switch=dut.switch_event; pre_done=transition_done_toggle;
        pre_busy=transition_busy;
        pre_parameters={trans_effect,trans_en,trans_offset,fade_factor};
        #1;
        if (!pre_rst && !test_done) begin
            if (!pre_frame && !pre_abort && !transition_abort && !pre_switch)
                check({trans_effect,trans_en,trans_offset,fade_factor} === pre_parameters,
                      "edge monitor: effect parameters change only on frame/start/abort");
            if (transition_done_toggle !== pre_done) begin
                check(pre_frame && pre_busy,"edge monitor: ack requires busy completed read frame");
                check(pre_abort || (!pre_parameters[19] && pre_parameters[8:0] == 9'd256),
                      "edge monitor: ack requires effect idle BEFORE this frame edge");
                check(transition_done_toggle === transition_req_toggle,
                      "edge monitor: acknowledgement matches request");
            end
        end
    end
    initial begin
        reset_and_prime;
        // Both request polarities and two complete rotations.
        for (rotation=0; rotation < 6; rotation=rotation+1) begin
            fx=(rotation+1)%3;
            request_image(new_index ^ 2'd1,fx); complete_effect(fx);
        end
        for (route=0; route < 3; route=route+1) begin
            for (target=0; target < 3; target=target+1) begin
                reset_and_prime;
                for (n=0; n <= target; n=n+1) begin
                    fx=(n+1)%3;
                    request_image(new_index ^ 2'd1,fx);
                    if (n < target) complete_effect(fx);
                end
                one_frame; one_frame; pending;
                cancel_and_return(route);
            end
        end
        // Also cancel FADE after trans_en falls, while fade-in is still active.
        reset_and_prime;
        request_image(2'd1,1); complete_effect(1);
        request_image(2'd0,2);
        repeat (19) one_frame;
        expect_state(2'd2,1'b0,10'd0,9'd48); pending;
        cancel_and_return(0);
        check(full_effects == 26 && cancellations == 10,"all scheduled complete/cancel scenarios executed");
        test_passed=(errors == 0); test_done=1;
        $display("GRID_TRANSITION_RESULT passed=%b checks=%0d errors=%0d complete=%0d cancels=%0d frames=%0d",
                 test_passed,checks,errors,full_effects,cancellations,frames);
        $finish;
    end
    initial begin
        #5000000;
        errors=errors+1; test_passed=0; test_done=1;
        $display("FAIL image_ctrl TIMEOUT"); $finish;
    end
endmodule
