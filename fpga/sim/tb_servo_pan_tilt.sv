`timescale 1ns/1ps
// Legacy POSITION-mode regression (unchanged pitch arithmetic and mailbox).
// Continuous horizontal mode is covered by tb_servo_continuous.sv.
// Timers are shortened. Pixel/control clocks are deliberately asynchronous.
module tb_servo_pan_tilt;
    reg clk=0, pclk=0;
    always #5 clk=~clk;
    always #7 pclk=~pclk;
    reg rst=0, sample=0, valid=0, center=0;
    reg [11:0] x=640, y=360;
    wire [11:0] pan_us, tilt_us, reverse_tilt_us;
    servo_circle_track #(.CLK_FREQ_HZ(1_000_000),.CONTROL_PERIOD_US(100)) pan(
        .I_pixel_clk(pclk),.I_pixel_rst_n(rst),.I_sample(sample),
        .I_fresh_valid(valid),.I_center_x(x),.I_clk(clk),.I_rst_n(rst),
        .I_center_event(center),.O_pulse_us(pan_us));
    servo_circle_track #(.CLK_FREQ_HZ(1_000_000),.CONTROL_PERIOD_US(100),
        .IMAGE_WIDTH(720),.CENTER_X(360),.MIN_US(500),.MAX_US(2500),
        .DEADBAND_PIXELS(12),.GAIN_SHIFT(3),.MAX_STEP_US(24),.REVERSE(1)) tilt(
        .I_pixel_clk(pclk),.I_pixel_rst_n(rst),.I_sample(sample),
        .I_fresh_valid(valid),.I_center_x(y),.I_clk(clk),.I_rst_n(rst),
        .I_center_event(center),.O_pulse_us(tilt_us));
    servo_circle_track #(.CLK_FREQ_HZ(1_000_000),.CONTROL_PERIOD_US(100),
        .IMAGE_WIDTH(720),.CENTER_X(360),.MIN_US(500),.MAX_US(2500),
        .DEADBAND_PIXELS(12),.GAIN_SHIFT(3),.MAX_STEP_US(24),.REVERSE(0)) reversed(
        .I_pixel_clk(pclk),.I_pixel_rst_n(rst),.I_sample(sample),
        .I_fresh_valid(valid),.I_center_x(y),.I_clk(clk),.I_rst_n(rst),
        .I_center_event(center),.O_pulse_us(reverse_tilt_us));

    task tick;
        begin
            @(negedge clk);
            while(!pan.control_tick) @(negedge clk);
            @(posedge clk); #1;
        end
    endtask
    task packet(input integer cx, input integer cy, input bit fresh);
        begin
            @(negedge pclk);
            while(pan.sample_req!=pan.ack_sync[1] || tilt.sample_req!=tilt.ack_sync[1])
                @(negedge pclk);
            x=cx; y=cy; valid=fresh; sample=1;
            @(negedge pclk); sample=0;
            repeat(8) @(negedge clk);
        end
    endtask
    task recenter;
        begin
            @(negedge clk); center=1;
            @(negedge clk); center=0;
            if(pan_us!=1500 || tilt_us!=1500) $fatal(1,"two-axis K4 center failed");
        end
    endtask
    task reset_all;
        begin
            @(negedge clk); rst=0; sample=0; center=0;
            repeat(5) @(negedge clk); rst=1;
            repeat(5) @(negedge clk);
        end
    endtask
    integer saved_pan, saved_tilt, i, px, py, previous_pan, previous_tilt;
    initial begin
        reset_all();
        if(pan_us!=1500 || tilt_us!=1500) $fatal(1,"startup is not forward");
        packet(1000,360,1); tick();
        if(pan_us!=1508 || tilt_us!=1500) $fatal(1,"horizontal motion coupled into pitch");
        packet(640,60,1); tick();
        if(pan_us!=1508 || tilt_us!=1524 || reverse_tilt_us!=1476)
            $fatal(1,"upward pitch sign/rate/reverse wrong or pitch coupled into pan");
        packet(1000,60,1); tick();
        if(pan_us!=1516 || tilt_us!=1548) $fatal(1,"upper-right simultaneous correction failed");
        packet(280,660,1); tick();
        if(pan_us!=1508 || tilt_us!=1524) $fatal(1,"lower-left simultaneous correction failed");
        $display("PASS: unchanged pan, independent up/down pitch, pitch reversal, simultaneous XY updates");

        saved_pan=pan_us; saved_tilt=tilt_us;
        packet(628,348,1); tick(); packet(652,372,1); tick();
        if(pan_us!=saved_pan || tilt_us!=saved_tilt) $fatal(1,"XY deadband did not hold position");
        for(i=0;i<2;i=i+1) begin packet(640,344,1); tick(); end
        if(tilt_us!=saved_tilt+1 || pan_us!=saved_pan) $fatal(1,"pitch sub-us corrections lost");
        saved_tilt=tilt_us;
        packet(1000,60,1); packet(1000,60,0); tick();
        repeat(10) tick();
        if(pan_us!=saved_pan || tilt_us!=saved_tilt) $fatal(1,"target loss did not freeze both axes");
        packet(640,720,1); tick(); packet(640,4095,1); tick();
        if(tilt_us!=saved_tilt) $fatal(1,"out-of-image Y coordinate accepted");
        packet(1000,60,1); tick();
        if(pan_us!=saved_pan+8 || tilt_us!=saved_tilt+24) $fatal(1,"reacquisition integrated loss time");
        $display("PASS: independent deadbands, Q8 pitch precision, both-axis loss hold, Y bounds and reacquisition");

        for(i=0;i<100;i=i+1) begin packet(1279,0,1); tick(); end
        if(pan_us!=2000 || tilt_us!=2500) $fatal(1,"independent maximum travel limits failed");
        for(i=0;i<200;i=i+1) begin packet(0,719,1); tick(); end
        if(pan_us!=1000 || tilt_us!=500) $fatal(1,"independent minimum travel limits failed");
        packet(1279,0,1); tick();
        if(pan_us!=1008 || tilt_us!=524) $fatal(1,"two-axis saturation windup");
        packet(1000,60,1); recenter(); tick();
        if(pan_us!=1500 || tilt_us!=1500) $fatal(1,"K4 retained pending XY error");
        $display("PASS: separate travel/rate limits, no saturation windup, K4 recenters both axes");

        reset_all();
        packet(640,248,1); tick(); // (360 - 248 - 12) / 8 = +12.5 us.
        if(tilt.pulse_q8 != 1500*256+3200 || pan_us!=1500)
            $fatal(1,"increased pitch proportional gain is not 1/8 us per pixel");
        packet(640,472,1); tick(); // Symmetric -12.5 us, not an absolute mapping.
        if(tilt_us!=1500) $fatal(1,"increased pitch gain is not symmetric");
        $display("PASS: exact increased pitch gain, 24 us step cap, full 500..2500 us range");

        reset_all();
        // Illustrative uncoupled moving-camera model, NOT measured mechanics.
        // Turning right moves image X left; tilting up moves image Y down.
        for(i=0;i<300;i=i+1) begin
            px=640+250-2*($signed({1'b0,pan_us})-1500);
            py=360-180+2*($signed({1'b0,tilt_us})-1500);
            previous_pan=pan_us; previous_tilt=tilt_us;
            packet(px,py,1); tick();
            if(pan_us<previous_pan || pan_us>previous_pan+8 ||
               tilt_us<previous_tilt || tilt_us>previous_tilt+24)
                $fatal(1,"moving-camera model violates negative-feedback direction/rate");
        end
        px=640+250-2*($signed({1'b0,pan_us})-1500);
        py=360-180+2*($signed({1'b0,tilt_us})-1500);
        if(px<628 || px>654 || py<346 || py>372 || pan_us<=1550 || tilt_us<=1550)
            $fatal(1,"two-axis convergence failed: image=%d,%d pulse=%d,%d",px,py,pan_us,tilt_us);
        saved_pan=pan_us; saved_tilt=tilt_us;
        packet(640,360,1); tick(); repeat(5) tick();
        if(pan_us!=saved_pan || tilt_us!=saved_tilt) $fatal(1,"centering target reset servo angles");
        $display("PASS: both moving-camera loops converge and retain their non-neutral positions");
        $finish;
    end
    initial begin #1500000; $fatal(1,"pan/tilt test timeout"); end
endmodule
