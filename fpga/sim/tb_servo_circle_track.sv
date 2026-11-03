`timescale 1ns/1ps
// Fast control-period simulation with asynchronous pixel/control clocks.
// Timer parameters are shortened; gains, Q8 arithmetic and safety limits are real.
module tb_servo_circle_track;
    reg clk=0, pclk=0;
    always #5 clk=~clk;
    always #7 pclk=~pclk;
    reg rst=0, sample=0, valid=0, center=0;
    reg [11:0] x=640;
    wire [11:0] pulse, reverse_pulse;
    servo_circle_track #(.CLK_FREQ_HZ(1_000_000), .CONTROL_PERIOD_US(100)) dut(
        .I_pixel_clk(pclk), .I_pixel_rst_n(rst),
        .I_sample(sample), .I_fresh_valid(valid), .I_center_x(x),
        .I_clk(clk), .I_rst_n(rst), .I_center_event(center), .O_pulse_us(pulse));
    servo_circle_track #(.CLK_FREQ_HZ(1_000_000), .CONTROL_PERIOD_US(100),
        .REVERSE(1)) reversed(
        .I_pixel_clk(pclk), .I_pixel_rst_n(rst),
        .I_sample(sample), .I_fresh_valid(valid), .I_center_x(x),
        .I_clk(clk), .I_rst_n(rst), .I_center_event(center),
        .O_pulse_us(reverse_pulse));

    task tick;
        begin
            @(negedge clk);
            while(!dut.control_tick) @(negedge clk);
            @(posedge clk); #1;
        end
    endtask
    task packet(input integer xpos, input bit fresh);
        begin
            @(negedge pclk);
            while(dut.sample_req != dut.ack_sync[1]) @(negedge pclk);
            x=xpos; valid=fresh; sample=1;
            @(negedge pclk); sample=0;
            // Allow destination capture; ensure no accidental test-side backlog.
            repeat(8) @(negedge clk);
        end
    endtask
    task recenter;
        begin
            @(negedge clk); center=1;
            @(negedge clk); center=0;
            if(pulse!=1500) $fatal(1,"K4 center failed");
        end
    endtask
    task reset_all;
        begin
            @(negedge clk); rst=0; sample=0; center=0;
            repeat(5) @(negedge clk); rst=1;
            repeat(5) @(negedge clk);
        end
    endtask
    integer saved, i, previous, plant_x;
    initial begin
        reset_all();
        if(pulse!=1500) $fatal(1,"startup is not straight ahead");
        packet(1000,1); tick();
        if(pulse!=1508 || reverse_pulse!=1492)
            $fatal(1,"direction/step limit failed: %d %d",pulse,reverse_pulse);
        repeat(5) tick();
        if(pulse!=1508) $fatal(1,"same observation reused during target loss");
        $display("PASS: 1500 us startup, direction reversal, 8 us limit, no stale-error integration");

        packet(640,1); tick();
        packet(628,1); tick();
        packet(652,1); tick();
        if(pulse!=1508) $fatal(1,"center/deadband recentered a non-neutral servo");
        for(i=0;i<4;i=i+1) begin packet(660,1); tick(); end
        if(pulse!=1509) $fatal(1,"fractional corrections lost: %d",pulse);
        packet(280,1); tick();
        if(pulse!=1501) $fatal(1,"left correction is not symmetric");
        $display("PASS: current-position hold in deadband, Q8 small-error accumulation, left correction");

        // Invalid frame replaces an unconsumed valid one, even if box is retained.
        packet(1000,1); packet(1000,0); tick();
        if(pulse!=1501) $fatal(1,"invalid packet did not cancel pending error");
        packet(4095,1); tick();
        if(pulse!=1501) $fatal(1,"out-of-image coordinates accepted");
        repeat(20) tick();
        packet(1000,1); tick();
        if(pulse!=1509) $fatal(1,"reacquisition jumped or integrated loss time");
        $display("PASS: missing/invalid frame freeze, coordinate bounds, gentle reacquisition");

        for(i=0;i<100;i=i+1) begin packet(1279,1); tick(); end
        if(pulse!=2000) $fatal(1,"upper safety clamp failed: %d",pulse);
        for(i=0;i<160;i=i+1) begin packet(0,1); tick(); end
        if(pulse!=1000) $fatal(1,"lower safety clamp failed: %d",pulse);
        packet(1279,1); tick();
        if(pulse!=1008) $fatal(1,"saturation windup delayed reversal");
        packet(1000,1); recenter(); tick();
        if(pulse!=1500) $fatal(1,"K4 did not discard pending error");
        repeat(4) tick();
        if(pulse!=1500) $fatal(1,"motion continued after K4 without fresh data");
        $display("PASS: both travel clamps, immediate departure from saturation, K4 fresh-data reset");

        reset_all();
        // Illustrative static-target plant: turning the camera right moves the
        // target left at 2 pixels/us. This is NOT a measured mechanical model.
        for(i=0;i<180;i=i+1) begin
            plant_x=640+300-2*($signed({1'b0,pulse})-1500);
            previous=pulse;
            packet(plant_x,1); tick();
            if(pulse<previous || pulse>previous+8)
                $fatal(1,"plant run violates direction or rate bound");
        end
        plant_x=640+300-2*($signed({1'b0,pulse})-1500);
        if(plant_x<628 || plant_x>654 || pulse<=1600)
            $fatal(1,"closed loop did not converge: x=%d pulse=%d",plant_x,pulse);
        saved=pulse;
        packet(640,1); tick(); repeat(5) tick();
        if(pulse!=saved) $fatal(1,"converged command returned to neutral");
        $display("PASS: moving-camera negative feedback converges and holds non-1500 equilibrium");
        $finish;
    end
    initial begin #1000000; $fatal(1,"control test timeout"); end
endmodule
