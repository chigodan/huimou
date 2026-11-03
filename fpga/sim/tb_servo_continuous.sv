`timescale 1ns/1ps
// Real controller arithmetic; shortened 20 ms/100 ms timers at the same 1:5
// ratio. Pixel/control clocks are asynchronous. Motor plant is illustrative.
module tb_servo_continuous;
    reg clk=0, pclk=0;
    always #5 clk=~clk;
    always #7 pclk=~pclk;
    reg rst=0, sample=0, valid=0, stop=0;
    reg [11:0] x=640, y=360;
    wire [11:0] pan_us, reverse_us, trimmed_us, pitch_us;
    servo_circle_track #(.CLK_FREQ_HZ(1_000_000),.CONTROL_PERIOD_US(100),
        .MIN_US(1300),.MAX_US(1700),.GAIN_SHIFT(1),.MAX_STEP_US(20),
        .SPEED_CONTROL(1),.MIN_DRIVE_US(40),.SAMPLE_TIMEOUT_US(500)) pan(
        .I_pixel_clk(pclk),.I_pixel_rst_n(rst),.I_sample(sample),
        .I_fresh_valid(valid),.I_center_x(x),.I_clk(clk),.I_rst_n(rst),
        .I_center_event(stop),.O_pulse_us(pan_us));
    servo_circle_track #(.CLK_FREQ_HZ(1_000_000),.CONTROL_PERIOD_US(100),
        .MIN_US(1300),.MAX_US(1700),.GAIN_SHIFT(1),.MAX_STEP_US(20),
        .SPEED_CONTROL(1),.MIN_DRIVE_US(40),.SAMPLE_TIMEOUT_US(500),.REVERSE(1)) reversed(
        .I_pixel_clk(pclk),.I_pixel_rst_n(rst),.I_sample(sample),
        .I_fresh_valid(valid),.I_center_x(x),.I_clk(clk),.I_rst_n(rst),
        .I_center_event(stop),.O_pulse_us(reverse_us));
    servo_circle_track #(.CLK_FREQ_HZ(1_000_000),.CONTROL_PERIOD_US(100),
        .CENTER_US(1520),.MIN_US(1320),.MAX_US(1720),.GAIN_SHIFT(1),.MAX_STEP_US(20),
        .SPEED_CONTROL(1),.MIN_DRIVE_US(40),.SAMPLE_TIMEOUT_US(500)) trimmed(
        .I_pixel_clk(pclk),.I_pixel_rst_n(rst),.I_sample(sample),
        .I_fresh_valid(valid),.I_center_x(x),.I_clk(clk),.I_rst_n(rst),
        .I_center_event(stop),.O_pulse_us(trimmed_us));
    servo_circle_track #(.CLK_FREQ_HZ(1_000_000),.CONTROL_PERIOD_US(100),
        .IMAGE_WIDTH(720),.CENTER_X(360),.MIN_US(500),.MAX_US(2500),
        .GAIN_SHIFT(3),.MAX_STEP_US(24),.REVERSE(1)) pitch(
        .I_pixel_clk(pclk),.I_pixel_rst_n(rst),.I_sample(sample),
        .I_fresh_valid(valid),.I_center_x(y),.I_clk(clk),.I_rst_n(rst),
        .I_center_event(stop),.O_pulse_us(pitch_us));

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
            while(pan.sample_req!=pan.ack_sync[1] || pitch.sample_req!=pitch.ack_sync[1])
                @(negedge pclk);
            x=cx; y=cy; valid=fresh; sample=1;
            @(negedge pclk); sample=0;
            repeat(8) @(negedge clk);
        end
    endtask
    task reset_all;
        begin
            @(negedge clk); rst=0; sample=0; stop=0;
            repeat(5) @(negedge clk); rst=1;
            repeat(5) @(negedge clk);
        end
    endtask
    task check_stop;
        begin
            if(pan_us!=1500 || reverse_us!=1500 || trimmed_us!=1520)
                $fatal(1,"neutral stop failed: %d %d %d",pan_us,reverse_us,trimmed_us);
        end
    endtask

    integer i, previous, held_pitch, plant_x_q8;
    initial begin
        reset_all(); check_stop();
        packet(1000,360,1); tick();
        if(pan_us!=1520 || reverse_us!=1480 || trimmed_us!=1540 || pitch_us!=1500)
            $fatal(1,"direction, acceleration, neutral trim or axis independence failed");
        tick(); tick();
        if(pan_us!=1560 || pitch_us!=1500) $fatal(1,"speed must persist between fresh camera frames");
        for(i=0;i<12;i=i+1) begin packet(1000,360,1); tick(); end
        if(pan_us!=1674 || reverse_us!=1326 || trimmed_us!=1694)
            $fatal(1,"constant image error must settle at proportional speed, not integrate");
        for(i=0;i<12;i=i+1) begin packet(740,360,1); tick(); end
        if(pan_us!=1544 || reverse_us!=1456) $fatal(1,"smaller error must reduce speed");
        packet(653,360,1); tick();
        if(pan_us!=1540 || reverse_us!=1460) $fatal(1,"minimum drive compensation failed");
        packet(652,360,1); check_stop();
        packet(628,360,1); tick(); check_stop();
        $display("PASS: proportional speed, gain, acceleration, min drive, reversal setting, deadband and trim");

        for(i=0;i<12;i=i+1) begin packet(1279,60,1); tick(); end
        if(pan_us!=1700 || reverse_us!=1300 || trimmed_us!=1720 || pitch_us<=1500)
            $fatal(1,"speed caps or independent pitch motion failed");
        packet(0,360,1); tick(); check_stop(); // Stop before reversing.
        tick();
        if(pan_us!=1480 || reverse_us!=1520) $fatal(1,"reversal did not ramp from neutral");
        for(i=0;i<12;i=i+1) begin packet(0,360,1); tick(); end
        if(pan_us!=1300 || reverse_us!=1700 || trimmed_us!=1320)
            $fatal(1,"negative speed cap failed");
        held_pitch=pitch_us;
        packet(0,60,0); check_stop(); // Detector may retain a box but fresh_valid=0.
        tick();
        if(pitch_us!=held_pitch) $fatal(1,"position pitch must hold on invalid frame");
        packet(1279,360,1); tick();
        if(pan_us!=1520) $fatal(1,"reacquisition must ramp from stop");
        packet(1280,360,1); check_stop();
        packet(4095,360,1); tick(); check_stop();
        $display("PASS: both speed caps, stop before reversal, invalid/retained-box stop, bounds and reacquisition");

        packet(1279,360,1); tick();
        while(pan.sample_age<499) @(negedge clk);
        @(posedge clk); #1; check_stop();
        repeat(4) tick(); check_stop();
        if(pan.fresh_pending) $fatal(1,"expired target was retained as usable");
        packet(1279,60,1); tick();
        if(pan_us!=1520) $fatal(1,"watchdog recovery did not ramp from stop");
        @(negedge clk); stop=1;
        @(negedge clk); stop=0; check_stop();
        if(pitch_us!=1500) $fatal(1,"K4 must still center pitch");
        repeat(4) tick(); check_stop();
        $display("PASS: exact no-frame timeout, no stale restart, K4 stops pan and homes pitch");

        reset_all();
        plant_x_q8=1000*256;
        for(i=0;i<200;i=i+1) begin
            previous=pan_us;
            packet(plant_x_q8/256,360,1); tick();
            if(pan_us<1500 || pan_us>1700 || pan_us>previous+20)
                $fatal(1,"closed-loop speed/rate bound failed");
            plant_x_q8=plant_x_q8-($signed({1'b0,pan_us})-1500)*16;
        end
        if(plant_x_q8/256<628 || plant_x_q8/256>652 || pan_us!=1500 || pitch_us!=1500)
            $fatal(1,"continuous-pan visual loop did not converge and stop: x=%d pulse=%d",plant_x_q8/256,pan_us);
        $display("PASS: illustrative moving-camera speed feedback converges to center and stops");
        $finish;
    end
    initial begin #1000000; $fatal(1,"continuous servo test timeout"); end
endmodule
