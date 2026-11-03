`timescale 1ns/1ps
module tb_servo_continuous_edge_start;
    reg clk=0,pclk=0,rst=0,sample=0,valid=0,home=0;
    always #5 clk=~clk;
    always #7 pclk=~pclk;
    reg [11:0] x=640;
    wire [11:0] pan_us,reverse_us,limited_us;
    servo_circle_track #(.CLK_FREQ_HZ(1_000_000),.CONTROL_PERIOD_US(100),
        .CENTER_US(1520),.MIN_US(1320),.MAX_US(1720),.SAMPLE_TIMEOUT_US(500),
        .GAIN_SHIFT(2),.MAX_STEP_US(8),.SPEED_CONTROL(1),.STABILITY_ENABLE(1),
        .START_BOOST_US(60),.START_BOOST_ERROR_PIXELS(160)) pan(
        .I_clk(clk),.I_rst_n(rst),.I_pixel_clk(pclk),.I_pixel_rst_n(rst),
        .I_sample(sample),.I_fresh_valid(valid),.I_center_x(x),
        .I_center_event(home),.O_pulse_us(pan_us));
    servo_circle_track #(.CLK_FREQ_HZ(1_000_000),.CONTROL_PERIOD_US(100),
        .CENTER_US(1520),.MIN_US(1320),.MAX_US(1720),.SAMPLE_TIMEOUT_US(500),
        .GAIN_SHIFT(2),.MAX_STEP_US(8),.SPEED_CONTROL(1),.STABILITY_ENABLE(1),
        .START_BOOST_US(60),.START_BOOST_ERROR_PIXELS(160),.REVERSE(1)) reversed(
        .I_clk(clk),.I_rst_n(rst),.I_pixel_clk(pclk),.I_pixel_rst_n(rst),
        .I_sample(sample),.I_fresh_valid(valid),.I_center_x(x),
        .I_center_event(home),.O_pulse_us(reverse_us));
    servo_circle_track #(.CLK_FREQ_HZ(1_000_000),.CONTROL_PERIOD_US(100),
        .CENTER_US(1520),.MIN_US(1510),.MAX_US(1530),.SAMPLE_TIMEOUT_US(500),
        .GAIN_SHIFT(2),.MAX_STEP_US(8),.SPEED_CONTROL(1),.STABILITY_ENABLE(1),
        .START_BOOST_US(60),.START_BOOST_ERROR_PIXELS(160)) limited(
        .I_clk(clk),.I_rst_n(rst),.I_pixel_clk(pclk),.I_pixel_rst_n(rst),
        .I_sample(sample),.I_fresh_valid(valid),.I_center_x(x),
        .I_center_event(home),.O_pulse_us(limited_us));
    task tick;
        begin
            @(negedge clk);while(!pan.control_tick) @(negedge clk);
            @(posedge clk);#1;
        end
    endtask
    task packet(input integer position,input bit fresh);
        begin
            @(negedge pclk);while(pan.sample_req!=pan.ack_sync[1]) @(negedge pclk);
            x=position;valid=fresh;sample=1;
            @(negedge pclk);sample=0;repeat(10) @(negedge clk);
        end
    endtask
    task stopped;
        begin
            if(pan_us!==1520 || reverse_us!==1520 || limited_us!==1520)
                $fatal(1,"STOP/neutral calibration mismatch");
        end
    endtask
    task clear_stop;
        begin
            @(negedge clk);home=1;@(negedge clk);home=0;stopped();
        end
    endtask
    integer i,previous;
    initial begin
        repeat(5) @(negedge clk);rst=1;repeat(8) @(negedge clk);
        packet(1279,1);tick();
        if(pan_us!=1520) $fatal(1,"one observation must not bypass start confirmation");
        packet(0,0);tick();
        packet(1279,1);tick();
        if(pan_us!=1580 || reverse_us!=1460 || limited_us!=1530)
            $fatal(1,"right edge failed confirmed 60us start/trim/reverse/clamp: %d,%d,%d",pan_us,reverse_us,limited_us);
        // A simple motor dead-zone fixture: the old first 8us command would
        // stay stopped here; the confirmed 60us command crosses +/-40us.
        if(pan_us-1520<40 || 1520-reverse_us<40) $fatal(1,"start remains inside motor dead zone");
        previous=pan_us;packet(1279,1);tick();
        if(pan_us!=previous+8) $fatal(1,"post-start acceleration no longer limited to 8us");
        packet(0,0);tick();
        if(pan_us<=1520) $fatal(1,"single missed moving frame stopped pan");
        packet(0,0);tick();stopped();
        packet(0,1);tick();stopped();packet(4095,0);tick();stopped();
        packet(0,1);tick();
        if(pan_us!=1460 || reverse_us!=1580 || limited_us!=1510)
            $fatal(1,"left edge failed confirmed start/trim/reverse/clamp");
        for(i=0;i<12;i=i+1) begin
            previous=pan_us;packet(0,1);tick();
            if(pan_us<previous-8 || pan_us>previous || pan_us<1320 || reverse_us>1720)
                $fatal(1,"left-edge acceleration/direction/bounds failed");
        end
        $display("PASS: x=0/1279 with one missed frame starts in both directions, crosses model dead zone, preserves trim/reverse/clamps and smooth ramp");
        packet(640,1);stopped();
        packet(690,1);tick();stopped();packet(690,1);tick();
        if(pan_us!=1528 || reverse_us!=1512) $fatal(1,"near-centre start was incorrectly boosted");
        previous=pan_us;packet(1279,1);tick();
        if(pan_us!=previous+8) $fatal(1,"boost retriggered while already moving");
        clear_stop();
        packet(1279,1);tick();stopped();
        packet(0,0);tick();packet(0,0);tick();stopped();
        packet(1279,1);tick();stopped();
        packet(1279,1);tick();
        if(pan_us!=1580) $fatal(1,"confirmed-loss reset/reacquisition failed");
        clear_stop();
        packet(1279,1);tick();stopped();
        packet(0,0);
        while(pan.sample_age<499) @(negedge clk);
        @(posedge clk);#1;stopped();
        packet(1279,1);tick();stopped();
        packet(1279,1);tick();
        if(pan_us!=1580) $fatal(1,"watchdog did not discard stale pending start");
        clear_stop();packet(1279,1);tick();stopped();clear_stop();
        packet(1279,1);tick();stopped();packet(1279,1);tick();
        if(pan_us!=1580) $fatal(1,"K4 clear did not discard pending confirmation");
        packet(280,1);tick();stopped(); // Opposite observation stops first.
        packet(280,1);tick();stopped();packet(280,1);tick();
        if(pan_us!=1460) $fatal(1,"confirmed reversal did not start from STOP");
        clear_stop();packet(1280,1);tick();stopped();packet(4095,1);tick();stopped();
        $display("PASS: no near-centre/moving boost, repeated-loss/timeout/K4 reset, confirmed reversal, coordinate validity and no stale start");
        $finish;
    end
    initial begin #100000;$fatal(1,"edge startup test timeout");end
endmodule
