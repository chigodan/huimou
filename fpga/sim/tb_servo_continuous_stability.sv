`timescale 1ns/1ps
// Same asynchronous mailbox and fixed-point controller as the real design.
// Compare moving-target command noise against the unconditioned controller.
module tb_servo_continuous_stability;
    reg clk=0,pclk=0,rst=0,sample=0,valid=0,home=0;
    always #5 clk=~clk;
    always #7 pclk=~pclk;
    reg [11:0] x=640;
    wire [11:0] smooth_us,raw_us,reversed_us;
    servo_circle_track #(.CLK_FREQ_HZ(1_000_000),.CONTROL_PERIOD_US(100),
        .CENTER_US(1520),.MIN_US(1320),.MAX_US(1720),.SAMPLE_TIMEOUT_US(500),
        .GAIN_SHIFT(2),.MAX_STEP_US(8),.SPEED_CONTROL(1),.STABILITY_ENABLE(1)) smooth(
        .I_clk(clk),.I_rst_n(rst),.I_pixel_clk(pclk),.I_pixel_rst_n(rst),
        .I_sample(sample),.I_fresh_valid(valid),.I_center_x(x),.I_center_event(home),.O_pulse_us(smooth_us));
    servo_circle_track #(.CLK_FREQ_HZ(1_000_000),.CONTROL_PERIOD_US(100),
        .CENTER_US(1520),.MIN_US(1320),.MAX_US(1720),.SAMPLE_TIMEOUT_US(500),
        .GAIN_SHIFT(2),.MAX_STEP_US(8),.SPEED_CONTROL(1)) raw(
        .I_clk(clk),.I_rst_n(rst),.I_pixel_clk(pclk),.I_pixel_rst_n(rst),
        .I_sample(sample),.I_fresh_valid(valid),.I_center_x(x),.I_center_event(home),.O_pulse_us(raw_us));
    servo_circle_track #(.CLK_FREQ_HZ(1_000_000),.CONTROL_PERIOD_US(100),
        .CENTER_US(1520),.MIN_US(1320),.MAX_US(1720),.SAMPLE_TIMEOUT_US(500),
        .GAIN_SHIFT(2),.MAX_STEP_US(8),.SPEED_CONTROL(1),.STABILITY_ENABLE(1),.REVERSE(1)) reversed(
        .I_clk(clk),.I_rst_n(rst),.I_pixel_clk(pclk),.I_pixel_rst_n(rst),
        .I_sample(sample),.I_fresh_valid(valid),.I_center_x(x),.I_center_event(home),.O_pulse_us(reversed_us));
    task tick;
        begin
            @(negedge clk);while(!smooth.control_tick) @(negedge clk);
            @(posedge clk);#1;
        end
    endtask
    task packet(input integer position,input bit fresh);
        begin
            @(negedge pclk);
            while(smooth.sample_req!=smooth.ack_sync[1]) @(negedge pclk);
            x=position;valid=fresh;sample=1;
            @(negedge pclk);sample=0;repeat(10) @(negedge clk);
        end
    endtask
    task stopped;
        begin
            if(smooth_us!==1520 || reversed_us!==1520) $fatal(1,"calibrated STOP mismatch");
        end
    endtask
    task restart_right;
        begin
            packet(1000,1);tick();stopped();
            packet(1000,1);tick();
            if(smooth_us!=1528 || reversed_us!=1512) $fatal(1,"two-frame start or 8us acceleration failed");
        end
    endtask
    integer i,previous,lo,hi,raw_lo,raw_hi,position,plant_x_q8,settled_frames;
    initial begin
        repeat(5) @(negedge clk);rst=1;repeat(8) @(negedge clk);stopped();
        // One spurious observation or opposite-direction observations cannot
        // start the continuous servo from rest.
        packet(1000,1);tick();stopped();
        packet(280,1);tick();stopped();
        packet(1000,1);tick();stopped();
        @(negedge clk);home=1;@(negedge clk);home=0;stopped();
        restart_right();
        for(i=0;i<15;i=i+1) begin
            previous=smooth_us;packet(1000,1);tick();
            if(smooth_us>previous+8 || smooth_us<previous || smooth_us>1720)
                $fatal(1,"steady motion acceleration or pulse bounds failed");
        end
        if(smooth_us!=1607 || reversed_us!=1433) $fatal(1,"filtered constant-error proportional speed changed");
        lo=4095;hi=0;raw_lo=4095;raw_hi=0;
        for(i=0;i<40;i=i+1) begin
            previous=smooth_us;packet((i%2)==0 ? 980 : 1020,1);tick();
            if(smooth_us<=1520 || reversed_us>=1520 || smooth_us>previous+8 || smooth_us<previous-8)
                $fatal(1,"moving coordinate noise caused STOP/reversal or rate jump");
            if(i>=20) begin
                if(smooth_us<lo) lo=smooth_us;if(smooth_us>hi) hi=smooth_us;
                if(raw_us<raw_lo) raw_lo=raw_us;if(raw_us>raw_hi) raw_hi=raw_us;
            end
        end
        if(hi-lo>5 || hi-lo>=raw_hi-raw_lo) $fatal(1,"moving noise attenuation failed: smooth=%d raw=%d",hi-lo,raw_hi-raw_lo);
        $display("PASS: moving-target pulse ripple %0d us vs unfiltered %0d us, 8us acceleration, reverse setting and neutral trim",hi-lo,raw_hi-raw_lo);
        packet(0,0);tick();
        if(smooth_us<=1520 || reversed_us>=1520) $fatal(1,"one missed frame caused stop/start jerk");
        packet(1000,1);tick();
        if(smooth_us<=1520) $fatal(1,"single-frame loss recovery stopped motion");
        packet(0,0);tick();packet(0,0);tick();stopped();
        restart_right();
        packet(652,1);stopped();
        for(i=0;i<24;i=i+1) begin
            position=(i%2)==0 ? 653 : 664;
            packet(position,1);tick();stopped();
        end
        packet(640,1);tick();stopped();
        restart_right();
        packet(280,1);tick();stopped(); // Reversal first stops and clears filter.
        packet(280,1);tick();stopped();
        packet(280,1);tick();
        if(smooth_us!=1512 || reversed_us!=1528) $fatal(1,"confirmed reversal did not ramp from STOP");
        $display("PASS: single missed frame grace, consecutive loss STOP, centre hysteresis and fresh-frame reversal confirmation");
        packet(0,0); // A missed frame must not extend the last-valid timeout.
        while(smooth.sample_age<499) @(negedge clk);
        @(posedge clk);#1;stopped();repeat(3) tick();stopped();
        restart_right();
        packet(4095,1);tick();
        if(smooth_us<=1520) $fatal(1,"isolated out-of-image packet should use bounded loss grace");
        packet(1280,1);tick();stopped();
        restart_right();
        @(negedge clk);home=1;@(negedge clk);home=0;stopped();
        repeat(3) tick();stopped();restart_right();
        $display("PASS: exact last-valid watchdog, no stale restart, coordinate validity, K4 and fresh reacquisition");
        @(negedge clk);home=1;@(negedge clk);home=0;
        plant_x_q8=1000*256;settled_frames=0;
        for(i=0;i<220;i=i+1) begin
            // Illustrative camera feedback plus a slowly moving target and
            // +/-3 pixels of centroid noise, then a stationary target.
            position=plant_x_q8/256+((i%2)==0 ? 3 : -3);
            packet(position,1);tick();
            if(smooth_us<1520 || smooth_us>1720)
                $fatal(1,"visual feedback produced direction chatter or exceeded limits");
            plant_x_q8=plant_x_q8-(smooth_us-1520)*16;
            if(i<60) plant_x_q8=plant_x_q8+192;
            if(i>=200 && smooth_us==1520) settled_frames=settled_frames+1;
        end
        if(plant_x_q8/256<616 || plant_x_q8/256>664 || settled_frames!=20)
            $fatal(1,"noisy visual loop failed to converge and stay stopped: x=%d stopped=%d",plant_x_q8/256,settled_frames);
        $display("PASS: illustrative noisy moving-camera loop tracks a drifting target, converges and stays stopped without direction chatter");
        $finish;
    end
    initial begin #1000000;$fatal(1,"continuous stability test timeout");end
endmodule
