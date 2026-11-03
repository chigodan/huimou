`timescale 1ns/1ps
// Exercise the actual top-level servo wiring. Compile with -i to omit unrelated
// camera/DDR/HDMI IP; drive the committed mixer mode and circle result boundary.
module tb_servo_mode_home;
    reg clk=0, pclk=0;
    always #5 clk=~clk;
    always #7 pclk=~pclk;
    reg rst=0, sample=0, fresh=0, center_key=0;
    reg [1:0] mode=0, switches=0;
    reg [11:0] x=640, y=360;
    design_top_wrapper #(.TRACK_CENTER_US(1520)) dut(
        .I_sys_clk(clk),.I_rst_n(rst),.I_uart_rx(1'b1),
        .I_button(switches),.I_key(4'b1111));
    defparam dut.u_servo_circle_track.CLK_FREQ_HZ=1_000_000;
    defparam dut.u_servo_circle_track.CONTROL_PERIOD_US=100;
    defparam dut.u_servo_circle_track.SAMPLE_TIMEOUT_US=500;
    defparam dut.u_servo_pitch_track.CLK_FREQ_HZ=1_000_000;
    defparam dut.u_servo_pitch_track.CONTROL_PERIOD_US=100;
    defparam dut.u_servo_pwm.CLK_FREQ_HZ=1_000_000;
    defparam dut.u_servo_pitch_pwm.CLK_FREQ_HZ=1_000_000;

    initial begin
        force dut.S_100m_clk=clk;
        force dut.S_hdmi_pixel_clk=pclk;
        force dut.S_rst_n=rst;
        force dut.S_hdmi_rst_n=rst;
        force dut.S_hdmi_uart_mode=mode;
        force dut.S_circle_sample=sample;
        force dut.S_circle_fresh_valid=fresh;
        force dut.S_circle_center_x=x;
        force dut.S_circle_center_y=y;
        force dut.S_servo_center_event=center_key;
    end
    task tick;
        begin
            @(negedge clk);
            while(!dut.u_servo_circle_track.control_tick) @(negedge clk);
            @(posedge clk); #1;
        end
    endtask
    task packet(input bit valid_target);
        begin
            @(negedge pclk);
            while(dut.u_servo_circle_track.sample_req!=dut.u_servo_circle_track.ack_sync[1] ||
                  dut.u_servo_pitch_track.sample_req!=dut.u_servo_pitch_track.ack_sync[1])
                @(negedge pclk);
            fresh=valid_target; x=1000; y=60; sample=1;
            @(negedge pclk); sample=0;
            repeat(8) @(negedge clk);
        end
    endtask
    task check_home;
        begin
            if(dut.S_servo_pulse_us!==12'd1520 || dut.S_servo_pitch_us!==12'd1500)
                $fatal(1,"expected calibrated pan STOP and pitch HOME: pan=%d pitch=%d",
                    dut.S_servo_pulse_us,dut.S_servo_pitch_us);
        end
    endtask
    task enter_and_move;
        begin
            @(negedge pclk); mode=3;
            repeat(10) @(negedge clk);
            check_home();
            repeat(2) tick(); check_home(); // No movement before a new result.
            packet(1); tick(); // First fresh frame only arms the start.
            if(dut.S_servo_pulse_us!=1520)
                $fatal(1,"continuous pan started before two fresh frames");
            packet(0); tick(); // A single missed frame must not starve startup.
            if(dut.S_servo_pulse_us!=1520)
                $fatal(1,"missing frame was incorrectly counted as a start confirmation");
            packet(1); tick();
            if(dut.S_servo_pulse_us!=1580 || dut.S_servo_pitch_us<=1500)
                $fatal(1,"top-level confirmed far target did not apply bounded start boost/pitch control");
        end
    endtask
    integer i, held_pitch;
    initial begin
        repeat(6) @(negedge clk); rst=1;
        repeat(10) @(negedge clk); check_home();
        if(dut.u_servo_circle_track.SPEED_CONTROL!=1 ||
           dut.u_servo_circle_track.STABLE_SPEED!=1 ||
           dut.TRACK_MIN_US!=1300 || dut.TRACK_MAX_US!=1700 ||
           dut.u_servo_pitch_track.SPEED_CONTROL!=0)
            $fatal(1,"top must select continuous pan and unchanged position pitch");
        enter_and_move();
        held_pitch=dut.S_servo_pitch_us;
        packet(0); tick(); // One missed frame must not jerk the pan to STOP.
        if(dut.S_servo_pitch_us!=held_pitch || dut.S_servo_pulse_us<=1520)
            $fatal(1,"isolated missed target frame should preserve pan motion and pitch");
        packet(0); tick(); // Confirmed loss stops pan, without homing pitch.
        if(dut.S_servo_pitch_us!=held_pitch || dut.S_servo_pulse_us!=1520)
            $fatal(1,"target loss inside tracking must hold pitch and stop pan");
        // Raw switch movement does not exit a UART-selected tracking mode.
        switches=2; packet(1); tick();
        if(dut.S_servo_pitch_us<=held_pitch)
            $fatal(1,"raw physical switch incorrectly overrode committed tracking mode");
        $display("PASS: target loss holds pitch; effective mode takes precedence over raw switches");

        // All three non-tracking modes must home even with switches still at 11.
        switches=3;
        for(i=0;i<3;i=i+1) begin
            // Launch a coordinate packet immediately before changing mode,
            // so it is still crossing the mailbox when the exit is applied.
            @(negedge pclk); fresh=1; sample=1;
            @(negedge pclk); sample=0; mode=i;
            repeat(12) @(negedge clk); check_home();
            packet(1); tick(); check_home(); // Non-tracking coordinates ignored.
            repeat(3) tick(); check_home();
            enter_and_move();
            $display("PASS: tracking -> mode %0d homes pitch/stops pan, drains stale coordinates, re-enters cleanly",i);
        end
        @(negedge clk); center_key=1;
        @(negedge clk); center_key=0; check_home();
        $display("PASS: K4 retains home/stop behavior");

        // Verify the actual PWM receives the returned position and neutral.
        @(negedge pclk); mode=0;
        repeat(12) @(negedge clk); check_home();
        @(posedge dut.O_servo_pitch_pwm); #1;
        if(dut.u_servo_pitch_pwm.pulse_us_latched!=1500 ||
           dut.u_servo_pwm.pulse_us_latched!=1520)
            $fatal(1,"mode exit home did not reach both frame-latched PWM commands");
        $display("PASS: pitch HOME and calibrated horizontal STOP reach actual PWM frame latches");
        $finish;
    end
    initial begin #1000000; $fatal(1,"mode home test timeout"); end
endmodule
