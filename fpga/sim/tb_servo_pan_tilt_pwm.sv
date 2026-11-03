`timescale 1ns/1ps
// Two actual 50 Hz generators on the same clock, with distinct pulse widths.
module tb_servo_pan_tilt_pwm;
    reg clk=0; always #500 clk=~clk;
    reg rst=0;
    reg [11:0] pan_command=1500, pitch_command=1500;
    wire pan_pwm, pitch_pwm;
    servo_pwm #(.CLK_FREQ_HZ(1_000_000)) pan(
        .I_clk(clk),.I_rst_n(rst),.I_pulse_us(pan_command),.O_pwm(pan_pwm));
    servo_pwm #(.CLK_FREQ_HZ(1_000_000)) pitch(
        .I_clk(clk),.I_rst_n(rst),.I_pulse_us(pitch_command),.O_pwm(pitch_pwm));
    time pan_rise=0, pitch_rise=0, last_pan=0, last_pitch=0;
    integer pan_frames=0, pitch_frames=0, pan_expected, pitch_expected;
    integer pan_checked=0, pitch_checked=0;
    always @(posedge pan_pwm) begin
        pan_frames=pan_frames+1;
        if(pan_frames>1) begin
            pan_rise=$time;
            if(last_pan!=0 && pan_rise-last_pan!=20_000_000) $fatal(1,"pan period wrong");
            last_pan=pan_rise;
            pan_expected=(pan_command<500)?500:((pan_command>2500)?2500:pan_command);
        end
    end
    always @(posedge pitch_pwm) begin
        pitch_frames=pitch_frames+1;
        if(pitch_frames>1) begin
            pitch_rise=$time;
            if(last_pitch!=0 && pitch_rise-last_pitch!=20_000_000) $fatal(1,"pitch period wrong");
            last_pitch=pitch_rise;
            pitch_expected=(pitch_command<500)?500:((pitch_command>2500)?2500:pitch_command);
        end
    end
    always @(negedge pan_pwm) if(rst && pan_frames>1) begin
        if($time-pan_rise != pan_expected*1000) $fatal(1,"pan pulse corrupted");
        pan_checked=pan_checked+1;
    end
    always @(negedge pitch_pwm) if(rst && pitch_frames>1) begin
        if($time-pitch_rise != pitch_expected*1000) $fatal(1,"pitch pulse corrupted");
        pitch_checked=pitch_checked+1;
    end
    initial begin
        repeat(5) @(negedge clk); rst=1;
        @(posedge pan_pwm); @(negedge pan_pwm); // Existing startup partial frame.
        pan_command=1800; pitch_command=1300;
        @(posedge pan_pwm);
        repeat(400) @(negedge clk);
        pan_command=1100; pitch_command=1700; // Mid-pulse changes affect next frame only.
        @(posedge pan_pwm);
        repeat(400) @(negedge clk);
        pan_command=3000; pitch_command=0;
        @(posedge pan_pwm);
        repeat(400) @(negedge clk);
        pan_command=1500; pitch_command=1500;
        @(posedge pan_pwm); repeat(2600) @(negedge clk);
        if(pan_checked!=4 || pitch_checked!=4) $fatal(1,"not all dual-channel widths checked");
        $display("PASS: independent pan/pitch widths, two 50 Hz channels, frame-atomic updates and pulse clamps");
        $finish;
    end
    initial begin #120000000; $fatal(1,"dual PWM test timeout"); end
endmodule
