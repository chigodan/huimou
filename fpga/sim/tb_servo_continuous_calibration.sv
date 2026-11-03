`timescale 1ns/1ps
// Legacy generic continuous-mode regression; not the fitted top-level servos.
// The current position calibration display is tested in tb_servo_position_calibration.
module tb_servo_continuous_calibration;
    reg clk=0, rst=0;
    always #5 clk=~clk;
    reg [3:0] keys=4'hf;
    wire [11:0] pulse;
    wire [6:0] step_us;
    wire stop_event, pwm;
    servo_test_control #(.CLK_FREQ_HZ(1_000_000),.CENTER_US(1520),
        .MIN_US(1300),.MAX_US(1700)) manual_control(
        .I_clk(clk),.I_rst_n(rst),.I_key_n(keys),.O_pulse_us(pulse),
        .O_step_us(step_us),.O_center_event(stop_event));
    servo_pwm #(.CLK_FREQ_HZ(1_000_000),.START_PULSE_US(1520),
        .MIN_PULSE_US(1300),.MAX_PULSE_US(1700)) generator(
        .I_clk(clk),.I_rst_n(rst),.I_pulse_us(pulse),.O_pwm(pwm));
    integer stop_count=0, i;
    time rise_time;
    always @(posedge clk) if(rst && stop_event) stop_count=stop_count+1;
    task press(input integer key_index);
        begin
            @(negedge clk); keys[key_index]=0;
            repeat(25000) @(negedge clk);
            keys[key_index]=1;
            repeat(25000) @(negedge clk);
        end
    endtask
    initial begin
        repeat(5) @(negedge clk);
        if(pulse!=1520 || generator.pulse_us_latched!=1520)
            $fatal(1,"calibrated STOP must apply to both controller and PWM reset");
        rst=1;
        @(posedge pwm); @(negedge pwm); // Skip startup partial frame.
        @(posedge pwm); rise_time=$time;
        @(negedge pwm);
        if($time-rise_time!=15200) $fatal(1,"custom neutral pulse width wrong");
        press(2);
        if(step_us!=50) $fatal(1,"K3 speed-pulse adjustment step failed");
        for(i=0;i<5;i=i+1) press(1);
        if(pulse!=1700)
            $fatal(1,"K2/manual upper speed clamp failed");
        for(i=0;i<9;i=i+1) press(0);
        if(pulse!=1300)
            $fatal(1,"K1/manual lower speed clamp failed");
        press(3);
        if(pulse!=1520 || stop_count!=1)
            $fatal(1,"K4 must issue one STOP and restore calibrated neutral");
        $display("PASS: legacy debounced manual pulse calibration, bounds, K4 and custom-neutral PWM");
        $finish;
    end
    initial begin #10000000; $fatal(1,"continuous calibration test timeout"); end
endmodule
