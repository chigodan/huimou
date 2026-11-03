`timescale 1ns/1ps
module tb_servo_pwm_tracking;
    reg clk=0; always #500 clk=~clk; // Actual 1 MHz, one cycle per microsecond.
    reg rst=0;
    reg [11:0] command=1500;
    wire pwm;
    servo_pwm #(.CLK_FREQ_HZ(1_000_000)) dut(
        .I_clk(clk),.I_rst_n(rst),.I_pulse_us(command),.O_pwm(pwm));
    time rise, last_rise;
    task next_rise;
        begin
            @(posedge pwm); rise=$time;
            if(last_rise!=0 && rise-last_rise!=20_000_000)
                $fatal(1,"PWM period is not 20 ms");
            last_rise=rise;
        end
    endtask
    task width_check(input integer us);
        begin
            @(negedge pwm);
            if($time-rise != us*1000) $fatal(1,"PWM width expected %d us got %t",us,$time-rise);
        end
    endtask
    initial begin
        last_rise=0;
        repeat(5) @(negedge clk); rst=1;
        // Ignore existing generator's reset-start partial frame; check steady frames.
        @(posedge pwm); @(negedge pwm);
        next_rise();
        repeat(400) @(negedge clk); command=2100;
        width_check(1500); // Command change must not alter the current pulse.
        next_rise(); width_check(2100);
        command=3000;
        next_rise(); width_check(2500);
        command=0;
        next_rise(); width_check(500);
        command=1500;
        next_rise(); width_check(1500);
        $display("PASS: 50 Hz, exact pulse widths, frame-boundary latching, 500..2500 us PWM clamp");
        $finish;
    end
    initial begin #200000000; $fatal(1,"PWM test timeout"); end
endmodule
