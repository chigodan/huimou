`timescale 1ns/1ps
// Actual top-level continuous-pan calibration: pulse offset display,
// debounced keys, 1300..1700 us speed bounds, K4 STOP and frame-latched PWM.
module tb_servo_continuous_top_calibration;
    reg clk=0, pclk=0, rst=0;
    always #5 clk=~clk;
    always #7 pclk=~pclk;
    reg [3:0] keys=4'hf;
    design_top_wrapper #(.SERVO_MANUAL_TEST(1)) dut(
        .I_sys_clk(clk),.I_rst_n(rst),.I_uart_rx(1'b1),
        .I_button(2'b00),.I_key(keys));
    // Shorten only the key timer for simulation; retain 20-sample debounce.
    defparam dut.u_servo_test_control.CLK_FREQ_HZ=100_000;
    defparam dut.u_servo_pwm.CLK_FREQ_HZ=1_000_000;
    defparam dut.u_servo_pitch_pwm.CLK_FREQ_HZ=1_000_000;
    initial begin
        force dut.S_100m_clk=clk;
        force dut.S_hdmi_pixel_clk=pclk;
        force dut.S_rst_n=rst;
        force dut.S_hdmi_rst_n=rst;
        force dut.S_hdmi_vsync=1'b0;
        force dut.S_hdmi_hsync=1'b0;
        force dut.S_hdmi_de=1'b0;
        force dut.S_hdmi_user=1'b0;
        force dut.S_hdmi_last=1'b0;
    end
    integer stop_count=0, i;
    time rise_time;
    always @(posedge clk) if(rst && dut.S_servo_center_event) stop_count=stop_count+1;
    task press(input integer key_index);
        begin
            @(negedge clk); keys[key_index]=0;
            repeat(2500) @(negedge clk);
            keys[key_index]=1;
            repeat(2500) @(negedge clk);
        end
    endtask
    task check_speed(input integer pulse_us, input integer offset);
        begin
            repeat(5) @(negedge pclk);
            if(dut.S_servo_pulse_us!=pulse_us || dut.S_servo_pitch_us!=1500 ||
               dut.g_servo_manual.u_servo_test_display.speed_offset_us!=offset)
                $fatal(1,"manual pulse/offset/pitch incorrect: pulse=%0d offset=%0d pitch=%0d",
                    dut.S_servo_pulse_us,dut.g_servo_manual.u_servo_test_display.speed_offset_us,dut.S_servo_pitch_us);
        end
    endtask
    // Check the rendered character selection, not only angle arithmetic.
    task check_glyph(input integer row, input integer col, input [7:0] expected);
        begin
            @(negedge pclk);
            force dut.g_servo_manual.u_servo_test_display.x=120+col*8;
            force dut.g_servo_manual.u_servo_test_display.y=75+row*16;
            force dut.S_hdmi_de=1'b1;
            #1;
            if(dut.g_servo_manual.u_servo_test_display.char_code!==expected)
                $fatal(1,"manual page wrong glyph at row=%0d col=%0d",row,col);
            release dut.g_servo_manual.u_servo_test_display.x;
            release dut.g_servo_manual.u_servo_test_display.y;
            force dut.S_hdmi_de=1'b0;
        end
    endtask
    initial begin
        repeat(6) @(negedge clk); rst=1;
        check_speed(1500,0);
        $display("PASS: manual top starts at horizontal STOP and pitch HOME"); $fflush();
        check_glyph(0,6,"C"); check_glyph(0,7,"O");
        check_glyph(0,8,"N"); check_glyph(0,9,"T");
        check_glyph(2,0,"O"); check_glyph(2,8,"0");
        check_glyph(2,9,"0"); check_glyph(2,10,"0"); check_glyph(2,11,"0");
        check_glyph(6,12,"S"); check_glyph(6,15,"P");
        check_glyph(7,0,"S"); check_glyph(7,3,"P");
        $display("PASS: horizontal calibration reports CONT, pulse OFFSET and K4 STOP"); $fflush();
        press(2); // Default 10 us -> 50 us.
        if(dut.S_servo_step_us!=50) $fatal(1,"K3 step cycle incorrect");
        for(i=0;i<22;i=i+1) press(1);
        check_speed(1700,200);
        @(posedge dut.O_servo_pwm); rise_time=$time; #1;
        if(dut.u_servo_pwm.pulse_us_latched!=1700)
            $fatal(1,"manual maximum position did not reach actual PWM");
        @(negedge dut.O_servo_pwm);
        if($time-rise_time!=17000) $fatal(1,"actual maximum PWM pulse width incorrect");
        for(i=0;i<42;i=i+1) press(0);
        check_speed(1300,200);
        press(3); check_speed(1500,0);
        if(stop_count!=1) $fatal(1,"K4 did not generate exactly one stop event");
        press(2); // 50 us -> 1 us.
        press(1); check_speed(1501,1);
        press(0); check_speed(1500,0);
        $display("PASS: debounced K1/K2, 1300..1700 us endpoints, pulse offset, 1 us step, independent pitch and K4 STOP");
        $finish;
    end
    initial begin #50000000; $fatal(1,"position calibration test timeout"); end
endmodule
