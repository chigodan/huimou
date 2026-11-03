`timescale 1ns/1ps
// Real mixer, circle tracker, logo and character ROM; pixel source models
// video_out's two-clock request-to-data latency.  No vendor IP is required.
module tb_circle_preserved;
    localparam W=160,H=100;
    reg clk=0; always #5 clk=~clk;
    reg rst=0,de=0,user=0,last=0;
    reg [1:0] mode=3;
    reg [23:0] camera_pixel=0,data1=0,data2=0;
    always @(posedge clk) begin
        data1 <= de ? camera_pixel : 24'd0;
        data2 <= data1;
    end
    wire out_de;
    wire [23:0] out_data;
    wire circle_sample, circle_fresh;
    wire [11:0] circle_x, circle_y, servo_pulse, pitch_pulse;
    integer scene_y=50;
    reg servo_clk=0; always #7 servo_clk=~servo_clk;
    hdmi_mixer #(.H_OFFSET(0),.V_OFFSET(0),.IMG_WIDTH(W),.IMG_HEIGHT(H)) dut(
        .I_clk(clk),.I_rst_n(rst),.I_mode(mode),
        .I_video_vsync(1'b0),.I_video_hsync(1'b0),.I_video_de(de),
        .I_video_user(user),.I_video_last(last),.I_debug_status(4'd0),
        .O_video_rd_en(),.I_video_rd_data(data2),.O_hdmi_vsync(),
        .O_hdmi_hsync(),.O_hdmi_de(out_de),.O_hdmi_data(out_data),
        .O_circle_sample(circle_sample),.O_circle_fresh_valid(circle_fresh),
        .O_circle_center_x(circle_x),.O_circle_center_y(circle_y));
    servo_circle_track #(.CLK_FREQ_HZ(1_000_000),.CONTROL_PERIOD_US(200),
        .IMAGE_WIDTH(W),.CENTER_X(W/2),.DEADBAND_PIXELS(4)) servo(
        .I_pixel_clk(clk),.I_pixel_rst_n(rst),.I_sample(circle_sample),
        .I_fresh_valid(circle_fresh),.I_center_x(circle_x),
        .I_clk(servo_clk),.I_rst_n(rst),.I_center_event(1'b0),.O_pulse_us(servo_pulse));
    servo_circle_track #(.CLK_FREQ_HZ(1_000_000),.CONTROL_PERIOD_US(200),
        .IMAGE_WIDTH(H),.CENTER_X(H/2),.DEADBAND_PIXELS(4),
        .MIN_US(500),.MAX_US(2500),.GAIN_SHIFT(3),.MAX_STEP_US(24),.REVERSE(1)) pitch(
        .I_pixel_clk(clk),.I_pixel_rst_n(rst),.I_sample(circle_sample),
        .I_fresh_valid(circle_fresh),.I_center_x(circle_y),
        .I_clk(servo_clk),.I_rst_n(rst),.I_center_event(1'b0),.O_pulse_us(pitch_pulse));
    integer reds=0,greens=0;
    always @(negedge clk) if(out_de) begin
        if(out_data==24'hff0000) reds=reds+1;
        if(out_data==24'h00ff00) greens=greens+1;
    end
    task reset_scene;
        begin
            @(negedge clk); rst=0; de=0; user=0; last=0;
            repeat(4) @(negedge clk); rst=1;
            repeat(5) @(negedge clk);
            user=1; @(negedge clk); user=0;
            repeat(4) @(negedge clk);
        end
    endtask
    task image(input integer cx,input integer shape);
        integer x,y,dx,dy;
        begin
            for(y=0;y<H;y=y+1) begin
                for(x=0;x<W;x=x+1) begin
                    @(negedge clk); de=1; user=(x==0 && y==0); last=(x==W-1);
                    dx=x-cx; dy=y-scene_y;
                    if(shape==0)
                        camera_pixel=(dx*dx+dy*dy<=225) ? 24'd0 : 24'hffffff;
                    else if(shape==1)
                        camera_pixel=(dx>=-15 && dx<=15 && dy>=-15 && dy<=15) ?
                                     24'd0 : 24'hffffff;
                    else
                        camera_pixel=24'hffffff;
                end
                @(negedge clk); de=0; user=0; last=0;
                repeat(8) @(negedge clk);
            end
            repeat(20) @(negedge clk);
        end
    endtask
    task publish;
        begin
            @(negedge clk); user=1;
            @(negedge clk); user=0;
            repeat(3) @(negedge clk);
        end
    endtask
    task check_circle(input integer cx);
        begin
            if(!dut.S_circle_valid) $fatal(1,"circle missing at x=%d",cx);
            if(dut.S_circle_center_x<cx-2 || dut.S_circle_center_x>cx+2 ||
               dut.S_circle_center_y<scene_y-2 || dut.S_circle_center_y>scene_y+2)
                $fatal(1,"circle centre misaligned at x=%d: %d,%d",cx,
                    dut.S_circle_center_x,dut.S_circle_center_y);
        end
    endtask
    integer held_pulse, held_pitch;
    initial begin
        reset_scene(); image(40,0); publish(); check_circle(40);
        repeat(500) @(negedge servo_clk);
        if(servo_pulse>=1500) $fatal(1,"real left-circle result did not move servo left");
        if(pitch_pulse!=1500) $fatal(1,"horizontal circle offset moved pitch");
        reds=0; greens=0; image(40,0);
        if(reds==0 || greens==0) $fatal(1,"circle overlay missing");
        $display("PASS: left circle recognition and red-box/green-centre overlay");
        reset_scene(); image(120,0); publish(); check_circle(120);
        repeat(500) @(negedge servo_clk);
        if(servo_pulse<=1500) $fatal(1,"real right-circle result did not move servo right");
        $display("PASS: right circle recognition");
        scene_y=25;
        reset_scene(); image(120,0); publish(); check_circle(120);
        image(120,0); publish(); check_circle(120);
        repeat(2500) @(negedge servo_clk);
        if(pitch_pulse<=1500 || servo_pulse<=1500)
            $fatal(1,"real upper-right circle did not move both servos correctly");
        $display("PASS: real upper-right circle result drives pan right and tilt up");
        held_pulse=servo_pulse;
        held_pitch=pitch_pulse;
        image(120,2); publish();
        if(!dut.S_circle_valid || dut.S_circle_lost_frames==0 || circle_fresh)
            $fatal(1,"held display box reported as a fresh observation");
        repeat(500) @(negedge servo_clk);
        if(servo_pulse!=held_pulse) $fatal(1,"servo moved using retained display box");
        if(pitch_pulse!=held_pitch) $fatal(1,"pitch moved using retained display box");
        $display("PASS: real circle results cross clocks; retained box does not move either servo");
        scene_y=75;
        reset_scene(); image(40,0); publish(); check_circle(40);
        image(40,0); publish(); check_circle(40);
        repeat(2500) @(negedge servo_clk);
        if(pitch_pulse>=1500 || servo_pulse>=1500)
            $fatal(1,"real lower-left circle did not move both servos correctly");
        $display("PASS: real lower-left circle result drives pan left and tilt down");
        mode=0;
        image(40,0); publish();
        repeat(500) @(negedge servo_clk);
        held_pulse=servo_pulse; held_pitch=pitch_pulse;
        if(circle_fresh) $fatal(1,"camera mode reported fresh circle results");
        image(120,0); publish(); repeat(500) @(negedge servo_clk);
        if(servo_pulse!=held_pulse || pitch_pulse!=held_pitch)
            $fatal(1,"servos moved outside circle mode");
        $display("PASS: leaving circle mode holds both axes");
        mode=3; scene_y=50;
        reset_scene(); image(40,1); publish();
        if(dut.S_circle_valid) $fatal(1,"square accepted as circle");
        $display("PASS: square rejection");
        mode=2; reset_scene();
        force dut.S_video_de_2d=1;
        force dut.S_osd_hit=1;
        force dut.S_osd_color=24'h00e6ff;
        @(posedge clk); #1;
        if(out_data !== 24'h00e6ff) $fatal(1,"OSD branch no longer reachable");
        $display("PASS: original OSD output remains reachable");
        $finish;
    end
    initial begin #3000000; $fatal(1,"test timeout"); end
endmodule
