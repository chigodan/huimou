`timescale 1ns/1ps
module tb_isp_luma_stats;
    reg clk=0,rst=0,sof=0,valid=0;
    reg [95:0] rgb=0;
    always #5 clk=~clk;
    wire sample;
    wire [7:0] mean;
    isp_luma_stats #(.MIN_SAMPLES(2)) dut(
        .clk(clk),.rst_n(rst),.frame_start(sof),.valid(valid),.rgb(rgb),
        .sample_valid(sample),.mean_luma(mean));
    task finish_frame(input integer expected);
        integer n;
        begin
            @(negedge clk);valid=0;sof=1;
            @(negedge clk);sof=0;
            n=0;
            while(!sample && n<30) begin @(negedge clk);n=n+1;end
            if(!sample || mean!==expected) $fatal(1,"Mean got %0d expected %0d",mean,expected);
            @(negedge clk);
        end
    endtask
    integer n,l;
    initial begin
        repeat(3) @(negedge clk);rst=1;
        for(n=0;n<64;n=n+1) begin
            @(negedge clk);valid=1;
            case(n/16) 0:l=0;1:l=64;2:l=128;3:l=255;endcase
            rgb={72'hffffff_ffffff_ffffff,8'(l),8'(l),8'(l)};
            if(n==31) begin @(negedge clk);valid=0;repeat(7) @(negedge clk);end
        end
        finish_frame(111);
        // Full 720p frame; 14,400 samples must fit without overflow.
        for(n=0;n<230400;n=n+1) begin
            @(negedge clk);valid=1;rgb={72'd0,8'd200,8'd100,8'd40};
        end
        finish_frame(110);
        // Frame-start may coincide with its first valid pixel.
        @(negedge clk);sof=1;valid=1;rgb={4{24'hffffff}};
        @(negedge clk);sof=0;
        repeat(31) @(negedge clk);
        finish_frame(255);
        $display("PASS: pre-AWB luma sampling, valid gaps, RGB weights, simultaneous SOF, full 720p frame and 24-cycle division");
        $finish;
    end
    initial begin #4000000;$fatal(1,"Statistics timeout");end
endmodule

module tb_camera_auto_exposure;
    reg clk=0,rst=0,enabled=1,ready=0,write_ready=1,error=0,capture=1,ack_enabled=1;
    reg sample=0;
    reg [7:0] mean=128;
    always #5 clk=~clk;
    wire request,applied_valid,mean_valid,tick;
    wire [15:0] exposure,gain,applied_exposure;
    wire [7:0] applied_gain,measured;
    wire [2:0] state;
    camera_auto_exposure #(.CLK_HZ(1000),.SETTLE_FRAMES(2)) dut(
        .clk(clk),.rst_n(rst),.enabled(enabled),.sensor_ready(ready),.write_ready(write_ready),
        .i2c_error(error),.sample_valid(sample),.mean_luma(mean),.request(request),
        .exposure(exposure),.gain(gain),.applied_exposure(applied_exposure),.applied_gain(applied_gain),
        .applied_valid(applied_valid),.mean_valid(mean_valid),.measured_luma(measured),.state(state),.report_tick(tick));
    integer ambient=60,frame_count=0,bus_delay=0,requests=0;
    integer sensor_exposure=1991,sensor_gain=144;
    reg [15:0] held_exposure,held_gain;
    // Independent sensor response model; uses the current gain table's
    // coarse stages and fine gain, not the controller's error/step logic.
    function integer response(input integer ex,ag,light);
        integer step,stage,fine;
        reg [63:0] value;
        begin
            step=ag-16;
            if(step>=160) begin stage=32;fine=128;end
            else begin stage=1<<(step/32);fine=128+(step%32)*4;end
            value=64'(ex)*light*stage*fine/(1991*128);
            response=(value>255) ? 255 : value;
        end
    endfunction
    always @(posedge clk) begin
        if(!rst) begin frame_count<=0;bus_delay<=0;write_ready<=1;sample<=0;end
        else begin
            sample<=0;
            if(frame_count==49) begin
                frame_count<=0;
                if(capture) begin sample<=1;mean<=8'(response(sensor_exposure,sensor_gain,ambient));end
            end else frame_count<=frame_count+1;
            if(request) begin
                requests=requests+1;
                if(bus_delay!=0 || !write_ready) $fatal(1,"Overlapping sensor transaction");
                held_exposure=exposure;held_gain=gain;
                if(ack_enabled) begin bus_delay<=12;write_ready<=0;end
            end else if(bus_delay!=0) begin
                if(exposure!==held_exposure || gain!==held_gain) $fatal(1,"Sensor payload changed while busy");
                bus_delay<=bus_delay-1;
                if(bus_delay==1) begin
                    sensor_exposure=held_exposure;sensor_gain=held_gain;write_ready<=1;
                end
            end
            if(exposure<8 || exposure>1991 || gain<16 || gain>176) $fatal(1,"AE parameter outside limits");
        end
    end
    task reach(input [2:0] expected);
        integer limit;
        begin
            limit=0;
            // Require fresh settling after changing illumination.
            repeat(160) @(negedge clk);
            while(state!==expected && limit<60000) begin @(negedge clk);limit=limit+1;end
            if(state!==expected) $fatal(1,"AE state %0d expected %0d: light=%0d mean=%0d exp=%0d gain=%0d",state,expected,ambient,mean,exposure,gain);
        end
    endtask
    integer saved_requests;
    reg [15:0] saved_exposure,saved_gain;
    initial begin
        repeat(3) @(negedge clk);rst=1;
        repeat(20) @(negedge clk);
        if(requests!=0) $fatal(1,"Wrote sensor before initialization");
        ready=1;reach(2);
        if(mean<120 || mean>136 || !applied_valid) $fatal(1,"AE did not converge");
        saved_requests=requests;repeat(400) @(negedge clk);
        if(requests!=saved_requests) $fatal(1,"Deadband caused repeated writes");
        ambient=15;reach(2);
        ambient=400;reach(2);
        if(gain!=16 || exposure>=1991) $fatal(1,"Bright-scene exposure adjustment failed");
        enabled=0;reach(5);saved_exposure=exposure;saved_gain=gain;saved_requests=requests;
        ambient=1;repeat(600) @(negedge clk);
        if(exposure!=saved_exposure || gain!=saved_gain || requests!=saved_requests) $fatal(1,"AE OFF did not freeze");
        enabled=1;reach(3);
        if(exposure!=1991 || gain!=176) $fatal(1,"Dark limit wrong");
        ambient=100000;reach(4);
        if(exposure!=8 || gain!=16) $fatal(1,"Bright limit wrong");
        capture=0;repeat(350) @(negedge clk);
        if(mean_valid || state!=0) $fatal(1,"Stale statistics still active");
        saved_requests=requests;repeat(300) @(negedge clk);
        if(requests!=saved_requests) $fatal(1,"AE wrote using stale statistics");
        // A sensor that never lowers write_ready must fault, rather than
        // reporting an exposure update that was never acknowledged.
        @(negedge clk);rst=0;ack_enabled=0;capture=0;
        repeat(3) @(negedge clk);rst=1;
        repeat(550) @(negedge clk);
        if(state!=6 || applied_valid) $fatal(1,"I2C handshake timeout not reported");
        $display("PASS: AE bright/dark convergence, deadband, limits, OFF hold, I2C payload/ack, stale video and timeout");
        $finish;
    end
    initial begin #3000000;$fatal(1,"AE test timeout");end
endmodule

module tb_image_frame_settings;
    reg clk=0,rst=0,frame=0,token=0;
    reg [1:0] requested=3;
    always #5 clk=~clk;
    wire [1:0] flags;
    wire ack;
    image_frame_settings dut(.clk(clk),.rst_n(rst),.frame_start(frame),
        .token_async(token),.flags_async(requested),.flags(flags),.ack(ack));
    task boundary;
        begin @(negedge clk);frame=1;@(negedge clk);frame=0;end
    endtask
    initial begin
        repeat(3) @(negedge clk);rst=1;
        if(flags!=3 || ack) $fatal(1,"Default image flags");
        requested=0;token=1;repeat(5) @(negedge clk);
        if(flags!=3 || ack) $fatal(1,"Image flags changed mid-frame");
        boundary();
        if(flags!=0 || ack) $fatal(1,"Missing setting or early ACK");
        boundary();if(!ack) $fatal(1,"Missing downstream settle ACK");
        requested=1;token=0;repeat(5) @(negedge clk);boundary();boundary();
        if(flags!=1 || ack) $fatal(1,"Toggle round trip");
        $display("PASS: frame-boundary image settings, delayed consumer ACK and toggle round trip");
        $finish;
    end
endmodule
