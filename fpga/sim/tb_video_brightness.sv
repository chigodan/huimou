`timescale 1ns/1ps
module tb_video_brightness;
    reg clk=0;
    always #5 clk=~clk;
    reg rst_n=0,sof=0,token=0,de=0,vs=0,hs=0;
    reg [7:0] value=0;
    reg [23:0] rgb=0;
    wire ack,ov,oh,od;
    wire [23:0] output_rgb;
    video_brightness dut (.clk(clk),.rst_n(rst_n),.frame_start(sof),
        .value_async(value),.token_async(token),.ack(ack),
        .vsync(vs),.hsync(hs),.de(de),.rgb(rgb),
        .out_vsync(ov),.out_hsync(oh),.out_de(od),.out_rgb(output_rgb));
    function [7:0] clamp(input integer c,delta);
        integer sum;
        begin sum=c+delta;clamp=sum<0 ? 0 : sum>255 ? 255:sum;end
    endfunction
    integer delta,c,checked=0;
    reg [23:0] expected;
    initial begin
        repeat(4) @(negedge clk);rst_n=1;
        // Exactly one registered pixel, with all sync/DE signals aligned.
        @(negedge clk);rgb=24'h123456;de=1;vs=1;hs=0;
        @(posedge clk);#1;
        if(output_rgb!==rgb || !od || !ov || oh) $fatal(1,"default brightness/sync");
        for(delta=-128;delta<=127;delta=delta+1) begin
            @(negedge clk);de=0;value=delta;token=!token;
            repeat(5) @(negedge clk);
            if(ack===token) $fatal(1,"brightness applied before frame boundary");
            sof=1;@(negedge clk);sof=0;
            if(ack!==token || dut.brightness!==value) $fatal(1,"frame boundary ACK");
            for(c=0;c<256;c=c+1) begin
                rgb={8'(c),8'(255-c),8'((c+73)%256)};de=1;vs=c[0];hs=c[1];
                expected={clamp(c,delta),clamp(255-c,delta),clamp((c+73)%256,delta)};
                @(posedge clk);#1;
                if(output_rgb!==expected || od!==de || ov!==vs || oh!==hs)
                    $fatal(1,"pixel delta=%0d c=%0d got %h expected %h",delta,c,output_rgb,expected);
                checked=checked+1;@(negedge clk);
            end
            de=0;rgb=24'hffffff;
            @(posedge clk);#1;
            if(od || output_rgb!==0) $fatal(1,"blanking must remain black");
        end
        @(negedge clk);rst_n=0;#1;
        if(ack!==0 || output_rgb!==0 || od) $fatal(1,"brightness reset");
        $display("PASS: %0d brightness pixels, all signed offsets, clipping, frame ACK, blanking and sync",checked);
        $finish;
    end
    initial begin #2000000;$fatal(1,"brightness pixel timeout");end
endmodule
