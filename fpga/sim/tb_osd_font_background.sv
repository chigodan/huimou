`timescale 1ns/1ps
module tb_osd_font_background;
    reg clk=0,rst=0,user=0,de=0;
    always #5 clk=~clk;
    reg [1:0] mode=0;
    reg [11:0] x=0,y=0;
    reg [23:0] video=0;
    reg target_valid=1;
    reg [3:0] lost=0;
    wire [23:0] rgb;
    wire out_de;
    hdmi_mixer dut(.I_clk(clk),.I_rst_n(rst),.I_mode(mode),.I_process_algo(3'd0),
        .I_capture_fps(10'd60),.I_display_fps(10'd60),.I_new_fps(10'd59),
        .I_capture_id(16'h31cf),.I_display_id(16'h31cd),.I_latency_ms(16'd38),
        .I_latency_valid(1'b1),.I_video_state(2'd1),
        .I_uart_override(1'b0),.I_uart_token(1'b0),.I_uart_algo(3'd0),
        .I_video_vsync(1'b0),.I_video_hsync(1'b0),.I_video_de(1'b0),
        .I_video_user(user),.I_video_last(1'b0),.I_debug_status(4'd0),
        .I_video_rd_data(video),.O_hdmi_data(rgb),.O_hdmi_de(out_de));
    reg [7:0] font_char=0;
    reg [3:0] font_row=0;
    wire [15:0] bits;
    osd_char_lib font(.I_char(font_char),.I_row(font_row),.O_row_bits(bits));
    function [23:0] blend_ref(input [23:0] pixel);
        integer c,n;
        begin
            for(c=0;c<3;c=c+1) begin
                n=(3*integer'(pixel[c*8 +: 8])+128+2)/4;
                blend_ref[c*8 +: 8]=n;
            end
        end
    endfunction
    task frame;
        begin
            repeat(4) @(negedge clk);
            user=1;@(negedge clk);user=0;repeat(110) @(negedge clk);
        end
    endtask
    task probe(input integer px,py,input [23:0] pixel,expected);
        begin
            @(negedge clk);x=px;y=py;video=pixel;de=1;
            @(posedge clk);#1;
            if(!out_de || rgb!==expected) $fatal(1,"alpha/text pixel (%d,%d) got=%h expected=%h",px,py,rgb,expected);
        end
    endtask
    reg [15:0] font_reference[0:4095];
    integer address,n,ry,cx,ink_x,ink_y;
    reg [23:0] pixel;
    task first_ink(input integer px,py,input [7:0] character,input [23:0] base);
        begin
            ink_x=-1;ink_y=-1;
            for(ry=0;ry<16;ry=ry+1)
                for(cx=0;cx<8;cx=cx+1)
                    if(ink_x<0 && font_reference[character*16+ry][15-cx]) begin
                        ink_x=cx;ink_y=ry;
                    end
            if(ink_x<0) $fatal(1,"reference character is blank");
            probe(px+ink_x,py+ink_y,base,24'h00ff00);
        end
    endtask
    initial begin
        $readmemh("simsun14_expected.mem",font_reference);
        font_char=8'hff;font_row=4'hf;#1;
        for(address=0;address<4096;address=address+1) begin
            font_char=address/16;font_row=address%16;#1;
            if(bits!==font_reference[address])
                $fatal(1,"SimSun source glyph mismatch at char=%h row=%0d",font_char,font_row);
        end
        $display("PASS: all 4096 font addresses match native SimSun 14 px/em reference, including Chinese and blank codes");
        force dut.S_x_2d=x;force dut.S_y_2d=y;force dut.S_video_de_2d=de;
        force dut.S_circle_valid=target_valid;force dut.S_circle_lost_frames=lost;
        force dut.S_circle_center_x=12'd640;force dut.S_circle_center_y=12'd360;
        repeat(5) @(negedge clk);rst=1;frame();
        for(n=0;n<256;n=n+1) begin
            pixel={8'(n),8'(255-n),8'((n*73)%256)};
            probe(1104,16,pixel,blend_ref(pixel));
            first_ink(1112,24,"M",pixel);
            probe(1000,300,pixel,pixel);
        end
        mode=3;frame();
        for(n=0;n<256;n=n+1) begin
            pixel={3{8'(n)}};
            probe(16,656,pixel,blend_ref(pixel));
            first_ink(24,664,"T",pixel);
        end
        lost=1;frame();first_ink(24,664,"T",24'd0);
        target_valid=0;frame();first_ink(24,664,"T",24'hffffff);
        @(negedge clk);de=0;@(posedge clk);#1;
        if(out_de || rgb!==0) $fatal(1,"OSD leaked into blanking");
        $display("PASS: 75%% video/25%% grey blend, all-green text in TRACKING/LOST/NOT FOUND, raw pixels and blanking");
        $finish;
    end
    initial begin #200000;$fatal(1,"font/background test timeout");end
endmodule
