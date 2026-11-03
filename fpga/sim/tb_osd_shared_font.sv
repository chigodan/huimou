`timescale 1ns/1ps
// Compare the retained Logo to the saved mixer; the removed counter area must pass video through.
// The compile command supplies hdmi_mixer_reference from that original source.
module tb_osd_shared_font;
    reg clk=0,rst=0,user=0;
    always #5 clk=~clk;
    reg [11:0] x=0,y=0;
    reg de=0;
    reg [23:0] video=24'h456789;
    wire [23:0] actual,expected;
    wire actual_de,expected_de;
    hdmi_mixer dut(.I_clk(clk),.I_rst_n(rst),.I_mode(2'd2),.I_process_algo(3'd4),
        .I_capture_fps(10'd30),.I_display_fps(10'd60),.I_new_fps(10'd30),
        .I_capture_id(16'd1),.I_display_id(16'd1),.I_latency_ms(16'd67),
        .I_latency_valid(1'b1),.I_video_state(2'd1),
        .I_uart_override(1'b0),.I_uart_token(1'b0),.I_uart_algo(3'd0),
        .I_video_vsync(1'b0),.I_video_hsync(1'b0),.I_video_de(1'b0),
        .I_video_user(user),.I_video_last(1'b0),.I_debug_status(4'd0),
        .I_video_rd_data(video),.O_hdmi_data(actual),.O_hdmi_de(actual_de));
    hdmi_mixer_reference original(.I_clk(clk),.I_rst_n(rst),.I_mode(2'd2),.I_process_algo(3'd4),
        .I_capture_fps(10'd30),.I_display_fps(10'd60),.I_new_fps(10'd30),
        .I_capture_id(16'd1),.I_display_id(16'd1),.I_latency_ms(16'd67),
        .I_latency_valid(1'b1),.I_video_state(2'd1),
        .I_uart_override(1'b0),.I_uart_token(1'b0),.I_uart_algo(3'd0),
        .I_video_vsync(1'b0),.I_video_hsync(1'b0),.I_video_de(1'b0),
        .I_video_user(user),.I_video_last(1'b0),.I_debug_status(4'd0),
        .I_video_rd_data(video),.O_hdmi_data(expected),.O_hdmi_de(expected_de));
    task probe(input integer px,input integer py);
        begin
            @(negedge clk);x=px;y=py;de=1;
            @(posedge clk);#1;
            if(actual!==expected || actual_de!==expected_de)
                $fatal(1,"original OSD changed at (%0d,%0d): %h vs %h",px,py,actual,expected);
        end
    endtask
    integer px,py,n;
    initial begin
        force dut.S_x_2d=x;force original.S_x_2d=x;
        force dut.S_y_2d=y;force original.S_y_2d=y;
        force dut.S_video_de_2d=de;force original.S_video_de_2d=de;
        repeat(5) @(negedge clk);rst=1;repeat(5) @(negedge clk);
        user=1;@(negedge clk);user=0;repeat(110) @(negedge clk);
        for(py=0;py<160;py=py+1)
            for(px=0;px<160;px=px+1) probe(px,py);
        // Former 112x16 text area and its surrounding pixels have no overlay.
        for(n=0;n<3;n=n+1) begin
            @(negedge clk);
            case(n)
                0: video=24'h000000;
                1: video=24'hffffff;
                2: video=24'h456789;
            endcase
            for(py=119;py<137;py=py+1) begin
                for(px=374;px<488;px=px+1) begin
                    @(negedge clk);x=px;y=py;de=1;
                    @(posedge clk);#1;
                    if(!actual_de || actual!==video)
                        $fatal(1,"removed counter area changed video at (%0d,%0d): got=%h expected=%h",px,py,actual,video);
                end
            end
        end
        @(negedge clk);de=0;@(posedge clk);#1;
        if(actual!==0 || actual_de) $fatal(1,"shared font leaked into blanking");
        $display("PASS: unchanged bitmap Logo; removed counter region passes black/white/colour video; blanking preserved");
        $finish;
    end
    initial begin #1500000; $fatal(1,"shared font test timeout"); end
endmodule
