`timescale 1ns/1ps
// Pixel-accurate target panel regression. Fixture controls detector results;
// tb_target_coordinates_detector verifies those results from real RGB images.
module tb_target_coordinates_osd;
    reg clk=0,rst=0,user=0;
    always #5 clk=~clk;
    reg [1:0] mode=3;
    reg [11:0] target_x=640,target_y=360,probe_x=0,probe_y=0;
    reg target_valid=1,probe_de=0;
    reg [3:0] lost_frames=0;
    wire [23:0] rgb;
    wire out_de,fresh,read_enable;
    wire [11:0] output_x,output_y;
    hdmi_mixer #(.IMG_WIDTH(1280),.IMG_HEIGHT(720)) dut(
        .I_clk(clk),.I_rst_n(rst),.I_mode(mode),.I_process_algo(3'd5),
        .I_capture_fps(10'd30),.I_display_fps(10'd60),.I_new_fps(10'd30),
        .I_capture_id(16'd1),.I_display_id(16'd1),.I_latency_ms(16'd67),
        .I_latency_valid(1'b1),.I_video_state(2'd1),
        .I_uart_override(1'b0),.I_uart_token(1'b0),.I_uart_algo(3'd0),
        .O_uart_ack(),.O_uart_report_mode(),
        .I_video_vsync(1'b0),.I_video_hsync(1'b0),.I_video_de(1'b0),
        .I_video_user(user),.I_video_last(1'b0),.I_debug_status(4'd0),
        .O_video_rd_en(read_enable),.I_video_rd_data(24'h456789),
        .O_hdmi_vsync(),.O_hdmi_hsync(),.O_hdmi_de(out_de),.O_hdmi_data(rgb),
        .O_circle_sample(),.O_circle_fresh_valid(fresh),.O_circle_center_x(output_x),.O_circle_center_y(output_y));
    reg [143:0] lines [0:1];
    reg [7:0] reference_char=0;
    reg [3:0] reference_row=0;
    wire [15:0] reference_bits;
    osd_char_lib reference_font(.I_char(reference_char),.I_row(reference_row),.O_row_bits(reference_bits));
    integer x,y,row,col,bit_col,fd;
    reg [23:0] text_color,expected;
    task frame;
        begin
            @(negedge clk);user=1;@(negedge clk);user=0;
            repeat(7) @(negedge clk);
        end
    endtask
    task render(input [255:0] filename);
        begin
            fd=$fopen(filename,"w");
            if(!fd) $fatal(1,"could not create coordinate preview");
            $fdisplay(fd,"P3\n160 48\n255");
            for(y=0;y<48;y=y+1) begin
                for(x=0;x<160;x=x+1) begin
                    @(negedge clk);probe_x=16+x;probe_y=656+y;probe_de=1;
                    reference_char=8'h20;reference_row=0;bit_col=0;
                    if(x>=8 && x<152 && y>=8 && y<40) begin
                        row=(y-8)/16;col=(x-8)/8;
                        reference_char=lines[row][143-col*8 -: 8];
                        reference_row=(y-8)%16;bit_col=15-((x-8)%8);
                    end
                    @(posedge clk);#1;
                    expected=24'h546d87;
                    if(x>=8 && x<152 && y>=8 && y<40 && reference_bits[bit_col]) expected=text_color;
                    if(!out_de || rgb!==expected)
                        $fatal(1,"target glyph mismatch x=%0d y=%0d got=%h expected=%h",x,y,rgb,expected);
                    $fdisplay(fd,"%0d %0d %0d",rgb[23:16],rgb[15:8],rgb[7:0]);
                end
            end
            $fclose(fd);
            @(negedge clk);probe_x=1000;probe_y=300;
            @(posedge clk);#1;
            if(rgb!==24'h456789 || read_enable!==0) $fatal(1,"annotation changed camera pixels/read requests outside panel");
            @(negedge clk);probe_de=0;
            @(posedge clk);#1;
            if(out_de || rgb!==0) $fatal(1,"target annotation leaked into blanking");
        end
    endtask
    initial begin
        force dut.S_x_2d=probe_x;force dut.S_y_2d=probe_y;force dut.S_video_de_2d=probe_de;
        force dut.S_circle_valid=target_valid;force dut.S_circle_lost_frames=lost_frames;
        force dut.S_circle_center_x=target_x;force dut.S_circle_center_y=target_y;
        repeat(5) @(negedge clk);rst=1;repeat(5) @(negedge clk);frame();
        if(!fresh || output_x!=640 || output_y!=360) $fatal(1,"existing feedback coordinates changed");
        lines[0]="TARGET: TRACKING  ";lines[1]="LIVE X:0640 Y:0360";text_color=24'h00ff00;
        target_x=1279;target_y=719; // The displayed pair must hold until next SOF.
        render("target_tracking.ppm");
        frame();lines[1]="LIVE X:1279 Y:0719";render("target_edge.ppm");
        lost_frames=1;frame();
        if(fresh || dut.S_target_state_latched!=2) $fatal(1,"retained box was marked as a fresh target");
        lines[0]="TARGET: LOST      ";lines[1]="LAST X:1279 Y:0719";text_color=24'h00ff00;
        render("target_lost.ppm");
        target_valid=0;lost_frames=0;frame();
        lines[0]="TARGET: NOT FOUND ";lines[1]="X:NA Y:NA         ";text_color=24'h00ff00;
        render("target_none.ppm");
        target_valid=1;target_x=1280;target_y=720;frame();
        if(dut.S_target_state_latched!=0) $fatal(1,"out-of-image coordinates must not be shown as valid");
        target_x=0;target_y=0;frame();
        if(dut.S_target_state_latched!=1 || dut.S_target_x_latched!=0 || dut.S_target_y_latched!=0)
            $fatal(1,"zero-based coordinate origin failed");
        mode=0;repeat(4) @(negedge clk);frame(); // Allow physical-switch synchronizer to settle.
        @(negedge clk);probe_x=40;probe_y=680;probe_de=1;
        @(posedge clk);#1;
        if(rgb!==24'h456789 || dut.S_target_panel || fresh || dut.S_target_state_latched!=0)
            $fatal(1,"target panel remains active outside circle tracking mode");
        $display("PASS: exact tracking/edge/lost/NA panel pixels, decimal coordinates, colors, origin and range limits");
        $display("PASS: stable per-frame coordinate pair, feedback passthrough, blanking and mode exit");
        $finish;
    end
    initial begin #5000000; $fatal(1,"target coordinate OSD timeout"); end
endmodule
