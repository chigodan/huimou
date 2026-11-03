`timescale 1ns/1ps
module tb_image_auto_osd;
    reg clk=0,rst=0,user=0;
    always #5 clk=~clk;
    reg [9:0] cap_fps=30, out_fps=60, new_fps=30;
    reg [15:0] cap_id=16'h12af, display_id=16'h12ad, latency=67;
    reg valid=1;
    reg [1:0] image_flags=3;
    reg [2:0] ae_state=2;
    reg [1:0] state=1;
    wire [23:0] rgb;
    wire out_de;
    hdmi_mixer #(.IMG_WIDTH(1280),.IMG_HEIGHT(720),.IMAGE_OSD_ENABLE(1)) dut(
        .I_clk(clk),.I_rst_n(rst),.I_mode(2'b11),.I_process_algo(3'd5),
        .I_capture_fps(cap_fps),.I_display_fps(out_fps),.I_new_fps(new_fps),
        .I_capture_id(cap_id),.I_display_id(display_id),.I_latency_ms(latency),
        .I_latency_valid(valid),.I_video_state(state),
        .I_image_flags(image_flags),.I_ae_state(ae_state),
        .I_uart_override(1'b0),.I_uart_token(1'b0),.I_uart_algo(3'd0),
        .O_uart_ack(),.O_uart_report_mode(),
        .I_video_vsync(1'b0),.I_video_hsync(1'b0),.I_video_de(1'b0),
        .I_video_user(user),.I_video_last(1'b0),.I_debug_status(4'd0),
        .O_video_rd_en(),.I_video_rd_data(24'h456789),
        .O_hdmi_vsync(),.O_hdmi_hsync(),.O_hdmi_de(out_de),.O_hdmi_data(rgb),
        .O_circle_sample(),.O_circle_fresh_valid(),.O_circle_center_x(),.O_circle_center_y());
    reg [11:0] probe_x=0,probe_y=0;
    reg probe_de=0;
    reg [143:0] lines [0:5];
    reg [7:0] reference_char=0;
    reg [3:0] reference_row=0;
    wire [15:0] reference_bits;
    osd_char_lib reference_font(.I_char(reference_char),.I_row(reference_row),.O_row_bits(reference_bits));
    integer x,y,row,col,bit_col,fd;
    reg [23:0] expected;
    task frame;
        begin
            @(negedge clk);user=1;@(negedge clk);user=0;
            repeat(7) @(negedge clk);
        end
    endtask
    task render(input [255:0] filename);
        begin
            fd=$fopen(filename,"w");
            if(!fd) $fatal(1,"could not create telemetry preview");
            $fdisplay(fd,"P3\n160 112\n255");
            for(y=0;y<112;y=y+1) begin
                for(x=0;x<160;x=x+1) begin
                    @(negedge clk);probe_x=1104+x;probe_y=16+y;probe_de=1;
                    reference_char=8'h20;reference_row=0;bit_col=0;
                    if(x>=8 && x<152 && y>=8 && y<104) begin
                        row=(y-8)/16;col=(x-8)/8;
                        reference_char=lines[row][143-col*8 -: 8];
                        reference_row=(y-8)%16;bit_col=15-((x-8)%8);
                    end
                    @(posedge clk);#1;
                    expected=24'h546d87;
                    if(x>=8 && x<152 && y>=8 && y<104 && reference_bits[bit_col]) expected=24'h00ff00;
                    if(!out_de || rgb!==expected)
                        $fatal(1,"telemetry glyph mismatch x=%0d y=%0d got=%h expected=%h",x,y,rgb,expected);
                    $fdisplay(fd,"%0d %0d %0d",rgb[23:16],rgb[15:8],rgb[7:0]);
                end
            end
            $fclose(fd);
            @(negedge clk);probe_x=1000;probe_y=200;
            @(posedge clk);#1;
            if(rgb!==24'h456789) $fatal(1,"telemetry changed pixels outside panel");
            @(negedge clk);probe_de=0;
            @(posedge clk);#1;
            if(out_de || rgb!==0) $fatal(1,"telemetry leaked into blanking");
        end
    endtask
    initial begin
        force dut.S_x_2d=probe_x;force dut.S_y_2d=probe_y;force dut.S_video_de_2d=probe_de;
        repeat(5) @(negedge clk);rst=1;repeat(5) @(negedge clk);frame();
        lines[0]="MODE: CIRCLE TRACK";lines[1]="FPS C030 D060 N030";
        lines[2]="LATENCY: 0067 MS  ";lines[3]="AWB:ON  AE:ON     ";
        lines[4]="VIDEO: LIVE       ";lines[5]="AE: STABLE        ";
        image_flags=0;ae_state=5;
        render("image_auto_on.ppm"); // Inputs changing mid-frame cannot tear text.
        frame();lines[3]="AWB:OFF AE:OFF    ";lines[5]="AE: HOLD / OFF    ";
        render("image_auto_off.ppm");
        image_flags=1;ae_state=6;frame();
        lines[3]="AWB:ON  AE:OFF    ";lines[5]="AE: I2C ERROR     ";
        render("image_auto_fault.ppm");
        $display("PASS: exact AWB/AE flags and stable/off/fault glyphs, retained FPS/latency/video rows, alpha background and frame snapshots");
        $finish;
    end
    initial begin #5000000; $fatal(1,"telemetry OSD test timeout"); end
endmodule
