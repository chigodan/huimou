`timescale 1ns/1ps
// Real mixer/font/logo and asynchronous ISP-state mailbox. Samples actual
// output RGB for all six banners; writes small PPM previews for visual review.
module tb_mode_status_osd;
    reg clk=0, isp_clk=0, rst=0;
    always #5 clk=~clk;
    always #7 isp_clk=~isp_clk;
    reg [1:0] mode=0;
    reg frame_start=0, uart_owner=0, uart_token=0;
    reg [2:0] uart_algo=0, requested_algo=0, applied_algo=0;
    reg isp_sof=0, isp_publish=0;
    wire [2:0] display_algo;
    always @(posedge isp_clk or negedge rst) begin
        if(!rst) begin applied_algo<=0; isp_publish<=0; end
        else begin
            if(isp_sof) applied_algo<=requested_algo;
            isp_publish<=isp_sof;
        end
    end
    uart_snapshot_mailbox #(.WIDTH(3)) state_mailbox(
        .src_clk(isp_clk),.src_rst_n(rst),.src_valid(isp_publish),.src_data(applied_algo),
        .dst_clk(clk),.dst_rst_n(rst),.dst_valid(),.dst_data(display_algo));
    wire [23:0] rgb;
    wire out_de, read_enable, fresh;
    hdmi_mixer #(.IMG_WIDTH(1280),.IMG_HEIGHT(720),.TELEMETRY_OSD_ENABLE(0)) dut(
        .I_clk(clk),.I_rst_n(rst),.I_mode(mode),.I_process_algo(display_algo),
        .I_uart_override(uart_owner),.I_uart_token(uart_token),.I_uart_algo(uart_algo),
        .O_uart_ack(),.O_uart_report_mode(),
        .I_video_vsync(1'b0),.I_video_hsync(1'b0),.I_video_de(1'b0),
        .I_video_user(frame_start),.I_video_last(1'b0),.I_debug_status(4'd0),
        .O_video_rd_en(read_enable),.I_video_rd_data(24'h456789),
        .O_hdmi_vsync(),.O_hdmi_hsync(),.O_hdmi_de(out_de),.O_hdmi_data(rgb),
        .O_circle_sample(),.O_circle_fresh_valid(fresh),
        .O_circle_center_x(),.O_circle_center_y());

    reg [11:0] probe_x=0, probe_y=0;
    reg probe_de=0;
    reg [143:0] expected_text;
    reg [7:0] expected_char=0;
    reg [3:0] expected_row=0;
    wire [15:0] expected_bits;
    osd_char_lib reference_font(.I_char(expected_char),.I_row(expected_row),.O_row_bits(expected_bits));
    integer x, y, fd, col, bit_col, ink_count;
    reg [23:0] expected_rgb;
    reg [255:0] preview_name;

    task frame;
        begin
            @(negedge clk); frame_start=1;
            @(negedge clk); frame_start=0;
            repeat(8) @(negedge clk);
        end
    endtask
    task report_isp(input [2:0] algo);
        begin
            @(negedge isp_clk); requested_algo=algo; isp_sof=1;
            @(negedge isp_clk); isp_sof=0;
            repeat(15) @(negedge clk);
            if(display_algo!==algo) $fatal(1,"coherent ISP state mailbox captured stale state");
        end
    endtask
    task select_mode(input [1:0] next_mode, input [2:0] isp_algo);
        begin
            @(negedge clk); mode=next_mode;
            report_isp(isp_algo); frame();
        end
    endtask
    task check_text(input [143:0] text_value);
        begin
            expected_text=text_value;
            if(dut.S_status_text!==text_value)
                $fatal(1,"wrong status text: %s expected %s",dut.S_status_text,text_value);
        end
    endtask
    task render_panel(input integer index);
        begin
            $sformat(preview_name,"status_%0d.ppm",index);
            fd=$fopen(preview_name,"w");
            if(fd==0) $fatal(1,"could not write preview");
            $fdisplay(fd,"P3\n160 32\n255"); ink_count=0;
            for(y=0;y<32;y=y+1) begin
                for(x=0;x<160;x=x+1) begin
                    @(negedge clk);
                    probe_x=1104+x; probe_y=16+y; probe_de=1;
                    expected_char=8'h20; expected_row=0; bit_col=0;
                    if(x>=8 && x<152 && y>=8 && y<24) begin
                        col=(x-8)/8;
                        expected_char=expected_text[143-col*8 -: 8];
                        expected_row=y-8;
                        bit_col=15-((x-8)%8);
                    end
                    @(posedge clk); #1;
                    expected_rgb=24'h546d87;
                    if(x>=8 && x<152 && y>=8 && y<24 && expected_bits[bit_col]) begin
                        expected_rgb=24'h00ff00; ink_count=ink_count+1;
                    end
                    if(out_de!==1'b1 || rgb!==expected_rgb)
                        $fatal(1,"banner pixel mismatch: mode=%0d x=%0d y=%0d rgb=%h expected=%h",index,x,y,rgb,expected_rgb);
                    $fdisplay(fd,"%0d %0d %0d",rgb[23:16],rgb[15:8],rgb[7:0]);
                end
            end
            $fclose(fd);
            if(ink_count<100) $fatal(1,"mode label contains no visible text");
            @(negedge clk); probe_x=1000;probe_y=200;
            @(posedge clk); #1;
            if(rgb!==24'h456789 || read_enable!==0)
                $fatal(1,"status overlay changed pixels outside panel or video read enable");
            @(negedge clk); probe_de=0;
            @(posedge clk); #1;
            if(out_de!==0 || rgb!==24'd0) $fatal(1,"status overlay leaked into blanking");
        end
    endtask
    initial begin
        force dut.S_x_2d=probe_x;
        force dut.S_y_2d=probe_y;
        force dut.S_video_de_2d=probe_de;
        repeat(5) @(negedge clk); rst=1;
        select_mode(0,0); check_text("MODE: CAMERA      "); render_panel(0);
        select_mode(1,1); check_text("MODE: EDGE        "); render_panel(1);
        report_isp(2); check_text("MODE: EDGE        "); // No mid-frame label change.
        frame(); check_text("MODE: GRAYSCALE   "); render_panel(2);
        report_isp(3); check_text("MODE: GRAYSCALE   ");
        frame(); check_text("MODE: REGION FILL "); render_panel(3);
        select_mode(2,4); check_text("MODE: OSD         "); render_panel(4);
        select_mode(3,5); check_text("MODE: CIRCLE TRACK"); render_panel(5);
        if(fresh) $fatal(1,"mode label fabricated a valid circle observation");
        $display("PASS: all six persistent labels, frame-boundary updates, exact RGB glyphs, blanking and pixel pass-through");

        mode=0; uart_owner=1; uart_algo=5; uart_token=1;
        repeat(5) frame(); check_text("MODE: CIRCLE TRACK");
        report_isp(2); uart_algo=3; uart_token=0;
        repeat(5) frame(); check_text("MODE: GRAYSCALE   ");
        report_isp(3); check_text("MODE: GRAYSCALE   ");
        frame(); check_text("MODE: REGION FILL ");
        uart_owner=0;
        repeat(5) frame(); check_text("MODE: CAMERA      ");
        $display("PASS: UART and switches select labels; processing name follows applied ISP state rather than requested algorithm");
        $finish;
    end
    initial begin #5000000; $fatal(1,"mode status test timeout"); end
endmodule
