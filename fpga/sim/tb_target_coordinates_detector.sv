`timescale 1ns/1ps
// Real RGB segmentation/blob tracker -> published feedback -> annotation.
// Small diagnostic frame: metadata remains active, large panels are suppressed.
module tb_target_coordinates_detector;
    localparam W=160,H=100;
    reg clk=0,rst=0,de=0,user=0,last=0;
    always #5 clk=~clk;
    reg [1:0] mode=3;
    reg [23:0] pixel=0,data1=0,data2=0;
    always @(posedge clk) begin data1<=de?pixel:24'd0;data2<=data1;end
    wire fresh;
    wire [11:0] cx,cy;
    hdmi_mixer #(.H_OFFSET(0),.V_OFFSET(0),.IMG_WIDTH(W),.IMG_HEIGHT(H)) dut(
        .I_clk(clk),.I_rst_n(rst),.I_mode(mode),.I_process_algo(3'd5),
        .I_capture_fps(10'd0),.I_display_fps(10'd0),.I_new_fps(10'd0),
        .I_capture_id(16'd0),.I_display_id(16'd0),.I_latency_ms(16'd0),
        .I_latency_valid(1'b0),.I_video_state(2'd0),
        .I_uart_override(1'b0),.I_uart_token(1'b0),.I_uart_algo(3'd0),
        .O_uart_ack(),.O_uart_report_mode(),
        .I_video_vsync(1'b0),.I_video_hsync(1'b0),.I_video_de(de),
        .I_video_user(user),.I_video_last(last),.I_debug_status(4'd0),
        .O_video_rd_en(),.I_video_rd_data(data2),.O_hdmi_vsync(),.O_hdmi_hsync(),.O_hdmi_de(),.O_hdmi_data(),
        .O_circle_sample(),.O_circle_fresh_valid(fresh),.O_circle_center_x(cx),.O_circle_center_y(cy));
    task publish;
        begin
            @(negedge clk);user=1;@(negedge clk);user=0;
            repeat(8) @(negedge clk);
        end
    endtask
    task image(input integer xpos,input integer ypos,input bit present);
        integer x,y,dx,dy;
        begin
            for(y=0;y<H;y=y+1) begin
                for(x=0;x<W;x=x+1) begin
                    @(negedge clk);de=1;user=(x==0 && y==0);last=(x==W-1);
                    dx=x-xpos;dy=y-ypos;
                    pixel=(present && dx*dx+dy*dy<=225) ? 24'd0 : 24'hffffff;
                end
                @(negedge clk);de=0;user=0;last=0;
                repeat(8) @(negedge clk);
            end
            repeat(20) @(negedge clk);
        end
    endtask
    integer i,held_x,held_y;
    initial begin
        repeat(5) @(negedge clk);rst=1;repeat(5) @(negedge clk);publish();
        image(40,50,1);publish();
        if(!fresh || cx<38 || cx>42 || cy<48 || cy>52 || dut.S_target_state_latched!=1 ||
           dut.S_target_x_latched!=cx || dut.S_target_y_latched!=cy)
            $fatal(1,"real detected circle did not reach coordinate annotation");
        held_x=cx;held_y=cy;
        image(0,0,0);publish();
        if(fresh || dut.S_target_state_latched!=2 || dut.S_target_x_latched!=held_x || dut.S_target_y_latched!=held_y)
            $fatal(1,"real detector retained box was not labeled LAST/LOST");
        for(i=0;i<8;i=i+1) begin image(0,0,0);publish();end
        if(fresh || dut.S_target_state_latched!=0) $fatal(1,"expired real target was not cleared to NA");
        image(120,25,1);publish();
        if(!fresh || dut.S_target_state_latched!=1 || dut.S_target_x_latched!=cx ||
           dut.S_target_y_latched!=cy || cx<118 || cx>122 || cy<23 || cy>27)
            $fatal(1,"real target reacquisition did not replace annotation coordinates");
        mode=0;image(120,25,1);publish();
        if(fresh || dut.S_target_state_latched!=0) $fatal(1,"mode exit did not clear target annotation");
        $display("PASS: real RGB circle -> published centre -> annotation, retained loss, expiry, reacquisition and mode exit");
        $finish;
    end
    initial begin #4000000; $fatal(1,"real detector annotation test timeout"); end
endmodule
