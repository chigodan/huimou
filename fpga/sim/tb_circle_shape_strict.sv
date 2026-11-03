`timescale 1ns/1ps
// Full RGB -> segmentation -> blobs -> contour check -> three-frame publication.
// Camera data has video_out's two-cycle latency; no detector signals are forced.
module tb_circle_shape_strict;
    localparam W=240,H=180;
    reg clk=0; always #5 clk=~clk;
    reg rst=0,de=0,user=0,last=0;
    reg [1:0] mode=3;
    reg [23:0] pixel=0,data1=0,data2=0;
    always @(posedge clk) begin data1<=de?pixel:24'd0; data2<=data1; end
    wire fresh, sample;
    wire [11:0] cx,cy;
    hdmi_mixer #(.H_OFFSET(0),.V_OFFSET(0),.IMG_WIDTH(W),.IMG_HEIGHT(H)) dut(
        .I_clk(clk),.I_rst_n(rst),.I_mode(mode),.I_video_vsync(1'b0),.I_video_hsync(1'b0),
        .I_video_de(de),.I_video_user(user),.I_video_last(last),.I_debug_status(4'd0),
        .O_video_rd_en(),.I_video_rd_data(data2),.O_hdmi_vsync(),.O_hdmi_hsync(),
        .O_hdmi_de(),.O_hdmi_data(),.O_circle_sample(sample),.O_circle_fresh_valid(fresh),
        .O_circle_center_x(cx),.O_circle_center_y(cy));

    task boundary;
        begin
            @(negedge clk); de=0; last=0; user=1;
            @(negedge clk); user=0;
            repeat(5) @(negedge clk);
        end
    endtask
    task reset_scene;
        begin
            @(negedge clk); rst=0; de=0; user=0; last=0;
            repeat(4) @(negedge clk); rst=1;
            repeat(5) @(negedge clk); boundary();
        end
    endtask
    // 0 circle; 1 square; 2 cross; 3 thick ring; 4 mild ellipse; 5 flat ellipse;
    // 6 clipped square; 7 saturated dark red circle; 8 grey circle;
    // 9 rotated mild ellipse; 10 circle with clutter; 11 blank;
    // 12 interrupted silhouette; 13 dim neutral circle.
    task frame_scene(input integer shape,input integer radius,input integer xpos,
                     input integer ypos,input bit expected_candidate);
        integer x,y,dx,dy,ax,ay,b;
        longint u,v,a2,b2;
        bit hit, clutter;
        begin
            for(y=0;y<H;y=y+1) begin
                for(x=0;x<W;x=x+1) begin
                    @(negedge clk); de=1; user=0; last=(x==W-1);
                    dx=x-xpos; dy=y-ypos; ax=dx<0?-dx:dx; ay=dy<0?-dy:dy;
                    hit=(dx*dx+dy*dy<=radius*radius);
                    if(shape==1) hit=(ax<=radius && ay<=radius);
                    else if(shape==2) hit=(ax<=radius && ay<=radius &&
                        (100*ax<55*radius || 100*ay<55*radius));
                    else if(shape==3) hit=hit && (100*(dx*dx+dy*dy)>12*radius*radius);
                    else if(shape==4 || shape==5) begin
                        b=(shape==4)?radius*85/100:radius*55/100;
                        hit=(dx*dx*b*b+dy*dy*radius*radius<=radius*radius*b*b);
                    end else if(shape==6) hit=(ax<=radius && ay<=radius &&
                                               (1000*(ax+ay)<=1368*radius));
                    else if(shape==9) begin
                        u=dx+dy; v=dy-dx; a2=radius*radius; b=radius*87/100; b2=b*b;
                        hit=(u*u*b2+v*v*a2<=2*a2*b2);
                    end else if(shape==11) hit=0;
                    else if(shape==12) hit=hit && (dy!=0);
                    clutter=(shape==10) &&
                        (((x>=10 && x<=60)&&(y>=10 && y<=65)) ||
                         ((x>=15 && x<=90)&&(y>=95 && y<=160)&&
                          ((x>=37 && x<=67)||(y>=115 && y<=140))) ||
                         (((x*17+y*31)%149)==0));
                    if(hit && shape==7) pixel=24'h500000;
                    else if(hit && shape==8) pixel=24'h505050;
                    else if(hit && shape==13) pixel=24'h323232;
                    else pixel=(hit || clutter)?24'd0:24'hffffff;
                end
                @(negedge clk); de=0; user=0; last=0;
                repeat(8) @(negedge clk);
            end
            repeat(14000) @(negedge clk);
            if(!dut.S_blob_scan_done || dut.S_blob_best_valid!==expected_candidate)
                $fatal(1,"shape=%d radius=%d candidate expected=%d got=%d done=%d",
                    shape,radius,expected_candidate,dut.S_blob_best_valid,dut.S_blob_scan_done);
            boundary();
        end
    endtask
    task acquire(input integer shape,input integer radius,input integer x,input integer y);
        integer i;
        begin
            reset_scene();
            for(i=0;i<3;i=i+1) begin
                frame_scene(shape,radius,x,y,1);
                if(i<2 && fresh) $fatal(1,"target published before three confirmations");
            end
            if(!fresh || !dut.S_circle_valid || cx<x-2 || cx>x+2 || cy<y-2 || cy>y+2)
                $fatal(1,"confirmed centre missing/misaligned: %d,%d expected %d,%d",cx,cy,x,y);
            $display("PASS: RGB shape=%d radius=%d confirmed at %d,%d",shape,radius,cx,cy);
        end
    endtask
    task reject(input integer shape,input integer radius);
        begin
            reset_scene(); frame_scene(shape,radius,120,90,0);
            if(fresh || dut.S_circle_valid) $fatal(1,"false target shape=%d",shape);
            $display("PASS: rejected RGB shape=%d radius=%d",shape,radius);
        end
    endtask
    initial begin
        acquire(0,10,120,90); // Far, 21-pixel diameter.
        acquire(0,30,120,90); // Quarter of the image width.
        acquire(0,65,120,90); // Near, substantially larger target.
        acquire(4,40,120,90); acquire(9,40,120,90);
        acquire(13,30,120,90); // Dim black must not be lost to a lower threshold.
        reject(1,30); reject(2,30); reject(3,35); reject(5,40);
        reject(6,40); reject(7,30); reject(8,30); reject(12,30);
        acquire(10,25,175,90);
        // A previously tracked target must not loosen the shape requirement.
        frame_scene(2,30,175,90,0);
        if(fresh || !dut.S_circle_valid || dut.S_circle_lost_frames==0)
            $fatal(1,"nearby non-round blob kept tracking fresh");
        frame_scene(11,25,175,90,0);
        if(fresh) $fatal(1,"blank frame published a fresh target");
        frame_scene(0,25,175,90,1);
        if(fresh) $fatal(1,"reacquisition skipped confirmation");
        frame_scene(0,25,175,90,1);
        if(fresh) $fatal(1,"reacquisition published after only two frames");
        frame_scene(0,25,175,90,1);
        if(!fresh) $fatal(1,"reacquisition failed after three valid frames");
        // Premature next frame / incomplete scan must not republish old data.
        boundary();
        if(fresh) $fatal(1,"incomplete frame reused the preceding detection");
        $display("PASS: complex clutter, no continuity bypass, retained box is not fresh, confirmed reacquisition");
        $finish;
    end
    initial begin #30000000; $fatal(1,"strict detector timeout"); end
endmodule
