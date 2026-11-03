`timescale 1ns/1ps
// Independent spatial reference; covers continuous beats, bubbles and frame restart.
module tb_edge_gaussian;
    localparam W=32, H=12, TH=58;
    reg clk=0;
    always #5 clk=~clk;
    reg rst_n=0, valid=0, sof=0, eol=0;
    reg [95:0] pixels=0;
    wire [95:0] edges, gray, filled;
    wire out_valid, out_sof, out_eol;
    edge_detect_96 #(.IMG_WIDTH(W),.IMG_HEIGHT(H),.EDGE_TH(TH)) dut (
        .I_clk(clk),.I_rst_n(rst_n),.I_tdata(pixels),.I_tvalid(valid),
        .I_tuser(sof),.I_tlast(eol),.I_tready(1'b1),
        .O_tdata(edges),.O_gray_data(gray),.O_fill_data(filled),
        .O_tvalid(out_valid),.O_tuser(out_sof),.O_tlast(out_eol));

    integer image[0:H-1][0:W-1];
    function integer sample_gray(input integer y,x);
        integer c;
        begin
            if (x<0) sample_gray=0;
            else begin
                c=image[y][x];
                sample_gray=(c>>2)+(c>>1)+(c>>2);
            end
        end
    endfunction
    function integer horizontal(input integer y,x);
        horizontal=(sample_gray(y,x-2)+2*sample_gray(y,x-1)+
                    sample_gray(y,x))>>2;
    endfunction
    function integer gaussian(input integer y,x);
        gaussian=(horizontal(y-2,x)+2*horizontal(y-1,x)+horizontal(y,x))>>2;
    endfunction
    function integer magnitude(input integer y,x);
        integer gx,gy;
        begin
            gx=gaussian(y-2,x)+2*gaussian(y-1,x)+gaussian(y,x)-
               gaussian(y-2,x-2)-2*gaussian(y-1,x-2)-gaussian(y,x-2);
            gy=gaussian(y,x-2)+2*gaussian(y,x-1)+gaussian(y,x)-
               gaussian(y-2,x-2)-2*gaussian(y-2,x-1)-gaussian(y-2,x);
            magnitude=(gx<0 ? -gx:gx)+(gy<0 ? -gy:gy);
        end
    endfunction

    reg prev_valid=0, prev_sof=0, prev_eol=0;
    reg [95:0] expected_edges=0, expected_gray=0, expected_uniform=0;
    integer in_y=0,in_x=0,scene=0,checked=0,hits=0,lane,c;
    always @(posedge clk) begin
        #1;
        if (!rst_n) begin
            prev_valid=0; prev_sof=0; prev_eol=0;
        end else begin
            if (out_valid!==prev_valid || out_sof!==prev_sof || out_eol!==prev_eol)
                $fatal(1,"Gaussian stream flags");
            if (prev_valid) begin
                if (edges!==expected_edges || gray!==expected_gray)
                    $fatal(1,"Gaussian/gray mismatch: edges %h expected %h",
                           edges,expected_edges);
                for (lane=0;lane<4;lane=lane+1)
                    if (expected_edges[95-lane*24 -: 24]!=0) begin
                        hits=hits+1;
                        if (filled[95-lane*24 -: 24]!==0)
                            $fatal(1,"fill contour is not black");
                    end
                if (scene==0 && filled!==expected_uniform)
                    $fatal(1,"uniform RGB555 fill");
                checked=checked+1;
            end else if (edges!==0 || gray!==0 || filled!==0)
                $fatal(1,"invalid beat not blanked");
            prev_valid=valid; prev_sof=sof; prev_eol=eol;
            expected_edges=0; expected_gray=0; expected_uniform=0;
            if (valid)
                for (lane=0;lane<4;lane=lane+1) begin
                    c=sample_gray(in_y,in_x+lane);
                    expected_gray[95-lane*24 -: 24]={3{c[7:0]}};
                    // Fill alone pads the two rows/columns before its centre
                    // RGB is available; edge/gray expectations are unchanged.
                    if(in_y>=2 && in_x+lane>=2)
                        expected_uniform[95-lane*24 -: 24]=24'h848484;
                    if (in_y>=4 && in_x>=4)
                        expected_edges[95-lane*24 -: 24]=
                            magnitude(in_y,in_x+lane)>=TH ? 24'hffffff:24'd0;
                end
        end
    end

    integer y,x,frame,l,scene_hits;
    initial begin
        repeat(4) @(negedge clk);
        rst_n=1;
        for (frame=0;frame<5;frame=frame+1) begin
            scene=frame; scene_hits=hits;
            for (y=0;y<H;y=y+1)
                for (x=0;x<W;x=x+1)
                    case(frame)
                        0: image[y][x]=128;
                        1: image[y][x]=x<16 ? 32:224;
                        2: image[y][x]=y<6 ? 32:224;
                        3: image[y][x]=y%2 ? 104:96;
                        4: image[y][x]=(x*17+y*31+7)%256;
                    endcase
            for (y=0;y<H;y=y+1)
                for (x=0;x<W;x=x+4) begin
                    @(negedge clk);
                    in_y=y;in_x=x;valid=1;sof=y==0 && x==0;eol=x==W-4;
                    for (l=0;l<4;l=l+1)
                        pixels[95-l*24 -: 24]={3{image[y][x+l][7:0]}};
                    if (frame==4 && (x/4+y)%3==0) begin
                        @(negedge clk);valid=0;sof=0;eol=0;
                    end
                end
            @(negedge clk);valid=0;sof=0;eol=0;
            repeat(3) @(negedge clk);
            if ((frame==0 || frame==3) && hits!=scene_hits)
                $fatal(1,"flat field/alternating-row noise produced contours");
            if ((frame==1 || frame==2) && hits==scene_hits)
                $fatal(1,"horizontal/vertical step not detected");
        end
        // Restart a frame before EOL, with no reset or drain between frames.
        @(negedge clk);valid=1;sof=1;eol=0;in_y=0;in_x=0;
        pixels={{3{24'd0}},24'h202020};
        // Gray reference must match the deliberately changed first word.
        image[0][0]=0;image[0][1]=0;image[0][2]=0;image[0][3]=32;
        @(negedge clk);sof=1;in_x=0;
        @(negedge clk);valid=0;sof=0;
        repeat(3) @(negedge clk);
        $display("PASS: %0d beats, 2-D Gaussian/Sobel reference, noise, contours, gray and stream timing",checked);
        $finish;
    end
    initial begin #200000;$fatal(1,"Gaussian test timeout");end
endmodule
