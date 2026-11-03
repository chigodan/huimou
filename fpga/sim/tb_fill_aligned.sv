`timescale 1ns/1ps
// End-to-end colour history + independent seed model at the actual row width.
// Contours are supplied in the Sobel-centre coordinate system; the existing
// Gaussian regression independently checks the real edge calculation.
module tb_fill_aligned;
    localparam W=1280,H=8,T=24;
    reg clk=0,rst_n=0,valid=0,sof=0,eol=0;
    always #5 clk=~clk;
    reg [95:0] pixels=0;
    reg [3:0] contours=0,stage_contours=0;
    always @(posedge clk)
        if(!rst_n) stage_contours<=0;
        else if(valid) stage_contours<=contours;
    wire [95:0] filled;
    wire ov,os,ol;
    edge_detect_96 #(.IMG_WIDTH(W),.IMG_HEIGHT(H)) dut (
        .I_clk(clk),.I_rst_n(rst_n),.I_tdata(pixels),.I_tvalid(valid),
        .I_tuser(sof),.I_tlast(eol),.I_tready(1'b1),.O_fill_data(filled),
        .O_tvalid(ov),.O_tuser(os),.O_tlast(ol));
    function integer distance_ref(input [23:0] a,b);
        integer ch,d;
        begin
            distance_ref=0;
            for(ch=0;ch<3;ch=ch+1) begin
                d=integer'(a[ch*8 +: 8])-integer'(b[ch*8 +: 8]);
                if(d<0) d=-d;
                if(d>distance_ref) distance_ref=d;
            end
        end
    endfunction
    function [23:0] mean_ref(input [23:0] a,b);
        integer ch,n;
        begin
            for(ch=0;ch<3;ch=ch+1) begin
                n=integer'(a[ch*8 +: 8])+integer'(b[ch*8 +: 8]);
                mean_ref[ch*8 +: 8]=n/2;
            end
        end
    endfunction
    function [23:0] quant_ref(input [23:0] a);
        integer ch,n;
        begin
            for(ch=0;ch<3;ch=ch+1) begin
                n=a[ch*8 +: 8]/8;
                quant_ref[ch*8 +: 8]=n*8+n/4;
            end
        end
    endfunction
    reg [23:0] image[0:H-1][0:W-1],cache[0:W-1],seed=0;
    reg cache_valid[0:W-1],active=0;
    reg [95:0] model_word=0,expected=0,raw_expected=0;
    reg [3:0] mask_expected=0;
    reg pv=0,ps=0,pl=0;
    integer checked=0,raw_checked=0;
    integer check_lane;
    always @(posedge clk) begin
        #1;
        if(!rst_n) begin pv=0;ps=0;pl=0;active=0;end
        else begin
            if({ov,os,ol}!=={pv,ps,pl}) $fatal(1,"aligned stream flags");
            if(pv) begin
                if(filled!==expected)
                    $fatal(1,"aligned fill got %h expected %h beat=%0d",filled,expected,checked);
                checked=checked+1;
            end else if(filled!==0) $fatal(1,"aligned idle output");
            if(valid) begin
                if(dut.fill_valid_mask!==mask_expected) $fatal(1,"warm-up/border validity");
                for(check_lane=0;check_lane<4;check_lane=check_lane+1)
                    if(mask_expected[3-check_lane]) begin
                        if(dut.fill_rgb_word[95-check_lane*24 -: 24] !==
                           raw_expected[95-check_lane*24 -: 24])
                            $fatal(1,"RGB not at Sobel centre lane=%0d",check_lane);
                        raw_checked=raw_checked+1;
                    end
            end
            pv=valid;ps=sof;pl=eol;expected=model_word;
        end
    end
    task model(input integer x,y);
        reg [23:0] p[0:3],candidate;
        reg left_ok,above_ok;
        integer lane,pair_start,dl,da;
        begin
            model_word=0;raw_expected=0;
            mask_expected=y<2 ? 0 : x==0 ? 4'b0011 : 4'b1111;
            if(x==0) active=0;
            for(lane=0;lane<4;lane=lane+1) begin
                p[lane]=(y<2 || x+lane<2) ? 0 : image[y-2][x+lane-2];
                raw_expected[95-lane*24 -: 24]=p[lane];
            end
            for(lane=0;lane<4;lane=lane+1) begin
                pair_start=lane/2*2;
                candidate=p[lane];
                if(!contours[3-pair_start] && !contours[2-pair_start] &&
                   distance_ref(p[pair_start],p[pair_start+1])<=T)
                    candidate=mean_ref(p[pair_start],p[pair_start+1]);
                if(!mask_expected[3-lane] || contours[3-lane]) begin
                    active=0;cache_valid[x+lane]=0;
                end else begin
                    dl=distance_ref(p[lane],seed);da=distance_ref(p[lane],cache[x+lane]);
                    left_ok=active && dl<=T;
                    above_ok=y>2 && cache_valid[x+lane] && da<=T;
                    if(above_ok && (!left_ok || da+4<=dl)) seed=cache[x+lane];
                    else if(!left_ok) seed=candidate;
                    active=1;cache[x+lane]=seed;cache_valid[x+lane]=1;
                    model_word[95-lane*24 -: 24]=quant_ref(seed);
                end
            end
        end
    endtask
    integer frame,x,y,lane,yy,xx,rand_state=32'h7f181302,r;
    task send(input integer sx,sy,input integer pattern);
        integer k;
        begin
            @(negedge clk);
            valid=1;sof=(sx==0 && sy==0);
            // Also cover implicit IMG_WIDTH wrapping without EOL.
            eol=(sx==W-4 && pattern!=2);
            contours=0;
            for(k=0;k<4;k=k+1) begin
                pixels[95-k*24 -: 24]=image[sy][sx+k];
                if(pattern==0) contours[3-k]=(sy>=2 && ((sx+k-2)%97==20 || sy==5));
                else if(pattern==3) contours[3-k]=($random(rand_state)&15)==0;
            end
            model(sx,sy);
        end
    endtask
    initial begin
        force dut.fill_edges=stage_contours;
        repeat(4) @(negedge clk);rst_n=1;
        for(frame=0;frame<5;frame=frame+1) begin
            for(yy=0;yy<H;yy=yy+1) for(xx=0;xx<W;xx=xx+1) begin
                r=$random(rand_state);
                case(frame)
                    0: image[yy][xx]=(xx%97<40 ? 24'hb02010 : 24'h10a0c0);
                    1: image[yy][xx]={8'(xx/8+yy*3),8'(xx/7+yy*5),8'(xx/9+yy*7)};
                    2: image[yy][xx]=(yy==0 ? (xx%32<16 ? 80 : 120) : 100)*24'h010101;
                    3: image[yy][xx]=r;
                    4: image[yy][xx]=24'h0010e0;
                endcase
            end
            for(y=0;y<H;y=y+1) for(x=0;x<W;x=x+4) begin
                send(x,y,frame);
                if(frame==1 && x%68==0) begin
                    @(negedge clk);valid=0;sof=0;eol=0;
                    repeat(2) @(negedge clk);
                end
                if(frame==3 && y==4 && x==140) begin
                    @(negedge clk);rst_n=0;valid=0;sof=0;eol=0;
                    repeat(3) @(negedge clk);rst_n=1;
                    // Restart this frame from SOF with a different colour.
                    for(yy=0;yy<H;yy=yy+1) for(xx=0;xx<W;xx=xx+1)
                        image[yy][xx]=24'he01020;
                    y=0;x=-4;
                    frame=4;
                end
            end
        end
        @(negedge clk);valid=0;sof=0;eol=0;
        repeat(4) @(negedge clk);
        $display("PASS aligned fill: %0d beats, %0d centre RGB pixels, continuous/bubbles, implicit rows, SOF/reset and borders",checked,raw_checked);
        $finish;
    end
    initial begin #1000000;$fatal(1,"aligned fill timeout");end
endmodule
