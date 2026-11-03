`timescale 1ns/1ps
// Independent model for local pair means and fixed, unquantized row seeds.
module tb_fill_local;
    localparam W=16,H=8,T=24;
    reg clk=0;
    always #5 clk=~clk;
    reg rst_n=0,valid=0,sof=0,eol=0;
    reg [95:0] pixels=0;
    // Seed-selection unit fixture; spatial alignment has its own regression.
    reg [95:0] seed_pixels=0;
    reg seed_above=0;
    always @(posedge clk) if(valid) begin
        seed_pixels<=pixels;
        seed_above<=!sof && (dut.y_pos!=0);
    end
    initial begin
        force dut.fill_rgb_word=seed_pixels;
        force dut.fill_valid_mask=4'hf;
        force dut.stage_fill_above=seed_above;
    end
    reg [3:0] contours=0;
    reg [3:0] stage_contours=0;
    always @(posedge clk)
        if(!rst_n) stage_contours<=0;
        else if(valid) stage_contours<=contours;
    wire [95:0] filled;
    wire ov,os,ol;
    edge_detect_96 #(.IMG_WIDTH(W),.IMG_HEIGHT(H)) dut (
        .I_clk(clk),.I_rst_n(rst_n),.I_tdata(pixels),.I_tvalid(valid),
        .I_tuser(sof),.I_tlast(eol),.I_tready(1'b1),
        .O_fill_data(filled),.O_tvalid(ov),.O_tuser(os),.O_tlast(ol));

    function close_ref(input [23:0] a,b);
        integer ch,delta;
        begin
            close_ref=1;
            for(ch=0;ch<3;ch=ch+1) begin
                delta=a[ch*8 +: 8]-b[ch*8 +: 8];
                if(delta<0) delta=-delta;
                if(delta>T) close_ref=0;
            end
        end
    endfunction
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
    function [23:0] average_ref(input [23:0] a,b);
        integer ch,total;
        begin
            for(ch=0;ch<3;ch=ch+1) begin
                total=a[ch*8 +: 8]+b[ch*8 +: 8];
                average_ref[ch*8 +: 8]=total/2;
            end
        end
    endfunction
    function [23:0] quant_ref(input [23:0] a);
        integer ch,code;
        begin
            for(ch=0;ch<3;ch=ch+1) begin
                code=a[ch*8 +: 8]/8;
                quant_ref[ch*8 +: 8]=code*8+code/4;
            end
        end
    endfunction

    reg [23:0] cache[0:W-1],seed=0;
    reg cache_valid[0:W-1],active=0;
    reg [95:0] model_word=0,expected=0;
    reg pv=0,ps=0,pl=0;
    integer checked=0;
    always @(posedge clk) begin
        #1;
        if(!rst_n) begin pv=0;ps=0;pl=0;end
        else begin
            if(ov!==pv || os!==ps || ol!==pl)
                $fatal(1,"local fill stream flags");
            if(pv) begin
                if(filled!==expected)
                    $fatal(1,"local fill got %h expected %h",filled,expected);
                checked=checked+1;
            end else if(filled!==0) $fatal(1,"local fill idle output");
            pv=valid;ps=sof;pl=eol;expected=model_word;
        end
    end

    task model(input integer x,y);
        reg [23:0] p[0:3],local_colour[0:3];
        integer lane,pair_start;
        integer dl,da;
        reg left_ok,above_ok;
        begin
            model_word=0;
            if(x==0 || sof) active=0;
            for(lane=0;lane<4;lane=lane+1)
                p[lane]=pixels[95-lane*24 -: 24];
            for(lane=0;lane<4;lane=lane+1) begin
                pair_start=(lane/2)*2;
                if(!contours[3-pair_start] && !contours[2-pair_start] &&
                   close_ref(p[pair_start],p[pair_start+1]))
                    local_colour[lane]=average_ref(p[pair_start],p[pair_start+1]);
                else local_colour[lane]=p[lane];
                if(contours[3-lane]) begin
                    active=0;cache_valid[x+lane]=0;
                end else begin
                    dl=distance_ref(p[lane],seed);
                    da=distance_ref(p[lane],cache[x+lane]);
                    left_ok=active && dl<=T;
                    above_ok=y>0 && cache_valid[x+lane] && da<=T;
                    if(above_ok && (!left_ok || da+4<=dl))
                        seed=cache[x+lane];
                    else if(!left_ok)
                        seed=local_colour[lane];
                    active=1;cache_valid[x+lane]=1;cache[x+lane]=seed;
                    model_word[95-lane*24 -: 24]=quant_ref(seed);
                end
            end
            if(eol) active=0;
        end
    endtask
    task send(input [95:0] data,input [3:0] mask,input integer x,y);
        begin
            @(negedge clk);
            pixels=data;contours=mask;valid=1;sof=x==0 && y==0;eol=x==W-4;
            model(x,y);
        end
    endtask

    integer a,b,ch,frame,x,y,lane,jitter,rand_state=32'h15c0ffee;
    reg [23:0] pa,pb;
    reg [95:0] data_word;
    reg [3:0] mask_word;
    initial begin
        force dut.fill_edges=stage_contours;
        // All 65,536 input pairs in each RGB channel, including +/-24 and wrap.
        for(ch=0;ch<3;ch=ch+1)
            for(a=0;a<256;a=a+1)
                for(b=0;b<256;b=b+1) begin
                    pa=0;pb=0;pa[ch*8 +: 8]=a;pb[ch*8 +: 8]=b;
                    if(dut.colour_close(pa,pb)!==close_ref(pa,pb))
                        $fatal(1,"colour comparison: channel %0d %0d/%0d",ch,a,b);
                    if(dut.colour_distance(pa,pb)!==distance_ref(pa,pb))
                        $fatal(1,"RGB distance: channel %0d %0d/%0d",ch,a,b);
                end
        repeat(4) @(negedge clk);rst_n=1;
        // A noisy first pixel is averaged; later words retain the same seed.
        send({24'h303030,24'h404040,24'h383838,24'h3c3c3c},0,0,0);
        if(model_word!=={4{24'h393939}}) $fatal(1,"local mean not used");
        send({4{24'h404040}},0,4,0);
        send({4{24'h383838}},0,8,0);
        send({4{24'h3c3c3c}},0,12,0);
        for(y=1;y<H;y=y+1)
            for(x=0;x<W;x=x+4)
                send({4{24'h404040}},0,x,y);
        // Do not average across a contour, including each pair endpoint.
        for(ch=0;ch<4;ch=ch+1) begin
            send({24'h303030,24'h404040,24'h505050,24'h606060},
                 4'b1000>>ch,0,0);
            send({24'hff0000,24'h0000ff,24'h00ff00,24'hffffff},0,4,0);
        end
        // Frame restarts, neighbouring rows, full-rate words and valid bubbles.
        for(frame=0;frame<9;frame=frame+1) begin
            for(y=0;y<H;y=y+1)
                for(x=0;x<W;x=x+4) begin
                    for(lane=0;lane<4;lane=lane+1) begin
                        jitter=$random(rand_state);
                        case(frame)
                            0: data_word[95-lane*24 -: 24]=24'h3f3f3f+y%2*24'h010101;
                            1: data_word[95-lane*24 -: 24]=(x+lane<8) ? 24'hff0000:24'h0000ff;
                            2: data_word[95-lane*24 -: 24]=24'h707070+(jitter&7)*24'h010101;
                            3,4,5,6: begin
                                if(y==0) data_word[95-lane*24 -: 24]=
                                    (x+lane<8 ? 80 : 120)*24'h010101;
                                else data_word[95-lane*24 -: 24]=
                                    (frame==3 ? 100 : frame==4 ? 104 :
                                     frame==5 ? 102 : 101)*24'h010101;
                            end
                            default: data_word[95-lane*24 -: 24]=jitter;
                        endcase
                    end
                    jitter=$random(rand_state);
                    mask_word=frame<7 ? 0:jitter[3:0];
                    send(data_word,mask_word,x,y);
                    if(y==1 && x==8 && (frame==3 || frame==6) &&
                       model_word!=={4{24'h525252}})
                        $fatal(1,"equal/near candidates did not keep left");
                    if(y==1 && x==8 && (frame==4 || frame==5) &&
                       model_word!=={4{24'h7b7b7b}})
                        $fatal(1,"closer above candidate not selected");
                    if(frame==0 && model_word!=={4{24'h393939}})
                        $fatal(1,"cross-row banding reintroduced");
                    if(frame>=7 && ((x/4+y)%3==0)) begin
                        @(negedge clk);valid=0;sof=0;eol=0;
                    end
                end
            @(negedge clk);valid=0;sof=0;eol=0;
            repeat(3) @(negedge clk);
        end
        // Reset in the middle of a frame must not expose an unreset RAM output.
        send({4{24'hff0000}},0,0,0);
        @(negedge clk);rst_n=0;valid=0;sof=0;eol=0;active=0;
        repeat(3) @(negedge clk);rst_n=1;
        send({4{24'h0000ff}},0,0,0);
        @(negedge clk);valid=0;sof=0;eol=0;
        repeat(3) @(negedge clk);
        $display("PASS: 196608 colour comparisons; %0d modelled fill beats, means, contours, row stability and reset",checked);
        $finish;
    end
    initial begin #200000;$fatal(1,"local fill timeout");end
endmodule
