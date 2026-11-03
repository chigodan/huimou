`timescale 1ns/1ps
module tb_circle_profile_validator;
    reg clk=0; always #5 clk=~clk;
    reg rst=0, frame=0, wr=0, req=0;
    reg [11:0] row=0, l=0, r=0, xmin=0, xmax=0, height=0;
    wire done, ok;
    circle_profile_validator dut(.I_clk(clk),.I_rst_n(rst),.I_frame_start(frame),
        .I_write(wr),.I_write_slot(3'd0),.I_write_row(row),.I_left(l),.I_right(r),
        .I_req(req),.I_slot(3'd0),.I_xmin(xmin),.I_xmax(xmax),.I_height(height),
        .O_done(done),.O_ok(ok));
    // Shape 0 circle, 1 axis-aligned ellipse, 2 rotated ellipse, 3 clipped
    // square (~78% fill), 4 cross (~78% fill). Every profile has equal bbox
    // scale, so area/aspect-only tests cannot distinguish the latter cases.
    task verify(input integer rx,input integer ry,input integer shape,input bit expected);
        integer x,y,left,right,rows,wmin,wmax,n,cyc;
        bit hit;
        longint a,b,u,v,xx,yy;
        begin
            rows=0; wmin=4095; wmax=0;
            for(y=-ry;y<=ry;y=y+1) begin
                left=4095; right=-4095;
                for(x=-rx;x<=rx;x=x+1) begin
                    xx=x; yy=y; a=rx; b=ry;
                    if(shape==0 || shape==1) hit=(xx*xx*b*b+yy*yy*a*a <= a*a*b*b);
                    else if(shape==2) begin
                        b=rx*87/100; u=xx+yy; v=yy-xx;
                        hit=(u*u*b*b+v*v*a*a <= 2*a*a*b*b);
                    end else if(shape==3) hit=(1000*(x<0?-x:x)+1000*(y<0?-y:y) <= 1368*rx);
                    else hit=((x<0?-x:x)*100<55*rx || (y<0?-y:y)*100<55*ry);
                    if(hit) begin if(x<left) left=x; if(x>right) right=x; end
                end
                if((right-left+1>=6) || ((rows>0) && right>=left)) begin
                    @(negedge clk); wr=1; row=rows; l=1024+left; r=1024+right;
                    if(l<wmin) wmin=l;
                    if(r>wmax) wmax=r;
                    rows=rows+1;
                end
            end
            @(negedge clk); wr=0; xmin=wmin; xmax=wmax; height=rows; req=1;
            @(negedge clk); req=0;
            cyc=0;
            while(!done && cyc<rows*10+50) begin @(negedge clk); cyc=cyc+1; end
            if(!done || ok!==expected)
                $fatal(1,"profile shape=%d radius=%d,%d expected=%d got=%d done=%d",shape,rx,ry,expected,ok,done);
            $display("PASS: profile shape=%d radius=%d,%d result=%d cycles=%d",shape,rx,ry,ok,cyc);
            repeat(3) @(negedge clk);
        end
    endtask
    initial begin
        repeat(4) @(negedge clk); rst=1;
        verify(10,10,0,1); verify(15,15,0,1); verify(40,40,0,1);
        verify(160,160,0,1); verify(320,320,0,1);
        verify(40,35,1,1); verify(35,40,1,1);
        verify(40,40,2,1); verify(160,160,2,1);
        verify(40,40,3,0); verify(160,160,3,0);
        verify(15,15,4,0); verify(80,80,4,0);
        $finish;
    end
    initial begin #1000000; $fatal(1,"profile timeout"); end
endmodule
