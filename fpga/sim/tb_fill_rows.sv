`timescale 1ns/1ps
module tb_fill_rows;
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
    reg [3:0] contour=0;
    wire [95:0] filled;
    wire out_valid,out_sof,out_eol;
    edge_detect_96 #(.IMG_WIDTH(16),.IMG_HEIGHT(8)) dut (
        .I_clk(clk),.I_rst_n(rst_n),.I_tdata(pixels),.I_tvalid(valid),
        .I_tuser(sof),.I_tlast(eol),.I_tready(1'b1),.O_fill_data(filled),
        .O_tvalid(out_valid),.O_tuser(out_sof),.O_tlast(out_eol));
    task beat(input [95:0] data,input first,last,
              input [3:0] edges,input [95:0] expected);
        begin
            @(negedge clk);
            pixels=data;sof=first;eol=last;valid=1;contour=edges;
            @(negedge clk);valid=0;sof=0;eol=0;
            @(posedge clk);#1;
            if(!out_valid || out_sof!==first || out_eol!==last || filled!==expected)
                $fatal(1,"row fill: got %h expected %h",filled,expected);
            @(posedge clk);#1;
            if(out_valid || filled!==0) $fatal(1,"idle timing");
        end
    endtask
    task row(input [23:0] colour,expected,input first,input [3:0] edges);
        integer group;
        begin
            for(group=0;group<4;group=group+1)
                beat({4{colour}},first && group==0,group==3,edges,{4{expected}});
        end
    endtask
    initial begin
        force dut.fill_edges=contour;
        repeat(4) @(negedge clk);rst_n=1;
        // Adjacent rows cross a quantization boundary but stay one colour.
        row(24'h3f3f3f,24'h393939,1,0);
        row(24'h404040,24'h393939,0,0);
        row(24'h3e3e3e,24'h393939,0,0);
        row(24'h575757,24'h393939,0,0); // seed +24: inclusive
        row(24'h585858,24'h5a5a5a,0,0); // seed +25: new region
        row(24'h575757,24'h5a5a5a,0,0);
        row(24'hff0000,24'hff0000,0,0); // protect a real colour boundary
        row(24'h0000ff,24'h0000ff,0,0);
        // A new frame must not inherit the old frame's cached seeds.
        row(24'h404040,24'h424242,1,0);
        row(24'h3f3f3f,24'h424242,0,0);
        row(24'h3f3f3f,24'h000000,0,4'b1111);
        row(24'h3f3f3f,24'h393939,0,0); // no seed crosses an all-edge row
        // Above seeds are indexed per lane. Also test implicit row wrapping:
        // the first row ends at IMG_WIDTH even though EOL is not asserted.
        beat({24'h3f0000,24'h003f00,24'h00003f,24'h3f3f3f},1,0,0,
             {24'h390000,24'h003900,24'h000039,24'h393939});
        repeat(3)
            beat({24'h3f0000,24'h003f00,24'h00003f,24'h3f3f3f},0,0,0,
                 {24'h390000,24'h003900,24'h000039,24'h393939});
        repeat(3)
            beat({24'h400000,24'h004000,24'h000040,24'h404040},0,0,0,
                 {24'h390000,24'h003900,24'h000039,24'h393939});
        beat({24'h400000,24'h004000,24'h000040,24'h404040},0,1,0,
             {24'h390000,24'h003900,24'h000039,24'h393939});
        $display("PASS: cross-row colour stability, bounded drift, real boundaries, contours, frame reset and all lanes");
        $finish;
    end
    initial begin #200000;$fatal(1,"row fill timeout");end
endmodule
