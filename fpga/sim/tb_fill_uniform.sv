`timescale 1ns/1ps
// Fill-only regression. Other modes are checked against the pre-change RTL
// separately; this test supplies contour masks to isolate colour decisions.
module tb_fill_uniform;
    reg clk=0;
    always #5 clk=~clk;
    reg rst_n=0, valid=0, sof=0, eol=0;
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
    wire out_valid, out_sof, out_eol;
    edge_detect_96 #(.IMG_WIDTH(16),.IMG_HEIGHT(8)) dut (
        .I_clk(clk),.I_rst_n(rst_n),.I_tlast(eol),.I_tuser(sof),
        .I_tdata(pixels),.I_tvalid(valid),.I_tready(1'b1),
        .O_tlast(out_eol),.O_tuser(out_sof),.O_fill_data(filled),
        .O_tvalid(out_valid));
    task beat(input [95:0] data, input first, last,
              input [3:0] edges, input [95:0] expected);
        begin
            @(negedge clk);
            pixels=data;sof=first;eol=last;valid=1;contour=edges;
            @(negedge clk);
            valid=0;sof=0;eol=0;
            @(posedge clk);#1;
            if (!out_valid || out_sof!==first || out_eol!==last)
                $fatal(1,"fill stream flags");
            if (filled!==expected)
                $fatal(1,"fill mismatch: got %h expected %h",filled,expected);
            @(posedge clk);#1;
            if (out_valid || filled!==0) $fatal(1,"invalid beat not blanked");
        end
    endtask
    integer lane;
    initial begin
        force dut.fill_edges=contour;
        repeat(3) @(negedge clk);rst_n=1;
        // Similar pixels straddling the old quantization boundary share a seed.
        beat({24'h3f3f3f,24'h404040,24'h3b3b3b,24'h454545},1,0,0,{4{24'h393939}});
        // Carry the same seed over an idle gap and a four-pixel word boundary.
        beat({24'h4a4a4a,24'h505050,24'h555555,24'h565656},0,0,0,{4{24'h393939}});
        // +24 stays in the old region; +25 starts from the local pair mean.
        beat({24'h575757,24'h585858,24'h666666,24'h666666},0,1,0,
             {24'h393939,24'h525252,24'h525252,24'h525252});
        // A new row starts from its own colour, not the previous row's seed.
        beat({4{24'h737373}},0,1,0,{4{24'h737373}});
        // Opposite colours remain distinct rather than being averaged together.
        beat({24'hff0000,24'h0000ff,24'h00ff00,24'hffffff},1,1,0,
             {24'hff0000,24'h0000ff,24'h00ff00,24'hffffff});
        // Exercise every contour lane; black ends the region.
        for(lane=0;lane<4;lane=lane+1)
            beat({4{24'h00ff00}},0,1,4'b1000>>lane,
                 {4{24'h00ff00}} & ~(96'hffffff000000000000000000 >> (lane*24)));
        beat({4{24'h00ff00}},0,1,4'b1111,96'd0);
        // Nearby shades must choose a new seed after a contour, including
        // when the contour is the last lane of the preceding word.
        beat({24'h3f3f3f,24'h404040,24'h414141,24'h424242},1,0,4'b0100,
             {24'h393939,24'd0,24'h424242,24'h424242});
        beat({4{24'h3f3f3f}},0,0,4'b0001,
             {24'h424242,24'h424242,24'h424242,24'd0});
        beat({4{24'h3f3f3f}},0,1,0,{4{24'h393939}});
        // Frame start clears a carried seed even without a preceding EOL.
        beat({4{24'h3f3f3f}},1,0,0,{4{24'h393939}});
        beat({4{24'h404040}},1,1,0,{4{24'h424242}});
        // RGB subtraction must not wrap at black/white extremes.
        beat({24'h000000,24'hffffff,24'h000000,24'hffffff},1,1,0,
             {24'h000000,24'hffffff,24'h000000,24'hffffff});
        $display("PASS: uniform fill, seed bounds, lane boundaries, contours, row/frame reset and idle timing");
        $finish;
    end
    initial begin #200000;$fatal(1,"fill test timeout");end
endmodule
