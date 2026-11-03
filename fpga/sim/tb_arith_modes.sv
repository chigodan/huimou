`timescale 1ns/1ps
// Unit regression: upstream camera/AWB are interface stubs.  The mode mux
// and image-processing module under test are the actual project RTL.
module tb_arith_modes;
    reg clk=0; always #5 clk=~clk;
    reg rst=0, sof=0;
    reg [1:0] mode=0;
    isp_top dut(.axi4s_video_aclk(clk),.I_rst_n(rst),.I_mode(mode),
        .I_tlast(1'b0),.I_tuser(sof),.I_tdata(40'd0),.I_tvalid(1'b0),
        .I_tdest(10'd0),.O_tready(1'b1),.O_tdata(),.O_tlast(),
        .O_tuser(),.O_tvalid(),.I_tready());
    reg ev=0, eu=0, el=0;
    reg [95:0] pixels=0;
    reg [95:0] seed_pixels=0;
    always @(posedge clk) if(ev) seed_pixels<=pixels;
    wire [95:0] edge_data, gray_data, fill_data;
    wire ov, ou, ol;
    edge_detect_96 #(.IMG_WIDTH(16),.IMG_HEIGHT(8)) effects(
        .I_clk(clk),.I_rst_n(rst),.I_tlast(el),.I_tuser(eu),
        .I_tdata(pixels),.I_tvalid(ev),.I_tready(1'b1),
        .O_tdata(edge_data),.O_gray_data(gray_data),.O_fill_data(fill_data),
        .O_tvalid(ov),.O_tuser(ou),.O_tlast(ol),.O_tready());
    task frame(input [1:0] requested);
        begin
            @(negedge clk); mode=requested;
            repeat(4) @(negedge clk);
            sof=1; @(negedge clk); sof=0;
        end
    endtask
    task check_mode(input [1:0] expected_mode, input [1:0] expected_effect);
        begin
            if(dut.mode_latched !== expected_mode ||
               dut.effect_latched !== expected_effect)
                $fatal(1,"mode/effect mismatch: %b/%d",dut.mode_latched,dut.effect_latched);
            if(expected_mode != 2'b01 && dut.selected_tdata !== {4{24'h123456}})
                $fatal(1,"camera/OSD/circle must receive raw RGB");
            if(expected_mode == 2'b01) begin
                case(expected_effect)
                    0: if(dut.selected_tdata !== {4{24'hffffff}}) $fatal(1,"edge mux");
                    1: if(dut.selected_tdata !== {4{24'h777777}}) $fatal(1,"gray mux");
                    2: if(dut.selected_tdata !== {4{24'h112233}}) $fatal(1,"fill mux");
                endcase
            end
        end
    endtask
    task beat(input [95:0] p, input first, input last);
        begin
            @(negedge clk); pixels=p; eu=first; el=last; ev=1;
            // The edge core now has a registered line-memory stage.
            @(posedge clk); #1;
            @(negedge clk); ev=0; eu=0; el=0;
            @(posedge clk); #1;
            if(ov !== 1 || ou !== first || ol !== last)
                $fatal(1,"effects lost valid/SOF/EOL alignment");
        end
    endtask
    integer y,x,white_edges;
    initial begin
        force dut.camera_O_tdata={4{24'h123456}};
        force dut.edge_O_tdata={4{24'hffffff}};
        force dut.gray_O_tdata={4{24'h777777}};
        force dut.fill_O_tdata={4{24'h112233}};
        repeat(4) @(negedge clk); rst=1;
        frame(0); check_mode(0,0);
        frame(1); check_mode(1,0);
        frame(1); check_mode(1,0); // staying in 01 does not auto-cycle
        frame(0); frame(0); frame(1); check_mode(1,1);
        frame(2); check_mode(2,1);
        frame(3); check_mode(3,1);
        frame(1); check_mode(1,1); // return from circle does not cycle
        frame(0); frame(1); check_mode(1,1); // short 00 pulse does not cycle
        frame(0); frame(0); frame(1); check_mode(1,2);
        frame(0); frame(0); frame(1); check_mode(1,0);
        $display("PASS: original camera/OSD/circle modes and edge-gray-fill cycling");
        // Known RGB values exercise all four gray lanes independently.
        beat({24'hff0000,24'h00ff00,24'h0000ff,24'hffffff},1,0);
        if(gray_data !== {24'h3f3f3f,24'h7f7f7f,24'h3f3f3f,24'hfdfdfd})
            $fatal(1,"RGB-to-gray mismatch: %h",gray_data);
        // Fill is locally quantized; it must not propagate one seed colour
        // across later pixels when no edge is present.
        force effects.edge0=0; force effects.edge1=0;
        // Isolate seed decisions here; tb_fill_aligned checks real RGB history.
        force effects.fill_rgb_word=seed_pixels;
        force effects.fill_valid_mask=4'hf;
        beat({4{24'hff0000}},1,0);
        if(fill_data !== {4{24'hff0000}}) $fatal(1,"fill seed");
        beat({4{24'h0000ff}},0,0);
        if(fill_data !== {4{24'h0000ff}}) $fatal(1,"fill colour locality");
        force effects.edge0=1;
        beat({4{24'h00ff00}},0,0);
        if(fill_data !== {24'd0,24'h00ff00,24'h00ff00,24'h00ff00}) $fatal(1,"fill boundary split");
        force effects.edge0=0;
        beat({4{24'h0000ff}},0,1);
        if(fill_data !== {4{24'h0000ff}}) $fatal(1,"new region not retained");
        release effects.edge0; release effects.edge1;
        release effects.fill_rgb_word; release effects.fill_valid_mask;
        @(negedge clk); ev=0; eu=0; el=0;
        @(posedge clk); #1;
        if(ov || gray_data !== 0 || fill_data !== 0) $fatal(1,"invalid beat not blanked");
        @(negedge clk); rst=0; repeat(2) @(negedge clk); rst=1;
        white_edges=0;
        for(y=0;y<8;y=y+1) begin
            for(x=0;x<4;x=x+1) begin
                beat(x<2 ? 96'd0 : {4{24'hffffff}},y==0 && x==0,x==3);
                if(y<4 && edge_data !== 0) $fatal(1,"uninitialized line memory leaked");
                if(y>=4 && edge_data != 0) white_edges=white_edges+1;
            end
            @(negedge clk); ev=0; eu=0; el=0;
        end
        if(white_edges==0) $fatal(1,"Sobel failed to detect vertical step");
        $display("PASS: grayscale, scanline region fill, Sobel and stream timing");
        $finish;
    end
    initial begin #200000; $fatal(1,"test timeout"); end
endmodule

module demosaic #(parameter IMG_HEIGHT=720,IMG_WIDTH=1280,
    data_complete_delay=50,parameter BAYER_MODE="BGGR")(
    input I_clk,I_rst_n,input [39:0] axi4s_video_tdata,
    input [9:0] axi4s_video_tdest,input axi4s_video_tlast,
    axi4s_video_tvalid,axi4s_video_tuser,O_tready,
    output O_tlast,O_tuser,O_tvalid,axi4s_video_tready,output [127:0] O_tdata);
    assign O_tlast=axi4s_video_tlast; assign O_tuser=axi4s_video_tuser;
    assign O_tvalid=axi4s_video_tvalid; assign axi4s_video_tready=O_tready;
    assign O_tdata=128'd0;
endmodule
module awb #(parameter IMG_HEIGHT=720,IMG_WIDTH=1280)(
    input I_clk,I_rst_n,I_tlast,I_tuser,input [95:0] I_tdata,
    input I_tvalid,O_tready,output O_tlast,O_tuser,O_tvalid,I_tready,
    output [95:0] O_tdata);
    assign O_tlast=I_tlast; assign O_tuser=I_tuser;
    assign O_tvalid=I_tvalid; assign I_tready=O_tready;
    assign O_tdata=I_tdata;
endmodule
