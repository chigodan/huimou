`timescale 1ns/1ps
// Integration with the actual 96-to-128 packer: SOF is intentionally one
// cycle earlier than the first valid packed word, as in the board project.
module tb_realtime_packer_tag;
    reg clk=0,rst=0,sof=0,valid=0;
    always #5 clk=~clk;
    wire packed_sof,packed_valid;
    wire [127:0] packed_data;
    data_96bit_to_128bit packer(
        .I_clk(clk),.I_rst_n(rst),.I_96b_frame_start(sof),.I_96b_valid(valid),
        .I_96b_data(96'h123456123456123456123456),
        .O_128b_frame_start(packed_sof),.O_128b_valid(packed_valid),.O_128b_data(packed_data));
    video_realtime_monitor #(.REF_CLK_HZ(1_000_000),.FRAME_WORDS(3)) dut(
        .I_rst_n(rst),.I_ref_clk(clk),.I_camera_clk(clk),.I_ddr_clk(clk),.I_pixel_clk(clk),
        .I_capture_start(sof && valid),.I_processed_start(packed_sof),
        .I_write_frame_start(1'b0),.I_write_slot(2'd0),.I_write_word(1'b0),.I_write_addr(25'd0),
        .I_read_frame_start(1'b0),.I_read_slot(2'd0),.I_display_start(1'b0),
        .O_capture_fps(),.O_display_fps(),.O_new_fps(),.O_capture_id(),.O_display_id(),
        .O_latency_ms(),.O_latency_valid(),.O_video_state());
    integer words=0;
    reg saw_early_sof=0;
    always @(posedge clk) if(rst) begin
        if(packed_sof && !packed_valid) saw_early_sof<=1;
        if(packed_valid) words<=words+1;
    end
    task image;
        begin
            @(negedge clk);sof=1;valid=1;
            @(negedge clk);sof=0;
            repeat(3) @(negedge clk);
            valid=0;repeat(5) @(negedge clk);
        end
    endtask
    initial begin
        repeat(5) @(negedge clk);rst=1;
        image();
        if(!saw_early_sof || words!=3 || !dut.write_tag_valid || dut.write_tag[63:32]!=1 || dut.tag_count!=0 || dut.tag_fault)
            $fatal(1,"packed-frame SOF/tag alignment failed");
        image();
        if(words!=6 || dut.write_tag[63:32]!=2 || dut.tag_count!=0 || dut.tag_fault)
            $fatal(1,"successive packed frames lost or duplicated metadata");
        $display("PASS: actual packer early SOF consumes exactly one capture tag per frame without valid qualification");
        $finish;
    end
    initial begin #100000; $fatal(1,"packer tag test timeout"); end
endmodule
