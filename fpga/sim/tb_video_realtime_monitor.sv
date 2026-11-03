`timescale 1ns/1ps
// Four asynchronous clocks; small frames but real ms/FPS arithmetic.
module tb_video_realtime_monitor;
    reg ref_clk=0, camera_clk=0, ddr_clk=0, pixel_clk=0, rst=0;
    always #5 ref_clk=~ref_clk;
    always #7 camera_clk=~camera_clk;
    always #3 ddr_clk=~ddr_clk;
    always #11 pixel_clk=~pixel_clk;
    reg cap=0, processed=0, wr_start=0, wr_word=0, rd_start=0, displayed=0;
    reg [1:0] wr_slot=0, rd_slot=0;
    reg [24:0] wr_addr=0;
    wire [9:0] cap_fps, out_fps, new_fps;
    wire [15:0] cap_id, display_id, latency;
    wire latency_valid;
    wire [1:0] state;
    video_realtime_monitor #(.REF_CLK_HZ(1_000_000),.FRAME_WORDS(4),
        .FPS_WINDOW_MS(1000),.STALE_MS(50)) dut(
        .I_rst_n(rst),.I_ref_clk(ref_clk),.I_camera_clk(camera_clk),
        .I_ddr_clk(ddr_clk),.I_pixel_clk(pixel_clk),
        .I_capture_start(cap),.I_processed_start(processed),
        .I_write_frame_start(wr_start),.I_write_slot(wr_slot),
        .I_write_word(wr_word),.I_write_addr(wr_addr),
        .I_read_frame_start(rd_start),.I_read_slot(rd_slot),.I_display_start(displayed),
        .O_capture_fps(cap_fps),.O_display_fps(out_fps),.O_new_fps(new_fps),
        .O_capture_id(cap_id),.O_display_id(display_id),.O_latency_ms(latency),
        .O_latency_valid(latency_valid),.O_video_state(state));
    integer ref_cycles=0;
    always @(posedge ref_clk) if(!rst) ref_cycles<=0; else ref_cycles<=ref_cycles+1;
    function [24:0] base_addr(input integer index);
        begin base_addr=index*7000000; end
    endfunction
    task reset_all;
        begin
            @(negedge ref_clk); rst=0; cap=0;processed=0;wr_start=0;wr_word=0;rd_start=0;displayed=0;
            repeat(8) @(negedge ref_clk);rst=1;
            repeat(8) @(negedge ref_clk);
        end
    endtask
    task wait_ms(input integer count);
        begin repeat(count*1000) @(negedge ref_clk); end
    endtask
    task capture;
        begin @(negedge camera_clk);cap=1;@(negedge camera_clk);cap=0; end
    endtask
    task process_frame;
        begin
            @(negedge camera_clk);processed=1;@(negedge camera_clk);processed=0;
            repeat(8) @(negedge camera_clk); // Matches video_in's settled SOF tag.
        end
    endtask
    task start_write(input integer index);
        begin
            @(negedge ddr_clk);wr_slot=index;wr_start=1;
            @(negedge ddr_clk);wr_start=0;wr_addr=base_addr(index);
        end
    endtask
    task words(input integer count);
        integer w;
        begin
            for(w=0;w<count;w=w+1) begin
                @(negedge ddr_clk);wr_word=1;
                @(negedge ddr_clk);wr_word=0;wr_addr=wr_addr+8;
            end
        end
    endtask
    task show(input integer index);
        begin
            @(negedge ddr_clk);rd_slot=index;rd_start=1;
            @(negedge ddr_clk);rd_start=0;
            repeat(12) @(negedge pixel_clk);
            displayed=1;@(negedge pixel_clk);displayed=0;
        end
    endtask
    task automatic at_cycle(input integer target);
        begin while(ref_cycles<target) @(negedge ref_clk); end
    endtask
    integer first_latency, n_source, n_output, latest_slot;
    initial begin
        reset_all();
        if(latency_valid || state!=0) $fatal(1,"startup must be WAIT/NA");
        wait_ms(2); capture(); wait_ms(3); process_frame(); start_write(0);words(4);
        wait_ms(3); show(0);
        if(!latency_valid || display_id!=1 || latency<5 || latency>7)
            $fatal(1,"capture-to-display measurement failed: id=%0d latency=%0d valid=%b",display_id,latency,latency_valid);
        first_latency=latency;
        wait_ms(2);show(0);
        if(!latency_valid || latency!=first_latency+2 || dut.new_count!=1 || dut.display_count!=2)
            $fatal(1,"repeated frames must age and must not count as fresh updates");
        show(3);
        if(latency_valid) $fatal(1,"unwritten DDR slot must not report a latency");
        $display("PASS: real frame association, known latency, repeated-frame age and refresh/update distinction");

        capture();process_frame();start_write(1);words(3);show(1);
        if(latency_valid) $fatal(1,"incomplete DDR frame must report NA");
        words(1);show(1);
        if(!latency_valid || display_id!=2) $fatal(1,"completed frame did not become measurable");
        capture();wait_ms(1);capture(); // Two captures queue before either output SOF.
        process_frame();start_write(2);words(4);
        process_frame();start_write(3);words(4);
        show(2);
        if(!latency_valid || display_id!=3) $fatal(1,"metadata queue associated wrong source frame");
        show(3);
        if(!latency_valid || display_id!=4) $fatal(1,"metadata queue order failed");
        if(dut.tag_fault) $fatal(1,"normal pipeline generated metadata fault");
        wait_ms(55);show(3);wait_ms(2);
        if(latency_valid || state!=2) $fatal(1,"stopped camera must become STALE with NA latency");
        $display("PASS: complete-frame gating, queued ISP latency, four buffer slots and source-loss timeout");

        reset_all();
        @(negedge ref_clk);
        dut.time_ms=32'hfffffffc;dut.time_gray=32'hfffffffc^(32'hfffffffc>>1);dut.tick_div=0;
        wait_ms(1);capture();process_frame();start_write(0);words(4);
        wait_ms(6);show(0);
        if(!latency_valid || latency<5 || latency>7) $fatal(1,"timestamp wrap corrupted latency");
        reset_all();
        repeat(5) capture();wait_ms(2);
        if(!dut.tag_fault || state!=3) $fatal(1,"metadata overflow must report TAG ERROR");
        $display("PASS: timestamp wrap, queue-overflow detection and invalid-measurement handling");

        reset_all();latest_slot=0;
        fork
            begin
                for(n_source=0;n_source<50;n_source=n_source+1) begin
                    at_cycle(5000+n_source*20000);
                    capture();process_frame();start_write(n_source%4);words(4);
                    latest_slot=n_source%4;
                end
            end
            begin
                for(n_output=0;n_output<100;n_output=n_output+1) begin
                    at_cycle(7500+n_output*10000);show(latest_slot);
                end
            end
        join
        at_cycle(1002000);repeat(12) @(negedge pixel_clk);
        if(cap_fps!=50 || out_fps!=100 || new_fps!=50)
            $fatal(1,"measured FPS wrong: capture=%0d output=%0d fresh=%0d",cap_fps,out_fps,new_fps);
        if(state!=1 || !latency_valid)
            $fatal(1,"healthy pipeline state=%0d valid=%b live=%b fault=%b age=%0d id=%0d",state,latency_valid,dut.video_live,dut.video_fault,dut.capture_age,display_id);
        $display("PASS: one-second measurement: 50 capture FPS / 100 refresh FPS / 50 fresh-frame FPS");
        $finish;
    end
    initial begin #15000000; $fatal(1,"real-time monitor test timeout"); end
endmodule
