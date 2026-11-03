`timescale 1ns/1ps

// Divider/control tests use deposited completed-frame statistics to cover
// arithmetic boundaries that would otherwise require billions of pixels.
module tb_awb_shared_frame;
    reg clk = 0;
    always #5 clk = ~clk;
    reg rst_n = 0, sof = 0;
    wire ready;
    awb #(.IMG_WIDTH(320), .IMG_HEIGHT(3)) dut (
        .I_clk(clk), .I_rst_n(rst_n), .I_tlast(1'b0), .I_tuser(sof),
        .I_tdata(96'd0), .I_tvalid(1'b0), .I_tready(ready),
        .O_tlast(), .O_tuser(), .O_tdata(), .O_tvalid(), .O_tready(1'b1)
    );

    function [19:0] gain(input [31:0] g, input [31:0] d);
        reg [63:0] q;
        begin
            q = ({32'd0, g} << 16) / ((d == 0) ? 32'd1 : d);
            gain = (q > 64'hfffff) ? 20'hfffff : q[19:0];
        end
    endfunction

    reg [19:0] pending_r = 20'h10000, pending_b = 20'h10000;
    reg [19:0] active_r = 20'h10000, active_b = 20'h10000;
    integer cases = 0;
    reg [31:0] seed = 32'h019731ab;
    function [31:0] random_word(input [31:0] x);
        reg [31:0] y;
        begin
            y = x ^ (x << 13);
            y = y ^ (y >> 17);
            random_word = y ^ (y << 5);
        end
    endfunction

    task exercise(input [31:0] r, input [31:0] g, input [31:0] b,
                  input integer busy_sof);
        integer n;
        begin
            @(negedge clk);
            dut.sum_r[0] = r;
            dut.sum_g[0] = g;
            dut.sum_b[0] = b;
            sof = 1;
            active_r = pending_r;
            active_b = pending_b;
            @(posedge clk); #1;
            if (dut.in_r_r1 !== active_r || dut.in_b_r1 !== active_b)
                $fatal(1, "Gains did not commit together at SOF");
            if (dut.div_state !== 1 || dut.div_count !== 0)
                $fatal(1, "Divider did not start at SOF");
            for (n = 1; n <= 96; n = n + 1) begin
                @(negedge clk);
                // A too-early SOF must not overwrite an in-flight snapshot.
                sof = busy_sof && ((n == 17) || (n == 65));
                @(posedge clk); #1;
                if (dut.in_r_r1 !== active_r || dut.in_b_r1 !== active_b)
                    $fatal(1, "Active gain changed during division");
                if (n < 96 && dut.next_gain_valid !== 0)
                    $fatal(1, "Partial red/blue gain published early");
                if (n == 48 && dut.div_state !== 2)
                    $fatal(1, "Red calculation is not 48 clocks");
            end
            if (dut.div_state !== 0 || dut.next_gain_valid !== 1)
                $fatal(1, "Pair did not finish in 96 clocks");
            pending_r = gain(g, r);
            pending_b = gain(g, b);
            if (dut.next_r_gain !== pending_r || dut.next_b_gain !== pending_b)
                $fatal(1, "Quotient mismatch R/G/B=%h/%h/%h got=%h/%h expected=%h/%h",
                       r,g,b,dut.next_r_gain,dut.next_b_gain,pending_r,pending_b);
            @(negedge clk); sof = 0;
            repeat (4) begin
                @(posedge clk); #1;
                if (dut.in_r_r1 !== active_r || dut.in_b_r1 !== active_b)
                    $fatal(1, "Ready gains committed without SOF");
            end
            cases = cases + 1;
        end
    endtask

    task reset_in_phase(input integer clocks);
        begin
            @(negedge clk);
            dut.sum_r[0] = 17;
            dut.sum_g[0] = 251;
            dut.sum_b[0] = 43;
            sof = 1;
            @(negedge clk); sof = 0;
            repeat (clocks) @(negedge clk);
            rst_n = 0;
            #1;
            if (dut.div_state !== 0 || dut.next_gain_valid !== 0 ||
                dut.in_r_r1 !== 20'h10000 || dut.in_b_r1 !== 20'h10000)
                $fatal(1, "Reset did not cancel divider/pending gains");
            repeat (2) @(negedge clk);
            rst_n = 1;
            repeat (100) @(negedge clk);
            if (dut.next_gain_valid !== 0)
                $fatal(1, "Stale result appeared after reset");
            pending_r = 20'h10000;
            pending_b = 20'h10000;
        end
    endtask

    integer i;
    reg [31:0] r, g, b;
    initial begin
        repeat (3) @(negedge clk);
        rst_n = 1;
        exercise(1, 1, 1, 0);
        exercise(100, 200, 400, 0);
        exercise(3, 7, 11, 0);
        exercise(0, 5, 0, 0);
        exercise(1, 32'hffffffff, 1, 0);
        exercise(32'hffffffff, 1, 32'h80000000, 0);
        exercise(32'hffffffff, 32'hffffffff, 32'h80000001, 0);
        exercise(32'hffffffff, 0, 7, 0);
        exercise(37, 193, 79, 1);
        for (i = 0; i < 100; i = i + 1) begin
            seed = random_word(seed); r = seed;
            seed = random_word(seed); g = seed;
            seed = random_word(seed); b = seed;
            exercise(r, g, b, 0);
        end
        // Commit the last randomized result, then reset in both phases.
        exercise(1, 1, 1, 0);
        reset_in_phase(12);
        exercise(19, 83, 127, 0);
        reset_in_phase(60);
        exercise(1, 1, 1, 0);
        $display("PASS: shared AWB %0d arithmetic/frame cases; 96-cycle pair; reset in both phases", cases);
        $finish;
    end
    initial begin #2000000; $fatal(1, "Timeout"); end
endmodule

// Real AWB + real signal_delay, with only the vendor RAM replaced by a
// synchronous read-before-write model matching the existing IP settings.
module tb_awb_frame_stream;
    localparam BEATS = 80, ROWS = 3, FRAMES = 8;
    reg clk = 0;
    always #5 clk = ~clk;
    reg rst_n = 0, sof = 0, valid = 0, last = 0, awb_enable = 1;
    reg [95:0] data = 0;
    wire [95:0] out_data;
    wire out_valid, out_last, out_sof, ready;
    awb #(.IMG_WIDTH(BEATS*4), .IMG_HEIGHT(ROWS), .CONTROL_ENABLE(1)) dut (
        .I_awb_enable(awb_enable),
        .I_clk(clk), .I_rst_n(rst_n), .I_tlast(last), .I_tuser(sof),
        .I_tdata(data), .I_tvalid(valid), .I_tready(ready),
        .O_tlast(out_last), .O_tuser(out_sof), .O_tdata(out_data),
        .O_tvalid(out_valid), .O_tready(1'b1)
    );

    function [19:0] gain(input [31:0] g, input [31:0] d);
        reg [63:0] q;
        begin
            q = ({32'd0, g} << 16) / d;
            gain = (q > 64'hfffff) ? 20'hfffff : q[19:0];
        end
    endfunction
    function [23:0] corrected(input [23:0] p, input [19:0] rg, input [19:0] bg);
        reg [27:0] rr, bb;
        reg [7:0] ro, bo;
        begin
            rr = p[23:16] * rg;
            bb = p[7:0] * bg;
            ro = (rr[27:24] != 0) ? 255 : rr[23:16];
            bo = (bb[27:24] != 0) ? 255 : bb[23:16];
            corrected = {ro, p[15:8], bo};
        end
    endfunction

    reg [95:0] expected [0:8191];
    reg [19:0] frame_r = 20'h10000, frame_b = 20'h10000;
    reg [31:0] sr = 1, sg = 1, sb = 1;
    reg [31:0] sums_r [0:FRAMES-1], sums_g [0:FRAMES-1], sums_b [0:FRAMES-1];
    integer sent = 0, received = 0, run = 0, rows_seen = 0;
    integer lane, cycle = 0, first_input = -1, first_output = -1;
    reg previous_out_valid = 0;
    always @(posedge clk) begin
        cycle = cycle + 1;
        if (rst_n && valid) begin
            if (first_input < 0) first_input = cycle;
            for (lane = 0; lane < 4; lane = lane + 1)
                expected[sent][lane*24 +: 24] = corrected(data[lane*24 +: 24], frame_r, frame_b);
            sent = sent + 1;
            if (({24'd0,data[23:16]} + {24'd0,data[15:8]} + {24'd0,data[7:0]}) < 720) begin
                sr = sr + data[23:16];
                sg = sg + data[15:8];
                sb = sb + data[7:0];
            end
        end
        #1;
        if (rst_n) begin
            if (out_sof !== sof || ready !== 1)
                $fatal(1, "Existing SOF/ready interface changed");
            if (dut.in_r_r1 !== frame_r || dut.in_b_r1 !== frame_b)
                $fatal(1, "Frame gain mismatch at cycle %0d", cycle);
            if (out_valid) begin
                if (first_output < 0) first_output = cycle;
                if (received >= sent || out_data !== expected[received])
                    $fatal(1, "Pixel mismatch beat=%0d got=%h expected=%h", received, out_data, expected[received]);
                received = received + 1;
                run = run + 1;
            end else if (previous_out_valid) begin
                if (run != BEATS) $fatal(1, "Output row length %0d, expected %0d", run, BEATS);
                rows_seen = rows_seen + 1;
                run = 0;
            end
            if (out_last !== (out_valid && !dut.valid_d1))
                $fatal(1, "Existing line-end interface changed");
            previous_out_valid = out_valid;
        end
    end

    integer f, row, x, j, rr, gg, bb;
    initial begin
        repeat (3) @(negedge clk);
        rst_n = 1;
        for (f = 0; f < FRAMES; f = f + 1) begin
            @(negedge clk);
            awb_enable = (f < 2 || f % 2 != 0);
            frame_r=20'h10000;frame_b=20'h10000;
            if (f >= 2 && awb_enable) begin
                frame_r = gain(sums_g[f-2], sums_r[f-2]);
                frame_b = gain(sums_g[f-2], sums_b[f-2]);
            end
            sof = 1;
            @(negedge clk); sof = 0;
            repeat (3) @(negedge clk);
            sr = 1; sg = 1; sb = 1;
            for (row = 0; row < ROWS; row = row + 1) begin
                for (x = 0; x < BEATS; x = x + 1) begin
                    @(negedge clk);
                    for (j = 0; j < 4; j = j + 1) begin
                        rr = (f*7 + row*11 + x*3 + j*47) % 220 + 1;
                        gg = (f*17 + row*13 + x*5 + j*73) % 230 + 1;
                        bb = (f*23 + row*19 + x*7 + j*31) % 210 + 1;
                        if (j == 0 && x % 11 == 0) begin rr = 255; gg = 255; bb = 255; end
                        else if (j == 0 && x % 17 == 0) begin rr = 240; gg = 240; bb = 240; end
                        data[j*24 +: 24] = {rr[7:0], gg[7:0], bb[7:0]};
                    end
                    valid = 1;
                    last = (x == BEATS-1);
                end
                @(negedge clk); valid = 0; last = 0;
                repeat (80) @(negedge clk);
            end
            repeat (100) @(negedge clk);
            if (dut.sum_r[0] !== sr || dut.sum_g[0] !== sg || dut.sum_b[0] !== sb)
                $fatal(1, "Statistics changed: sampling/brightness exclusion/init");
            sums_r[f] = sr; sums_g[f] = sg; sums_b[f] = sb;
            if (received != sent) $fatal(1, "Frame did not drain all pixels");
        end
        repeat (10) @(negedge clk);
        if (sent != FRAMES*ROWS*BEATS || received != sent || rows_seen != FRAMES*ROWS)
            $fatal(1, "Throughput/count mismatch");
        $display("PASS: %0d frames, %0d beats/%0d pixels; 4 pixels/clock; stats, AWB ON/OFF frame gains, RAM path, clipping; first output offset=%0d clocks",
                 FRAMES, received, received*4, first_output-first_input);
        $finish;
    end
    initial begin #2000000; $fatal(1, "Timeout"); end
endmodule

// Simulation only. Do not add this file to the TD synthesis source list.
module blk_mem_gen_awb_delay_signal (
    input clka, input wea, input [10:0] addra, input [95:0] dia,
    input clkb, input [10:0] addrb, output reg [95:0] dob
);
    reg [95:0] memory [0:2047];
    always @(posedge clka) if (wea) memory[addra] <= dia;
    always @(posedge clkb) dob <= memory[addrb];
endmodule
