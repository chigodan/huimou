`timescale 1ns/1ps
module tb_osd_decimal_formatter;
    reg clk=0, rst=0, start=0;
    always #5 clk=~clk;
    reg [95:0] values=0;
    wire [11:0] cap_bcd, display_bcd, new_bcd;
    wire [15:0] latency_bcd, x_bcd, y_bcd;
    wire done;
    osd_decimal_formatter dut(.I_clk(clk),.I_rst_n(rst),.I_start(start),
        .I_values(values),.O_capture_bcd(cap_bcd),.O_display_bcd(display_bcd),
        .O_new_bcd(new_bcd),.O_latency_bcd(latency_bcd),
        .O_x_bcd(x_bcd),.O_y_bcd(y_bcd),.O_done(done));
    function [15:0] expected_bcd(input integer number);
        begin
            expected_bcd[15:12]=(number/1000)%10;
            expected_bcd[11:8]=(number/100)%10;
            expected_bcd[7:4]=(number/10)%10;
            expected_bcd[3:0]=number%10;
        end
    endfunction
    task launch(input [95:0] snapshot);
        begin
            @(negedge clk);values=snapshot;start=1;
            @(negedge clk);start=0;
        end
    endtask
    task finish_and_check;
        reg [15:0] e0,e1,e2;
        integer cycle;
        begin
            for(cycle=1;cycle<=96;cycle=cycle+1) begin
                @(negedge clk);
                if(done!==(cycle==96)) $fatal(1,"formatter must finish in exactly 96 clocks");
            end
            e0=expected_bcd(values[95:80]);e1=expected_bcd(values[79:64]);e2=expected_bcd(values[63:48]);
            if(cap_bcd!==e0[11:0] || display_bcd!==e1[11:0] || new_bcd!==e2[11:0] ||
               latency_bcd!==expected_bcd(values[47:32]) ||
               x_bcd!==expected_bcd(values[31:16]) || y_bcd!==expected_bcd(values[15:0]))
                $fatal(1,"shared decimal conversion mismatch for %h",values);
        end
    endtask
    integer i;
    reg [15:0] a,b,c,d,e,f;
    initial begin
        repeat(5) @(negedge clk);rst=1;
        // Exhaust every 16-bit latency value and every 12-bit pixel coordinate;
        // also sweep 0..999 FPS, leading zeros, decimal carries and modulo wrap.
        for(i=0;i<65536;i=i+1) begin
            a=i%1000;b=(i*7)%1000;c=(65535-i)%1000;
            d=i;e=65535-i;f=i^16'h5a5a;
            launch({a,b,c,d,e,f});finish_and_check();
        end
        $display("PASS: exhaustive 16-bit decimal conversion, all FPS digits, leading zeros, modulo-10000 and exact 96-clock deadline");
        // A new frame cancels an unfinished conversion and uses its new data.
        launch({16'd999,16'd999,16'd999,16'd65535,16'd4095,16'd4095});
        repeat(29) @(negedge clk);
        launch({16'd30,16'd60,16'd25,16'd67,16'd1279,16'd719});finish_and_check();
        repeat(4) @(negedge clk);
        if(done || cap_bcd!=12'h030 || display_bcd!=12'h060 || new_bcd!=12'h025 ||
           latency_bcd!=16'h0067 || x_bcd!=16'h1279 || y_bcd!=16'h0719)
            $fatal(1,"restart or idle result retention failed");
        launch({6{16'd999}});repeat(23) @(negedge clk);rst=0;
        @(negedge clk);
        if({cap_bcd,display_bcd,new_bcd,latency_bcd,x_bcd,y_bcd,done}!==0)
            $fatal(1,"reset during conversion did not clear outputs");
        rst=1;repeat(100) @(negedge clk);
        if(done) $fatal(1,"reset left a pending conversion");
        $display("PASS: frame restart, output retention and reset while busy");
        $finish;
    end
    initial begin #100000000; $fatal(1,"decimal formatter test timeout"); end
endmodule
