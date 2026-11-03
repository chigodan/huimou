`timescale 1ns/1ps
// Bounded sliding parser. Payload AA55 is data; bad candidates rescan one byte later.
module uart_packet_rx (
    input wire clk, rst_n, ms_tick,
    input wire [7:0] byte_data,
    input wire byte_valid, byte_error,
    output reg packet_valid,
    input wire packet_ready,
    output reg [7:0] version, seq, cmd, length,
    output reg [255:0] payload
);
    reg [7:0] buffer [0:39];
    reg [5:0] count, scan, remove_count;
    reg [7:0] gap_ms;
    reg scanning;
    reg [15:0] crc;
    integer i;
    function [15:0] crc_byte;
        input [15:0] previous;
        input [7:0] value;
        reg [15:0] work;
        integer b;
        begin
            work = previous ^ {value,8'h00};
            for (b=0; b<8; b=b+1)
                work = work[15] ? (work << 1) ^ 16'h1021 : work << 1;
            crc_byte = work;
        end
    endfunction
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            count<=0; scan<=0; remove_count<=0; gap_ms<=0; scanning<=0; crc<=16'hffff;
            packet_valid<=0; version<=0; seq<=0; cmd<=0; length<=0; payload<=0;
        end else begin
            if (packet_valid && packet_ready) packet_valid<=0;
            if (byte_valid) gap_ms<=0;
            else if (ms_tick && gap_ms<101) gap_ms<=gap_ms+1'b1;
            if (byte_error || (ms_tick && gap_ms>=100 && !byte_valid)) begin
                count<=0; scanning<=0; remove_count<=0;
            end else if (byte_valid) begin
                // A normal host has one request outstanding. Overload never executes a partial packet.
                if (count<40) begin buffer[count]<=byte_data; count<=count+1'b1; end
                else begin count<=0; scanning<=0; remove_count<=0; end
            end else if (remove_count!=0) begin
                // Fixed one-byte shift avoids a 40-byte variable barrel shifter.
                for(i=0;i<39;i=i+1) buffer[i]<=buffer[i+1];
                count<=count-1'b1;remove_count<=remove_count-1'b1;
            end else if (!packet_valid) begin
                if (scanning) begin
                    if (scan < 6+buffer[5]) begin
                        crc<=crc_byte(crc,buffer[scan]); scan<=scan+1'b1;
                    end else begin
                        scanning<=0;
                        if (crc == {buffer[7+buffer[5]],buffer[6+buffer[5]]}) begin
                            version<=buffer[2]; seq<=buffer[3]; cmd<=buffer[4]; length<=buffer[5];
                            for (i=0;i<32;i=i+1)
                                payload[i*8+:8] <= (i<buffer[5]) ? buffer[6+i] : 8'h00;
                            packet_valid<=1;
                            remove_count<=8+buffer[5];
                        end else begin
                            for(i=0;i<39;i=i+1) buffer[i]<=buffer[i+1];
                            count<=count-1'b1;
                        end
                    end
                end else if (count>=2) begin
                    if (buffer[0]!=8'haa || buffer[1]!=8'h55) begin
                        for(i=0;i<39;i=i+1) buffer[i]<=buffer[i+1];
                        count<=count-1'b1;
                    end else if (count>=6) begin
                        if (buffer[5]>32) begin
                            for(i=0;i<39;i=i+1) buffer[i]<=buffer[i+1];
                            count<=count-1'b1;
                        end else if (count>=8+buffer[5]) begin
                            scanning<=1; scan<=2; crc<=16'hffff;
                        end
                    end
                end else if (count==1 && buffer[0]!=8'haa) count<=0;
            end
        end
    end
endmodule
