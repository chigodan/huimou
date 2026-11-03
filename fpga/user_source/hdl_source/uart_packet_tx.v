`timescale 1ns/1ps
module uart_packet_tx (
    input wire clk, rst_n,
    input wire packet_valid,
    output wire packet_ready,
    input wire [7:0] seq, cmd, length,
    input wire [255:0] payload,
    output reg [7:0] byte_data,
    output wire byte_valid,
    input wire byte_ready
);
    reg active, draining;
    reg [5:0] index;
    reg [7:0] saved_seq, saved_cmd, saved_length;
    reg [255:0] saved_payload;
    reg [15:0] crc;
    function [15:0] crc_byte;
        input [15:0] previous;
        input [7:0] value;
        reg [15:0] work;
        integer b;
        begin
            work=previous ^ {value,8'h00};
            for(b=0;b<8;b=b+1) work=work[15] ? (work<<1)^16'h1021 : work<<1;
            crc_byte=work;
        end
    endfunction
    assign packet_ready=!active;
    assign byte_valid=active && !draining;
    always @* begin
        case(index)
            0: byte_data=8'haa;
            1: byte_data=8'h55;
            2: byte_data=8'h01;
            3: byte_data=saved_seq;
            4: byte_data=saved_cmd;
            5: byte_data=saved_length;
            default: begin
                if (index<6+saved_length) byte_data=saved_payload[(index-6)*8+:8];
                else if (index==6+saved_length) byte_data=crc[7:0];
                else byte_data=crc[15:8];
            end
        endcase
    end
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            active<=0; draining<=0; index<=0; crc<=16'hffff;
            saved_seq<=0; saved_cmd<=0; saved_length<=0; saved_payload<=0;
        end else if (!active) begin
            if (packet_valid) begin
                active<=1; draining<=0; index<=0; crc<=16'hffff;
                saved_seq<=seq; saved_cmd<=cmd; saved_length<=length; saved_payload<=payload;
            end
        end else if (draining) begin
            if (byte_ready) begin active<=0; draining<=0; end
        end else if (byte_ready) begin
            if (index>=2 && index<6+saved_length) crc<=crc_byte(crc,byte_data);
            if (index==7+saved_length) draining<=1;
            else index<=index+1'b1;
        end
    end
endmodule
