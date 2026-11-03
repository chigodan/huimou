`timescale 1ns/1ps
// Bundled command bus is held until both frame domains acknowledge its toggle.
// At least one settle frame is required: consumers see these registers one frame later.
module uart_frame_command #(parameter integer SETTLE_FRAMES=1)(
    input wire clk, rst_n, frame_start,
    input wire override_async, token_async,
    input wire [2:0] algo_async,
    output reg owner,
    output reg [2:0] algo,
    output reg ack
);
    (* async_reg="true" *) reg owner_meta, owner_sync, token_meta, token_sync;
    reg [2:0] algo_meta, algo_sync;
    reg seen;
    reg [2:0] settle;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            owner_meta<=0;owner_sync<=0;token_meta<=0;token_sync<=0;
            algo_meta<=0;algo_sync<=0;owner<=0;algo<=0;ack<=0;seen<=0;settle<=0;
        end else begin
            owner_meta<=override_async;owner_sync<=owner_meta;
            token_meta<=token_async;token_sync<=token_meta;
            algo_meta<=algo_async;algo_sync<=algo_meta;
            if (frame_start) begin
                if (token_sync!=seen || owner_sync!=owner) begin
                    owner<=owner_sync;algo<=algo_sync;seen<=token_sync;
                    settle<=SETTLE_FRAMES;
                end else if (settle!=0) begin
                    settle<=settle-1'b1;
                    if (settle==1) ack<=seen;
                end
            end
        end
    end
endmodule
