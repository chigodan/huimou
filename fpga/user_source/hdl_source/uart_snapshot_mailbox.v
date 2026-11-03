`timescale 1ns/1ps
// One coherent sample in flight. Events arriving while busy are skipped, not torn.
module uart_snapshot_mailbox #(parameter integer WIDTH=92)(
    input wire src_clk, src_rst_n, src_valid,
    input wire [WIDTH-1:0] src_data,
    input wire dst_clk, dst_rst_n,
    output reg dst_valid,
    output reg [WIDTH-1:0] dst_data
);
    reg [WIDTH-1:0] held;
    reg request, acknowledge;
    (* async_reg="true" *) reg ack_meta, ack_sync, req_meta, req_sync;
    reg capture_pending;
    always @(posedge src_clk or negedge src_rst_n) begin
        if (!src_rst_n) begin held<=0;request<=0;ack_meta<=0;ack_sync<=0; end
        else begin
            ack_meta<=acknowledge;ack_sync<=ack_meta;
            if (src_valid && request==ack_sync) begin held<=src_data;request<=!request; end
        end
    end
    always @(posedge dst_clk or negedge dst_rst_n) begin
        if (!dst_rst_n) begin
            req_meta<=0;req_sync<=0;acknowledge<=0;dst_valid<=0;dst_data<=0;capture_pending<=0;
        end else begin
            req_meta<=request;req_sync<=req_meta;dst_valid<=0;
            if (capture_pending) begin
                dst_data<=held;dst_valid<=1;acknowledge<=req_sync;capture_pending<=0;
            end else if (req_sync!=acknowledge) capture_pending<=1;
        end
    end
endmodule
