`timescale 1ns/1ps
module uart_video_status (
    input wire clk, rst_n, ms_tick,
    input wire pixel_clk, pixel_rst_n, frame_start, sample, fresh_valid,
    input wire [11:0] center_x, center_y,
    input wire [1:0] display_mode,
    input wire display_token,
    input wire isp_clk, isp_rst_n,
    input wire [2:0] isp_algo,
    input wire isp_frame_start,
    output wire [2:0] observed_algo,
    output wire video_alive,
    output reg [31:0] frame_id, sample_id,
    output reg [11:0] x, y,
    output reg fresh,
    output reg token,
    output reg [1:0] mode,
    output reg [15:0] sample_age
);
    reg [31:0] pixel_frame, pixel_sample;
    reg [4:0] publish_delay;
    wire snapshot_valid;
    wire [91:0] snapshot;
    wire isp_snapshot_valid;
    wire [2:0] isp_snapshot;
    reg [2:0] last_isp_algo;
    reg [15:0] video_age;
    reg have_frame, have_sample;
    always @(posedge pixel_clk or negedge pixel_rst_n) begin
        if (!pixel_rst_n) begin pixel_frame<=0;pixel_sample<=0;publish_delay<=0; end
        else begin
            if (frame_start) pixel_frame<=pixel_frame+1'b1;
            if (sample && display_mode==2'b11) pixel_sample<=pixel_sample+1'b1;
            publish_delay<={publish_delay[3:0],frame_start};
        end
    end
    uart_snapshot_mailbox #(.WIDTH(92)) u_video_mailbox (
        .src_clk(pixel_clk),.src_rst_n(pixel_rst_n),.src_valid(publish_delay[4]),
        .src_data({display_token,display_mode,fresh_valid,center_y,center_x,pixel_sample,pixel_frame}),
        .dst_clk(clk),.dst_rst_n(rst_n),.dst_valid(snapshot_valid),.dst_data(snapshot)
    );
    uart_snapshot_mailbox #(.WIDTH(3)) u_isp_mailbox (
        .src_clk(isp_clk),.src_rst_n(isp_rst_n),.src_valid(isp_frame_start),.src_data(isp_algo),
        .dst_clk(clk),.dst_rst_n(rst_n),.dst_valid(isp_snapshot_valid),.dst_data(isp_snapshot)
    );
    assign video_alive=have_frame && video_age<250;
    assign observed_algo=(mode==0) ? 3'd0 : (mode==2) ? 3'd4 : (mode==3) ? 3'd5 :
                         (last_isp_algo>=1 && last_isp_algo<=3) ? last_isp_algo : 3'd1;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            frame_id<=0;sample_id<=0;x<=0;y<=0;fresh<=0;token<=0;mode<=0;
            video_age<=16'hffff;sample_age<=16'hffff;have_frame<=0;have_sample<=0;last_isp_algo<=0;
        end else begin
            if (ms_tick) begin
                if (video_age<16'hfffe) video_age<=video_age+1'b1;
                if (have_sample && sample_age<16'hfffe) sample_age<=sample_age+1'b1;
            end
            if (isp_snapshot_valid) last_isp_algo<=isp_snapshot;
            if (snapshot_valid) begin
                frame_id<=snapshot[31:0];sample_id<=snapshot[63:32];
                x<=snapshot[75:64];y<=snapshot[87:76];fresh<=snapshot[88];
                mode<=snapshot[90:89];token<=snapshot[91];
                video_age<=0;have_frame<=1;
                if (snapshot[90:89]==3) begin sample_age<=0;have_sample<=1; end
                else begin sample_age<=16'hffff;have_sample<=0;fresh<=0; end
            end
        end
    end
endmodule
