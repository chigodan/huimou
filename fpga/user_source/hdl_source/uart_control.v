`timescale 1ns/1ps
// UART v1: legacy commands, display brightness (11/12), image auto control (13/14).
module uart_control #(
    parameter integer CLK_HZ=100_000_000,
    parameter integer BAUD=115200,
    parameter integer ALGO_ENABLED=1,
    parameter integer TRACKING_ENABLED=1,
    parameter integer IMAGE_AUTO_ENABLED=0
)(
    input wire clk, rst_n, rx,
    output wire tx, ms_tick,
    output reg owner, pending, token,
    output reg [2:0] target_algo,
    output reg [7:0] target_brightness,
    output reg brightness_token,
    input wire brightness_ack,
    output reg [1:0] target_image_flags,
    output reg image_token,
    input wire image_isp_ack, image_ae_ack,
    input wire [7:0] image_flags, image_mean, image_gain,
    input wire [2:0] image_state,
    input wire [15:0] image_exposure,
    input wire [19:0] image_r_gain, image_b_gain,
    input wire isp_ack, display_ack,
    input wire [2:0] observed_algo,
    input wire video_alive, result_fresh, result_token,
    input wire [1:0] result_mode,
    input wire [11:0] center_x, center_y,
    input wire [15:0] sample_age,
    input wire [31:0] frame_id, sample_id
);
    wire [7:0] rx_byte;
    wire rx_valid, rx_error;
    wire request_valid, request_ready;
    wire [7:0] request_version, request_seq, request_cmd, request_len;
    wire [255:0] request_payload;
    reg response_valid;
    wire response_ready;
    reg [7:0] response_seq, response_cmd, response_len;
    reg [255:0] response_payload;
    wire [7:0] tx_byte;
    wire tx_byte_valid, tx_byte_ready;
    localparam integer MS_DIV_WIDTH=(CLK_HZ/1000>1) ? $clog2(CLK_HZ/1000) : 1;
    reg [MS_DIV_WIDTH-1:0] divider;
    reg [31:0] uptime_ms;
    (* async_reg="true" *) reg isp_ack_meta, isp_ack_sync, display_ack_meta, display_ack_sync;
    (* async_reg="true" *) reg brightness_ack_meta, brightness_ack_sync;
    reg brightness_pending, image_pending;
    (* async_reg="true" *) reg image_isp_meta, image_isp_sync, image_ae_meta, image_ae_sync;
    reg [1:0] committed_image_flags, previous_image_flags;
    reg terminal_image;
    reg [7:0] committed_brightness, previous_brightness;
    reg [2:0] committed_algo, previous_algo;
    reg [8:0] apply_age;
    reg [7:0] pending_seq;
    reg terminal_valid, terminal_ok;
    reg [7:0] terminal_value;
    reg terminal_brightness;
    reg cache_valid;
    reg [11:0] cache_age;
    reg [7:0] cache_version, cache_seq, cache_cmd, cache_len, cache_reply_len;
    reg [255:0] cache_payload, cache_reply;
    reg [255:0] reply_work;
    reg [7:0] error_work;
    wire [2:0] active_algo=owner ? committed_algo : observed_algo;
    wire coherent_sample=active_algo==5 && result_mode==3 && (!owner || result_token==token);
    wire valid_result=coherent_sample && !pending && video_alive && result_fresh &&
                      sample_age<250 && center_x<1280 && center_y<720;
    wire operation_pending=pending || brightness_pending || image_pending;
    wire [7:0] pending_command=image_pending ? 8'h13 : brightness_pending ? 8'h11 : 8'h10;
    wire [7:0] pending_value=image_pending ? {6'd0,target_image_flags} :
        brightness_pending ? target_brightness : {5'd0,target_algo};
    wire application_acked=image_pending ? (image_isp_sync==image_token && image_ae_sync==image_token) :
                           brightness_pending ? brightness_ack_sync==brightness_token :
                           (isp_ack_sync==token && display_ack_sync==token);
    wire application_finished=operation_pending && (application_acked || apply_age>=400);
    wire cache_matches=cache_version==request_version && cache_cmd==request_cmd &&
                       cache_len==request_len && cache_payload==request_payload;
    assign request_ready=!response_valid && !application_finished && !terminal_valid;
    assign ms_tick=(divider==CLK_HZ/1000-1);
    uart_rx #(.CLK_HZ(CLK_HZ),.BAUD(BAUD)) u_uart_rx (
        .clk(clk),.rst_n(rst_n),.rx(rx),.data(rx_byte),.valid(rx_valid),.framing_error(rx_error));
    uart_packet_rx u_packet_rx (
        .clk(clk),.rst_n(rst_n),.ms_tick(ms_tick),.byte_data(rx_byte),
        .byte_valid(rx_valid),.byte_error(rx_error),.packet_valid(request_valid),
        .packet_ready(request_ready),.version(request_version),.seq(request_seq),
        .cmd(request_cmd),.length(request_len),.payload(request_payload));
    uart_packet_tx u_packet_tx (
        .clk(clk),.rst_n(rst_n),.packet_valid(response_valid),.packet_ready(response_ready),
        .seq(response_seq),.cmd(response_cmd),.length(response_len),.payload(response_payload),
        .byte_data(tx_byte),.byte_valid(tx_byte_valid),.byte_ready(tx_byte_ready));
    uart_tx #(.CLK_HZ(CLK_HZ),.BAUD(BAUD)) u_uart_tx (
        .clk(clk),.rst_n(rst_n),.data(tx_byte),.valid(tx_byte_valid),.ready(tx_byte_ready),.tx(tx));

    task reply;
        input [7:0] seq_value, cmd_value, len_value;
        input [255:0] payload_value;
        begin
            response_valid<=1;response_seq<=seq_value;response_cmd<=cmd_value|8'h80;
            response_len<=len_value;response_payload<=payload_value;
        end
    endtask
    task cache_request;
        input [7:0] len_value;
        input [255:0] payload_value;
        begin
            cache_valid<=1;cache_age<=0;
            cache_version<=request_version;cache_seq<=request_seq;
            cache_cmd<=request_cmd;cache_len<=request_len;cache_payload<=request_payload;
            cache_reply_len<=len_value;cache_reply<=payload_value;
        end
    endtask
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            divider<=0;uptime_ms<=0;owner<=0;pending<=0;token<=0;target_algo<=0;
            target_brightness<=0;brightness_token<=0;brightness_pending<=0;
            committed_brightness<=0;previous_brightness<=0;
            image_pending<=0;target_image_flags<=3;image_token<=0;
            committed_image_flags<=3;previous_image_flags<=3;terminal_image<=0;
            image_isp_meta<=0;image_isp_sync<=0;image_ae_meta<=0;image_ae_sync<=0;
            brightness_ack_meta<=0;brightness_ack_sync<=0;
            isp_ack_meta<=0;isp_ack_sync<=0;display_ack_meta<=0;display_ack_sync<=0;
            committed_algo<=0;previous_algo<=0;apply_age<=0;pending_seq<=0;
            terminal_valid<=0;terminal_ok<=0;terminal_value<=0;terminal_brightness<=0;
            response_valid<=0;response_seq<=0;response_cmd<=0;response_len<=0;response_payload<=0;
            cache_valid<=0;cache_age<=0;cache_version<=0;cache_seq<=0;cache_cmd<=0;
            cache_len<=0;cache_payload<=0;cache_reply_len<=0;cache_reply<=0;
        end else begin
            isp_ack_meta<=isp_ack;isp_ack_sync<=isp_ack_meta;
            display_ack_meta<=display_ack;display_ack_sync<=display_ack_meta;
            brightness_ack_meta<=brightness_ack;brightness_ack_sync<=brightness_ack_meta;
            image_isp_meta<=image_isp_ack;image_isp_sync<=image_isp_meta;
            image_ae_meta<=image_ae_ack;image_ae_sync<=image_ae_meta;
            if (ms_tick) begin divider<=0;uptime_ms<=uptime_ms+1'b1; end
            else divider<=divider+1'b1;
            if (operation_pending && ms_tick && apply_age<400) apply_age<=apply_age+1'b1;
            if (response_valid || !response_ready) cache_age<=0;
            else if (cache_valid && ms_tick) begin
                if (cache_age>=3999) cache_valid<=0;
                else cache_age<=cache_age+1'b1;
            end
            if (response_valid && response_ready) response_valid<=0;
            // Decide on time, independently of TX congestion. A late acknowledgement
            // must never turn an expired transaction into success.
            if (application_finished) begin
                pending<=0;brightness_pending<=0;image_pending<=0;
                terminal_valid<=1;terminal_brightness<=brightness_pending;terminal_image<=image_pending;
                terminal_value<=pending_value;
                terminal_ok<=apply_age<400 && application_acked;
                if (image_pending) begin
                    if (apply_age<400 && application_acked) committed_image_flags<=target_image_flags;
                    else begin target_image_flags<=previous_image_flags;image_token<=!image_token;end
                end else if (brightness_pending) begin
                    if (apply_age<400 && application_acked)
                        committed_brightness<=target_brightness;
                    else begin
                        target_brightness<=previous_brightness;brightness_token<=!brightness_token;
                    end
                end else if (apply_age<400 && application_acked) begin
                    owner<=1;committed_algo<=target_algo;
                end else begin
                    target_algo<=previous_algo;token<=!token;
                end
            end
            if (terminal_valid && !response_valid) begin
                terminal_valid<=0;
                reply_work=0;
                if (terminal_ok) begin
                    reply_work[15:0]={terminal_value,8'h00};
                    reply(pending_seq,terminal_image ? 8'h13 : terminal_brightness ? 8'h11 : 8'h10,2,reply_work);
                    cache_reply_len<=2;
                end else begin
                    reply_work[7:0]=8'h06;
                    reply(pending_seq,terminal_image ? 8'h13 : terminal_brightness ? 8'h11 : 8'h10,1,reply_work);
                    cache_reply_len<=1;
                end
                cache_valid<=1;cache_age<=0;cache_version<=1;cache_seq<=pending_seq;
                cache_cmd<=terminal_image ? 8'h13 : terminal_brightness ? 8'h11 : 8'h10;
                cache_len<=1;cache_payload<={248'd0,terminal_value};cache_reply<=reply_work;
            end else if (request_valid && request_ready && !request_cmd[7]) begin
                error_work=0;reply_work=0;
                if (request_version!=1) error_work=5;
                else if (request_seq==0) error_work=3;
                else if (operation_pending && request_seq==pending_seq) begin
                    if (request_cmd!=pending_command || request_len!=1 ||
                        request_payload!={248'd0,pending_value})
                        error_work=7;
                    // Exact pending repeat has no immediate response.
                end else if (cache_valid && request_seq==cache_seq) begin
                    if (!cache_matches) error_work=7;
                    else reply(cache_seq,cache_cmd,cache_reply_len,cache_reply);
                end else begin
                    case (request_cmd)
                        8'h01: begin
                            if (request_len!=0) error_work=2;
                            else begin
                                reply_work[7:0]=0;reply_work[15:8]=1;
                                reply_work[31:16]=16'h0012 | (ALGO_ENABLED ? 16'd1 : 16'd0) |
                                                      (TRACKING_ENABLED ? 16'd8 : 16'd0) |
                                                      (IMAGE_AUTO_ENABLED ? 16'h0020 : 16'd0);
                                reply_work[47:32]=1280;reply_work[63:48]=720;
                                reply(request_seq,request_cmd,8,reply_work);cache_request(8,reply_work);
                            end
                        end
                        8'h02: begin
                            if (request_len!=0) error_work=2;
                            else begin
                                reply_work[15:8]={5'd0,active_algo};
                                reply_work[23:16]=pending ? {5'd0,target_algo} : 8'hff;
                                reply_work[31:24]={3'b000,pending,owner,
                                    (TRACKING_ENABLED!=0 && active_algo==5),valid_result,video_alive};
                                reply_work[47:32]=1280;reply_work[63:48]=720;
                                reply_work[95:64]=frame_id;reply_work[127:96]=sample_id;
                                reply_work[143:128]=valid_result ? {4'd0,center_x} : 16'hffff;
                                reply_work[159:144]=valid_result ? {4'd0,center_y} : 16'hffff;
                                reply_work[175:160]=coherent_sample ? sample_age : 16'hffff;
                                reply_work[191:176]=0;reply_work[223:192]=uptime_ms;
                                reply(request_seq,request_cmd,28,reply_work);cache_request(28,reply_work);
                            end
                        end
                        8'h10: begin
                            if (ALGO_ENABLED==0) error_work=1;
                            else if (request_len!=1) error_work=2;
                            else if (request_payload[7:0]>5) error_work=3;
                            else if (operation_pending) error_work=4;
                            else if (isp_ack_sync!=token || display_ack_sync!=token) error_work=4;
                            else if (owner && request_payload[2:0]==committed_algo) begin
                                reply_work[15:0]={5'd0,committed_algo,8'h00};
                                reply(request_seq,request_cmd,2,reply_work);cache_request(2,reply_work);
                            end else begin
                                pending<=1;pending_seq<=request_seq;apply_age<=0;
                                previous_algo<=active_algo;target_algo<=request_payload[2:0];token<=!token;
                            end
                        end
                        8'h11: begin
                            if (request_len!=1) error_work=2;
                            else if (operation_pending || brightness_ack_sync!=brightness_token) error_work=4;
                            else if (request_payload[7:0]==committed_brightness) begin
                                reply_work[15:0]={committed_brightness,8'h00};
                                reply(request_seq,request_cmd,2,reply_work);cache_request(2,reply_work);
                            end else begin
                                brightness_pending<=1;pending_seq<=request_seq;apply_age<=0;
                                previous_brightness<=committed_brightness;
                                target_brightness<=request_payload[7:0];brightness_token<=!brightness_token;
                            end
                        end
                        8'h12: begin
                            if (request_len!=0) error_work=2;
                            else begin
                                reply_work[15:0]={committed_brightness,8'h00};
                                reply(request_seq,request_cmd,2,reply_work);cache_request(2,reply_work);
                            end
                        end
                        8'h13: begin
                            if (IMAGE_AUTO_ENABLED==0) error_work=1;
                            else if (request_len!=1) error_work=2;
                            else if (request_payload[7:0]>3) error_work=3;
                            else if (operation_pending || image_isp_sync!=image_token || image_ae_sync!=image_token) error_work=4;
                            else if (request_payload[1:0]==committed_image_flags) begin
                                reply_work[15:0]={6'd0,committed_image_flags,8'h00};
                                reply(request_seq,request_cmd,2,reply_work);cache_request(2,reply_work);
                            end else begin
                                image_pending<=1;pending_seq<=request_seq;apply_age<=0;
                                previous_image_flags<=committed_image_flags;
                                target_image_flags<=request_payload[1:0];image_token<=!image_token;
                            end
                        end
                        8'h14: begin
                            if (IMAGE_AUTO_ENABLED==0) error_work=1;
                            else if (request_len!=0) error_work=2;
                            else begin
                                reply_work[15:8]=image_flags;
                                reply_work[23:16]={5'd0,image_state};
                                reply_work[31:24]=image_mean;
                                reply_work[39:32]=8'd128;
                                reply_work[47:40]=image_gain;
                                reply_work[63:48]=image_exposure;
                                reply_work[95:64]={12'd0,image_r_gain};
                                reply_work[127:96]={12'd0,image_b_gain};
                                reply(request_seq,request_cmd,16,reply_work);cache_request(16,reply_work);
                            end
                        end
                        default: error_work=1;
                    endcase
                end
                if (error_work!=0) begin
                    reply_work=0;reply_work[7:0]=error_work;
                    reply(request_seq,request_cmd,1,reply_work);
                    // Conflicts must not evict the cached request they conflict with.
                    if (error_work!=7) cache_request(1,reply_work);
                end
            end
        end
    end
endmodule
