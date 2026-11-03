`timescale 1ns/1ps
module tb_uart_image_auto;
    reg clk=0,isp_clk=0,pixel_clk=0;
    always #10 clk=~clk;
    always #7 isp_clk=~isp_clk;
    always #13 pixel_clk=~pixel_clk;
    reg rst_n=0,rx=1,enable_pixel=1,enable_isp=1;
    reg pixel_frame=0,isp_frame=0;
    integer pc=0,ic=0;
    always @(posedge pixel_clk) begin
        pixel_frame<=0;
        if(pc==149) begin pc<=0;pixel_frame<=enable_pixel;end else pc<=pc+1;
    end
    always @(posedge isp_clk) begin
        isp_frame<=0;
        if(ic==149) begin ic<=0;isp_frame<=enable_isp;end else ic<=ic+1;
    end
    wire tx,owner,pending,token,ms_tick,isp_ack,pixel_ack,brightness_ack,brightness_token;
    wire [2:0] target;
    wire [7:0] brightness;
    wire [1:0] image_flags,awb_flags,ae_flags;
    wire image_token,awb_ack,ae_ack;
    image_frame_settings awb_settings(.clk(isp_clk),.rst_n(rst_n),.frame_start(isp_frame),
        .token_async(image_token),.flags_async(image_flags),.flags(awb_flags),.ack(awb_ack));
    image_frame_settings ae_settings(.clk(pixel_clk),.rst_n(rst_n),.frame_start(pixel_frame),
        .token_async(image_token),.flags_async(image_flags),.flags(ae_flags),.ack(ae_ack));
    uart_frame_command #(.SETTLE_FRAMES(2)) isp (
        .clk(isp_clk),.rst_n(rst_n),.frame_start(isp_frame),
        .override_async(owner||pending),.token_async(token),.algo_async(target),
        .owner(),.algo(),.ack(isp_ack));
    uart_frame_command pixel (
        .clk(pixel_clk),.rst_n(rst_n),.frame_start(pixel_frame),
        .override_async(owner||pending),.token_async(token),.algo_async(target),
        .owner(),.algo(),.ack(pixel_ack));
    video_brightness display (
        .clk(pixel_clk),.rst_n(rst_n),.frame_start(pixel_frame),
        .value_async(brightness),.token_async(brightness_token),.ack(brightness_ack),
        .vsync(1'b0),.hsync(1'b0),.de(1'b1),.rgb(24'h808080),
        .out_vsync(),.out_hsync(),.out_de(),.out_rgb());
    uart_control #(.CLK_HZ(1_000_000),.BAUD(100000),.IMAGE_AUTO_ENABLED(1)) dut (
        .clk(clk),.rst_n(rst_n),.rx(rx),.tx(tx),.ms_tick(ms_tick),
        .owner(owner),.pending(pending),.token(token),.target_algo(target),
        .target_image_flags(image_flags),.image_token(image_token),
        .image_isp_ack(awb_ack),.image_ae_ack(ae_ack),
        .image_flags({3'd0,3'b111,ae_flags[1],awb_flags[0]}),
        .image_state(ae_flags[1] ? 3'd2 : 3'd5),.image_mean(8'd127),
        .image_exposure(16'd1600),.image_gain(8'd48),
        .image_r_gain(awb_flags[0] ? 20'h12000 : 20'h10000),
        .image_b_gain(awb_flags[0] ? 20'he000 : 20'h10000),
        .target_brightness(brightness),.brightness_token(brightness_token),.brightness_ack(brightness_ack),
        .isp_ack(isp_ack),.display_ack(pixel_ack),.observed_algo(3'd0),
        .video_alive(1'b1),.result_fresh(1'b1),.result_token(token),.result_mode(2'b11),
        .center_x(12'd640),.center_y(12'd360),.sample_age(16'd8),
        .frame_id(32'd123),.sample_id(32'd42));
    function [15:0] crc_byte(input [15:0] previous,input [7:0] b);
        reg [15:0] c;integer n;
        begin
            c=previous^{b,8'd0};
            for(n=0;n<8;n=n+1) c=(c<<1)^(c[15] ? 16'h1021:16'd0);
            crc_byte=c;
        end
    endfunction
    reg [7:0] packet[0:39],decoded;
    reg [7:0] response_seq,response_cmd,response_len;
    reg [255:0] response_payload;
    integer bytes_seen=0,packet_size=0,responses=0,b,j,log_file;
    reg [15:0] received_crc;
    initial begin
        log_file=$fopen("image_auto_responses.hex","w");
        forever begin
            @(negedge tx);#100;
            if(tx!==0) $fatal(1,"UART start bit");
            for(b=0;b<8;b=b+1) begin #200;decoded[b]=tx;end
            #200;if(tx!==1) $fatal(1,"UART stop bit");
            packet[bytes_seen]=decoded;
            if(bytes_seen==5) packet_size=8+decoded;
            bytes_seen=bytes_seen+1;
            if(packet_size!=0 && bytes_seen==packet_size) begin
                if(packet[0]!=8'haa || packet[1]!=8'h55 || packet[2]!=1) $fatal(1,"UART header/version");
                received_crc=16'hffff;
                for(j=2;j<packet_size-2;j=j+1) received_crc=crc_byte(received_crc,packet[j]);
                if(received_crc!=={packet[packet_size-1],packet[packet_size-2]}) $fatal(1,"response CRC");
                response_seq=packet[3];response_cmd=packet[4];response_len=packet[5];response_payload=0;
                for(j=0;j<response_len;j=j+1) response_payload[j*8 +: 8]=packet[6+j];
                for(j=0;j<packet_size;j=j+1) $fwrite(log_file,"%02x ",packet[j]);
                $fwrite(log_file,"\n");$fflush(log_file);
                responses=responses+1;bytes_seen=0;packet_size=0;
            end
        end
    end
    task send_byte(input [7:0] value);
        integer bit_number;
        begin
            @(negedge clk);rx=0;#200;
            for(bit_number=0;bit_number<8;bit_number=bit_number+1) begin rx=value[bit_number];#200;end
            rx=1;#200;
        end
    endtask
    task request(input [7:0] seq_value,cmd,len,input [255:0] payload,input bad_crc);
        reg [15:0] crc;integer n;
        begin
            crc=16'hffff;
            send_byte(8'haa);send_byte(8'h55);
            send_byte(1);crc=crc_byte(crc,1);
            send_byte(seq_value);crc=crc_byte(crc,seq_value);
            send_byte(cmd);crc=crc_byte(crc,cmd);
            send_byte(len);crc=crc_byte(crc,len);
            for(n=0;n<len;n=n+1) begin send_byte(payload[n*8 +: 8]);crc=crc_byte(crc,payload[n*8 +: 8]);end
            send_byte(crc[7:0]^{7'd0,bad_crc});send_byte(crc[15:8]);
        end
    endtask
    task expect_reply(input [7:0] seq_value,cmd,len,result_value);
        integer previous,tries;
        begin
            previous=responses;tries=0;
            while(responses==previous && tries<1000000) begin @(negedge clk);tries=tries+1;end
            if(responses!=previous+1 || response_seq!==seq_value || response_cmd!==cmd ||
               response_len!==len || response_payload[7:0]!==result_value)
                $fatal(1,"bad/absent response expected %h/%h/%0d/%0d got %h/%h/%0d/%0d",
                    seq_value,cmd,len,result_value,response_seq,response_cmd,response_len,response_payload[7:0]);
        end
    endtask
    task set_image(input [7:0] seq_value,flags);
        begin
            request(seq_value,8'h13,1,{248'd0,flags},0);expect_reply(seq_value,8'h93,2,0);
            if(response_payload[15:8]!==flags || awb_flags!==flags[1:0] || ae_flags!==flags[1:0])
                $fatal(1,"Image ACK preceded applied frame settings");
        end
    endtask
    integer previous;
    reg saved_token;
    initial begin
        repeat(10) @(negedge clk);rst_n=1;
        request(1,1,0,0,0);expect_reply(1,8'h81,8,0);
        if(response_payload[31:16]!==16'h003b) $fatal(1,"Image capability");
        request(2,8'h14,0,0,0);expect_reply(2,8'h94,16,0);
        if(response_payload[15:8]!==31 || response_payload[31:24]!==127 || response_payload[39:32]!==128 ||
           response_payload[47:40]!==48 || response_payload[63:48]!==1600 ||
           response_payload[95:64]!==32'h12000 || response_payload[127:96]!==32'he000)
            $fatal(1,"Image telemetry byte layout");
        set_image(3,0);saved_token=image_token;
        set_image(3,0);
        if(image_token!==saved_token) $fatal(1,"Duplicate image request reapplied");
        request(3,8'h13,1,1,0);expect_reply(3,8'h93,1,7);
        set_image(3,0);
        set_image(4,1);set_image(5,2);set_image(6,3);
        if(owner || pending || token || target!=0) $fatal(1,"Image command changed mode ownership");
        request(7,8'h13,1,4,0);expect_reply(7,8'h93,1,3);
        request(8,8'h13,0,0,0);expect_reply(8,8'h93,1,2);
        request(9,8'h14,1,0,0);expect_reply(9,8'h94,1,2);
        previous=responses;request(10,8'h13,1,0,1);#30000;
        if(responses!=previous || image_flags!=3) $fatal(1,"Bad CRC changed image flags");
        request(11,8'h11,1,32,0);expect_reply(11,8'h91,2,0);
        request(12,8'h14,0,0,0);expect_reply(12,8'h94,16,0);
        if(response_payload[15:8]!=31 || response_payload[31:24]!=127) $fatal(1,"Display brightness changed camera telemetry");
        enable_pixel=0;#5000;
        request(20,8'h13,1,0,0);#1000;
        if(!dut.image_pending || pending) $fatal(1,"Image pending state");
        request(21,8'h10,1,5,0);expect_reply(21,8'h90,1,4);
        request(22,8'h11,1,40,0);expect_reply(22,8'h91,1,4);
        previous=responses;request(20,8'h13,1,0,0);#30000;
        if(responses!=previous) $fatal(1,"Pending repeat acknowledged early");
        request(20,8'h13,1,1,0);expect_reply(20,8'h93,1,7);
        enable_pixel=1;expect_reply(20,8'h93,2,0);
        enable_pixel=0;#5000;
        request(30,8'h13,1,3,0);expect_reply(30,8'h93,1,6);
        if(image_flags!=0 || dut.committed_image_flags!=0) $fatal(1,"Image timeout rollback");
        enable_pixel=1;#40000;
        if(awb_flags!=0 || ae_flags!=0 || awb_ack!=image_token || ae_ack!=image_token)
            $fatal(1,"Late frame ACK changed rolled-back setting");
        set_image(31,3);
        $display("PASS: UART image capability, telemetry, four settings, independent brightness, ACK timing, retries, BUSY, CRC and timeout rollback");
        $finish;
    end
    initial begin #60000000; $fatal(1,"Image UART timeout");end
endmodule
