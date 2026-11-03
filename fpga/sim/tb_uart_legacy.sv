`timescale 1ns/1ps
module tb_uart_legacy;
    reg clk=0, isp_clk=0, pixel_clk=0;
    always #10 clk=~clk;
    always #7 isp_clk=~isp_clk;
    always #13 pixel_clk=~pixel_clk;
    reg rst_n=0, rx=1, enable_isp=1, enable_pixel=1;
    wire tx, ms_tick, owner, pending, token;
    wire [2:0] target;
    reg isp_frame=0, pixel_frame=0;
    integer isp_count=0, pixel_count=0;
    always @(posedge isp_clk) begin
        isp_frame<=0;
        if(isp_count==149) begin isp_count<=0;isp_frame<=enable_isp; end
        else isp_count<=isp_count+1;
    end
    always @(posedge pixel_clk) begin
        pixel_frame<=0;
        if(pixel_count==149) begin pixel_count<=0;pixel_frame<=enable_pixel; end
        else pixel_count<=pixel_count+1;
    end
    wire isp_ack, pixel_ack, io_owner, po_owner;
    wire [2:0] io_algo, po_algo;
    uart_frame_command #(.SETTLE_FRAMES(2)) isp (
        .clk(isp_clk),.rst_n(rst_n),.frame_start(isp_frame),
        .override_async(owner||pending),.token_async(token),.algo_async(target),
        .owner(io_owner),.algo(io_algo),.ack(isp_ack));
    uart_frame_command #(.SETTLE_FRAMES(1)) pixel (
        .clk(pixel_clk),.rst_n(rst_n),.frame_start(pixel_frame),
        .override_async(owner||pending),.token_async(token),.algo_async(target),
        .owner(po_owner),.algo(po_algo),.ack(pixel_ack));
    reg fresh=1, mismatch_token=0;
    reg [15:0] age=8;
    uart_control #(.CLK_HZ(1_000_000),.BAUD(100000)) dut (
        .clk(clk),.rst_n(rst_n),.rx(rx),.tx(tx),.ms_tick(ms_tick),
        .owner(owner),.pending(pending),.token(token),.target_algo(target),
        .brightness_ack(1'b0),.isp_ack(isp_ack),.display_ack(pixel_ack),.observed_algo(3'd0),
        .video_alive(1'b1),.result_fresh(fresh),.result_token(token^mismatch_token),
        .result_mode(2'b11),.center_x(12'd640),.center_y(12'd360),
        .sample_age(age),.frame_id(32'd123),.sample_id(32'd42));
    reg [7:0] vectors [0:2559];
    reg [7:0] lengths [0:63];
    reg [7:0] packet [0:39];
    integer response_count=0, received_bytes=0, expected_bytes=0, log_file;
    reg [7:0] response_seq, response_cmd, response_len;
    reg [255:0] response_payload;
    reg [7:0] decoded_byte;
    integer bit_number, j;
    // Decode actual TX bits independently of the design's UART RX/parser.
    initial begin
        log_file=$fopen("uart_responses.hex","w");
        forever begin
            @(negedge tx); #100;
            if(tx!==0) $fatal(1,"invalid TX start");
            for(bit_number=0;bit_number<8;bit_number=bit_number+1) begin
                #200;decoded_byte[bit_number]=tx;
            end
            #200;if(tx!==1) $fatal(1,"invalid TX stop");
            packet[received_bytes]=decoded_byte;
            if(received_bytes==0 && decoded_byte!=8'haa) $fatal(1,"TX header AA");
            if(received_bytes==1 && decoded_byte!=8'h55) $fatal(1,"TX header 55");
            if(received_bytes==5) expected_bytes=8+decoded_byte;
            received_bytes=received_bytes+1;
            if(expected_bytes!=0 && received_bytes==expected_bytes) begin
                response_seq=packet[3];response_cmd=packet[4];response_len=packet[5];
                response_payload=0;
                for(j=0;j<response_len;j=j+1) response_payload[j*8+:8]=packet[6+j];
                for(j=0;j<expected_bytes;j=j+1) $fwrite(log_file,"%02x ",packet[j]);
                $fwrite(log_file,"\n");$fflush(log_file);
                response_count=response_count+1;received_bytes=0;expected_bytes=0;
            end
        end
    end
    task send_byte(input [7:0] value);
        integer b;
        begin
            @(negedge clk);rx=0;#200;
            for(b=0;b<8;b=b+1) begin rx=value[b];#200;end
            rx=1;#200;
        end
    endtask
    task send_vector(input integer n);
        integer b;
        begin for(b=0;b<lengths[n];b=b+1) send_byte(vectors[n*40+b]);end
    endtask
    task expect_reply(input [7:0] seq_value,cmd_value,len_value,result_value);
        integer previous,tries;
        begin
            previous=response_count;tries=0;
            while(response_count==previous && tries<1000000) begin @(negedge clk);tries=tries+1;end
            if(response_count!=previous+1) $fatal(1,"response timeout seq=%0d",seq_value);
            if(response_seq!==seq_value || response_cmd!==cmd_value ||
               response_len!==len_value || response_payload[7:0]!==result_value)
                $fatal(1,"bad reply seq=%h cmd=%h len=%h result=%h expected %h/%h/%h/%h",
                    response_seq,response_cmd,response_len,response_payload[7:0],
                    seq_value,cmd_value,len_value,result_value);
        end
    endtask
    integer i,previous;
    reg saved_token;
    initial begin
        $readmemh("uart_requests.hex",vectors);$readmemh("uart_lengths.hex",lengths);
        repeat(10) @(negedge clk);rst_n=1;
        send_vector(0);expect_reply(1,8'h81,8,0);
        if(response_payload[31:16]!=16'h001b || owner) $fatal(1,"PING capabilities/ownership");
        send_vector(1);expect_reply(2,8'h90,2,0);
        if(!owner || pending || target!=2 || io_algo!=2 || po_algo!=2) $fatal(1,"gray did not apply");
        saved_token=token;
        send_vector(2);expect_reply(2,8'h90,2,0);
        if(token!=saved_token) $fatal(1,"duplicate re-applied mode");
        send_vector(3);expect_reply(2,8'h90,1,7);
        send_vector(2);expect_reply(2,8'h90,2,0);
        send_vector(4);expect_reply(3,8'h82,28,0);
        if(response_payload[15:8]!=2 || response_payload[143:128]!=16'hffff) $fatal(1,"status gray");
        for(i=0;i<6;i=i+1) begin
            send_vector(5+i);expect_reply(10+i,8'h90,2,0);
            if(response_payload[15:8]!=i || target!=i) $fatal(1,"algorithm %0d",i);
        end
        send_vector(11);expect_reply(20,8'h82,28,0);
        if(!response_payload[25] || response_payload[143:128]!=640 ||
           response_payload[159:144]!=360) $fatal(1,"valid circle status");
        fresh=0;send_vector(12);expect_reply(21,8'h82,28,0);
        if(response_payload[25] || response_payload[143:128]!=16'hffff) $fatal(1,"lost target status");
        fresh=1;age=300;send_vector(13);expect_reply(22,8'h82,28,0);
        if(response_payload[25]) $fatal(1,"stale result reported valid");
        age=8;mismatch_token=1;send_vector(14);expect_reply(23,8'h82,28,0);
        if(response_payload[25]) $fatal(1,"old generation reported valid");
        mismatch_token=0;previous=response_count;send_vector(15);#30000;
        if(response_count!=previous || target!=5) $fatal(1,"bad CRC executed");
        send_vector(16);expect_reply(31,8'h81,8,0);
        send_vector(17);expect_reply(32,8'h81,1,5);
        send_vector(18);expect_reply(33,8'h90,1,2);
        send_vector(19);expect_reply(34,8'h90,1,3);
        send_vector(20);expect_reply(35,8'ha0,1,1);
        enable_isp=0;enable_pixel=0;#5000;
        send_vector(21);#1000;if(!pending) $fatal(1,"SET not pending");
        send_vector(22);expect_reply(41,8'h90,1,4);
        previous=response_count;send_vector(23);#30000;
        if(response_count!=previous) $fatal(1,"pending duplicate should wait");
        send_vector(24);expect_reply(42,8'h82,28,0);
        if(response_payload[23:16]!=2 || !response_payload[28]) $fatal(1,"pending status");
        enable_isp=1;enable_pixel=1;expect_reply(40,8'h90,2,0);
        enable_isp=0;#5000;send_vector(25);#10000;enable_pixel=0;
        expect_reply(50,8'h90,1,6);
        if(target!=2 || pending || dut.committed_algo!=2) $fatal(1,"timeout rollback");
        saved_token=token;send_vector(26);expect_reply(50,8'h90,1,6);
        if(token!=saved_token) $fatal(1,"timeout repeat re-executed");
        send_vector(27);expect_reply(51,8'h90,1,4);
        enable_isp=1;enable_pixel=1;#15000;
        send_vector(28);expect_reply(52,8'h90,2,0);
        // Expire an incomplete frame, then recover to a fresh PING.
        send_byte(8'haa);send_byte(8'h55);send_byte(1);send_byte(1);send_byte(8'h10);
        #2100000;send_vector(29);expect_reply(60,8'h81,8,0);
        send_vector(30);expect_reply(0,8'h81,1,3);
        send_vector(31);expect_reply(61,8'hff,1,1);
        send_byte(8'haa);send_byte(8'h55);send_byte(1);send_byte(1);send_byte(8'h10);send_byte(8'hff);
        send_vector(32);expect_reply(62,8'h81,8,0);
        // Two queued STATUS replies straddle the application deadline. Restore
        // frame clocks AFTER the deadline; the deferred SET reply must stay an error.
        enable_isp=0;enable_pixel=0;#5000;send_vector(33);
        wait(dut.apply_age==398);
        send_vector(34);send_vector(35);
        fork
            begin wait(dut.apply_age==400);#20000;enable_isp=1;enable_pixel=1;end
            begin
                expect_reply(71,8'h82,28,0);
                expect_reply(72,8'h82,28,0);
                expect_reply(70,8'h90,1,6);
            end
        join
        if(dut.committed_algo!=3 || target!=3) $fatal(1,"late ACK committed expired mode");
        $display("PASS: UART wire-level control, six algorithms, status, CRC rejection, duplicates, busy, timeout and recovery (%0d replies)",response_count);
        $fclose(log_file);$finish;
    end
    initial begin #100000000;$fatal(1,"watchdog");end
endmodule
