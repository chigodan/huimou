// Passive telemetry: RAW/ISP-input SOF -> first HDMI pixel of the SAME DDR frame.
// 1 ms resolution; sensor exposure/MIPI transport before the FPGA input and
// the monitor's own display-panel scanout are outside this measurement.
// Four slot tags follow actual buffer selection. Incomplete frames show NA.
module video_realtime_monitor #(
    parameter integer REF_CLK_HZ = 100_000_000,
    parameter integer FRAME_WORDS = 172800, // 1280*720*24/128
    parameter integer FPS_WINDOW_MS = 1000,
    parameter integer STALE_MS = 250
)(
    input wire I_rst_n, I_ref_clk, I_camera_clk, I_ddr_clk, I_pixel_clk,
    input wire I_capture_start, I_processed_start,
    input wire I_write_frame_start,
    input wire [1:0] I_write_slot,
    input wire I_write_word,
    input wire [24:0] I_write_addr,
    input wire I_read_frame_start,
    input wire [1:0] I_read_slot,
    input wire I_display_start,
    output reg [9:0] O_capture_fps, O_display_fps, O_new_fps,
    output reg [15:0] O_capture_id, O_display_id, O_latency_ms,
    output reg O_latency_valid,
    output wire [1:0] O_video_state // 0 wait, 1 live, 2 stale, 3 tag error
);
    function [31:0] gray_decode;
        input [31:0] value;
        integer b;
        begin
            gray_decode[31]=value[31];
            for(b=30;b>=0;b=b-1) gray_decode[b]=gray_decode[b+1]^value[b];
        end
    endfunction
    function [9:0] fps_value;
        input [31:0] count;
        reg [63:0] rate;
        begin
            // Default hardware uses a true one-second window: no divider.
            if(FPS_WINDOW_MS==1000) rate={32'd0,count};
            else rate=(count*64'd1000)/FPS_WINDOW_MS;
            fps_value=(rate>999) ? 10'd999 : rate[9:0];
        end
    endfunction

    // A common, slowly changing Gray timebase avoids torn timestamps.
    localparam integer TICK_BITS=(REF_CLK_HZ/1000>1) ? $clog2(REF_CLK_HZ/1000) : 1;
    reg [TICK_BITS-1:0] tick_div;
    reg [31:0] time_ms, time_gray;
    wire ms_tick=(tick_div==REF_CLK_HZ/1000-1);
    wire [31:0] next_time=time_ms+1'b1;
    always @(posedge I_ref_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin tick_div<=0; time_ms<=0; time_gray<=0; end
        else if(ms_tick) begin
            tick_div<=0; time_ms<=next_time; time_gray<=next_time^(next_time>>1);
        end else tick_div<=tick_div+1'b1;
    end
    (* ASYNC_REG="TRUE" *) reg [31:0] cam_time_meta, cam_time_sync;
    (* ASYNC_REG="TRUE" *) reg [31:0] pix_time_meta, pix_time_sync;
    wire [31:0] camera_now=gray_decode(cam_time_sync);
    wire [31:0] pixel_now=gray_decode(pix_time_sync);
    always @(posedge I_camera_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin cam_time_meta<=0;cam_time_sync<=0; end
        else begin cam_time_meta<=time_gray;cam_time_sync<=cam_time_meta; end
    end
    always @(posedge I_pixel_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin pix_time_meta<=0;pix_time_sync<=0; end
        else begin pix_time_meta<=time_gray;pix_time_sync<=pix_time_meta; end
    end

    // SOF tags pass through a four-entry in-order metadata queue, independently
    // of the pixel pipeline latency of camera/edge/gray/fill. Never stall video.
    reg [31:0] capture_count, capture_gray;
    wire [31:0] next_capture=capture_count+1'b1;
    reg [63:0] capture_tags [0:3];
    reg [1:0] tag_wr, tag_rd;
    reg [2:0] tag_count;
    reg tag_fault, write_tag_valid;
    reg [63:0] write_tag;
    always @(posedge I_camera_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            capture_count<=0;capture_gray<=0;tag_wr<=0;tag_rd<=0;tag_count<=0;
            tag_fault<=0;write_tag_valid<=0;write_tag<=0;
        end else begin
            if(I_capture_start) begin
                capture_count<=next_capture;
                capture_gray<=next_capture^(next_capture>>1);
            end
            if(I_capture_start && !(I_processed_start && tag_count==0)) begin
                if(tag_count<4 || I_processed_start) begin
                    capture_tags[tag_wr]<={next_capture,camera_now};
                    tag_wr<=tag_wr+1'b1;
                end else tag_fault<=1'b1;
            end
            if(I_processed_start) begin
                write_tag_valid<=!tag_fault;
                if(tag_count!=0) begin
                    write_tag<=capture_tags[tag_rd];tag_rd<=tag_rd+1'b1;
                end else if(I_capture_start) write_tag<={next_capture,camera_now};
                else begin write_tag_valid<=0;tag_fault<=1'b1; end
            end
            case({I_capture_start,I_processed_start})
                2'b10: if(tag_count<4) tag_count<=tag_count+1'b1;
                2'b01: if(tag_count!=0) tag_count<=tag_count-1'b1;
                default: tag_count<=tag_count;
            endcase
        end
    end

    // write_tag is held for a whole camera frame. video_in's synchronized
    // write-start pulse follows processed SOF, so the bundled tag is settled.
    reg [31:0] slot_id [0:3], slot_stamp [0:3];
    localparam integer WORD_BITS=(FRAME_WORDS>1) ? $clog2(FRAME_WORDS+1) : 1;
    reg [WORD_BITS-1:0] slot_words [0:3];
    reg [3:0] slot_valid, slot_ready;
    wire [1:0] word_slot=(I_write_addr>=25'd21000000) ? 2'd3 :
                         (I_write_addr>=25'd14000000) ? 2'd2 :
                         (I_write_addr>=25'd7000000) ? 2'd1 : 2'd0;
    integer slot;
    always @(posedge I_ddr_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            slot_valid<=0;slot_ready<=0;
            for(slot=0;slot<4;slot=slot+1) begin
                slot_id[slot]<=0;slot_stamp[slot]<=0;slot_words[slot]<=0;
            end
        end else begin
            if(I_write_frame_start) begin
                slot_id[I_write_slot]<=write_tag[63:32];
                slot_stamp[I_write_slot]<=write_tag[31:0];
                slot_valid[I_write_slot]<=write_tag_valid;
                slot_ready[I_write_slot]<=0;slot_words[I_write_slot]<=0;
            end
            if(I_write_word && !(I_write_frame_start && word_slot==I_write_slot) &&
               slot_valid[word_slot] && slot_words[word_slot]<FRAME_WORDS) begin
                slot_words[word_slot]<=slot_words[word_slot]+1'b1;
                if(slot_words[word_slot]==FRAME_WORDS-1) slot_ready[word_slot]<=1'b1;
            end
        end
    end
    wire read_valid=slot_valid[I_read_slot] && slot_ready[I_read_slot] &&
                    !(I_write_frame_start && I_write_slot==I_read_slot);
    wire read_event;
    wire [64:0] read_snapshot;
    uart_snapshot_mailbox #(.WIDTH(65)) u_read_tag (
        .src_clk(I_ddr_clk),.src_rst_n(I_rst_n),.src_valid(I_read_frame_start),
        .src_data({read_valid,slot_id[I_read_slot],slot_stamp[I_read_slot]}),
        .dst_clk(I_pixel_clk),.dst_rst_n(I_rst_n),
        .dst_valid(read_event),.dst_data(read_snapshot));

    reg [64:0] saved_read_tag;
    reg read_pending, have_display_id;
    reg [31:0] last_display_id, display_count, display_gray, new_count, new_gray;
    wire [64:0] display_tag=read_event ? read_snapshot : saved_read_tag;
    wire tag_available=(read_pending || read_event) && display_tag[64];
    wire [31:0] frame_age=pixel_now-display_tag[31:0]; // Modular subtraction.
    wire display_valid=tag_available && frame_age<STALE_MS;
    wire [31:0] next_display=display_count+1'b1;
    wire [31:0] next_new=new_count+1'b1;
    always @(posedge I_pixel_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            saved_read_tag<=0;read_pending<=0;have_display_id<=0;last_display_id<=0;
            display_count<=0;display_gray<=0;new_count<=0;new_gray<=0;
            O_display_id<=0;O_latency_ms<=0;O_latency_valid<=0;
        end else begin
            if(read_event) begin saved_read_tag<=read_snapshot;read_pending<=1; end
            if(I_display_start) begin
                display_count<=next_display;display_gray<=next_display^(next_display>>1);
                read_pending<=0;O_latency_valid<=display_valid;
                O_latency_ms<=display_valid ? frame_age[15:0] : 16'd0;
                if(tag_available) O_display_id<=display_tag[47:32];
                if(display_valid && (!have_display_id || last_display_id!=display_tag[63:32])) begin
                    new_count<=next_new;new_gray<=next_new^(next_new>>1);
                    last_display_id<=display_tag[63:32];have_display_id<=1;
                end
            end
        end
    end

    // FPS counts actual frame events in a reference-clock 1-second window.
    // D is HDMI refresh; N counts changes of the displayed source frame ID.
    (* ASYNC_REG="TRUE" *) reg [31:0] cap_meta, cap_sync, out_meta, out_sync, new_meta, new_sync;
    (* ASYNC_REG="TRUE" *) reg fault_meta, fault_sync;
    wire [31:0] cap_now=gray_decode(cap_sync), out_now=gray_decode(out_sync), new_now=gray_decode(new_sync);
    reg [31:0] cap_previous, out_previous, new_previous, last_capture_seen;
    reg [15:0] fps_window, capture_age;
    reg [9:0] capture_fps, display_fps, new_fps;
    reg have_capture;
    always @(posedge I_ref_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            cap_meta<=0;cap_sync<=0;out_meta<=0;out_sync<=0;new_meta<=0;new_sync<=0;
            fault_meta<=0;fault_sync<=0;cap_previous<=0;out_previous<=0;new_previous<=0;
            last_capture_seen<=0;fps_window<=0;capture_age<=STALE_MS;have_capture<=0;
            capture_fps<=0;display_fps<=0;new_fps<=0;
        end else begin
            cap_meta<=capture_gray;cap_sync<=cap_meta;
            out_meta<=display_gray;out_sync<=out_meta;new_meta<=new_gray;new_sync<=new_meta;
            fault_meta<=tag_fault;fault_sync<=fault_meta;
            if(cap_now!=last_capture_seen) begin
                last_capture_seen<=cap_now;capture_age<=0;have_capture<=1;
            end else if(ms_tick && capture_age<STALE_MS) capture_age<=capture_age+1'b1;
            if(ms_tick) begin
                if(fps_window==FPS_WINDOW_MS-1) begin
                    fps_window<=0;
                    capture_fps<=fps_value(cap_now-cap_previous);
                    display_fps<=fps_value(out_now-out_previous);
                    new_fps<=fps_value(new_now-new_previous);
                    cap_previous<=cap_now;out_previous<=out_now;new_previous<=new_now;
                end else fps_window<=fps_window+1'b1;
            end
        end
    end
    wire report_event;
    wire [48:0] report;
    uart_snapshot_mailbox #(.WIDTH(49)) u_rates (
        .src_clk(I_ref_clk),.src_rst_n(I_rst_n),.src_valid(ms_tick),
        .src_data({fault_sync,have_capture,(have_capture && capture_age<STALE_MS),
                   cap_now[15:0],capture_fps,display_fps,new_fps}),
        .dst_clk(I_pixel_clk),.dst_rst_n(I_rst_n),.dst_valid(report_event),.dst_data(report));
    reg video_have_capture, video_live, video_fault;
    assign O_video_state=video_fault ? 2'd3 : !video_have_capture ? 2'd0 :
                         !video_live ? 2'd2 : O_latency_valid ? 2'd1 : 2'd0;
    always @(posedge I_pixel_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            O_capture_fps<=0;O_display_fps<=0;O_new_fps<=0;O_capture_id<=0;
            video_have_capture<=0;video_live<=0;video_fault<=0;
        end else if(report_event) begin
            video_fault<=report[48];video_have_capture<=report[47];video_live<=report[46];
            O_capture_id<=report[45:30];O_capture_fps<=report[29:20];
            O_display_fps<=report[19:10];O_new_fps<=report[9:0];
        end
    end
endmodule
