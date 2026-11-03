`timescale 1ns/1ps

// Held command bus + toggle, applied only at a frame boundary. The following
// boundary acknowledges that downstream sequential consumers saw the setting.
module image_frame_settings (
    input wire clk, rst_n, frame_start,
    input wire token_async,
    input wire [1:0] flags_async,
    output reg [1:0] flags,
    output reg ack
);
    (* ASYNC_REG="TRUE" *) reg token_meta, token_sync;
    reg [1:0] flags_meta, flags_sync;
    reg seen, settle;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            token_meta<=0;token_sync<=0;flags_meta<=3;flags_sync<=3;
            flags<=3;ack<=0;seen<=0;settle<=0;
        end else begin
            token_meta<=token_async;token_sync<=token_meta;
            flags_meta<=flags_async;flags_sync<=flags_meta;
            if (frame_start) begin
                if (token_sync!=seen) begin
                    flags<=flags_sync;seen<=token_sync;settle<=1;
                end else if (settle) begin ack<=seen;settle<=0;end
            end
        end
    end
endmodule

// Pre-AWB RGB statistics. Sample lane zero once per 16 valid four-pixel beats
// (1/64 of the pixels). Neither AWB nor display-brightness edits affect AE.
// One 24/16 iterative divider runs only after a completed frame: no pixel stall.
module isp_luma_stats #(parameter integer MIN_SAMPLES=256)(
    input wire clk, rst_n, frame_start, valid,
    input wire [95:0] rgb,
    output reg sample_valid,
    output reg [7:0] mean_luma
);
    wire [9:0] luma_sum={2'b0,rgb[23:16]}+{1'b0,rgb[15:8],1'b0}+{2'b0,rgb[7:0]};
    wire [7:0] luma=luma_sum[9:2];
    reg [3:0] phase;
    reg [23:0] sum;
    reg [15:0] count;
    reg busy;
    reg [4:0] bit_count;
    reg [23:0] dividend, quotient;
    reg [15:0] denominator;
    reg [16:0] remainder;
    wire [16:0] shifted={remainder[15:0],dividend[23]};
    wire take_bit=shifted>={1'b0,denominator};
    wire [16:0] next_remainder=take_bit ? shifted-{1'b0,denominator} : shifted;
    wire [23:0] next_quotient={quotient[22:0],take_bit};
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            phase<=0;sum<=0;count<=0;busy<=0;bit_count<=0;
            dividend<=0;quotient<=0;denominator<=1;remainder<=0;
            sample_valid<=0;mean_luma<=0;
        end else begin
            sample_valid<=0;
            if (frame_start) begin
                sum<=valid ? {16'd0,luma} : 24'd0;
                count<=valid ? 16'd1 : 16'd0;
                phase<=valid ? 4'd1 : 4'd0;
                if (!busy && count>=MIN_SAMPLES && count!=0) begin
                    busy<=1;bit_count<=0;dividend<=sum;denominator<=count;
                    quotient<=0;remainder<=0;
                end
            end else if (valid) begin
                phase<=phase+1'b1;
                if (phase==0 && count!=16'hffff) begin
                    sum<=sum+{16'd0,luma};count<=count+1'b1;
                end
            end
            if (busy) begin
                dividend<={dividend[22:0],1'b0};quotient<=next_quotient;
                remainder<=next_remainder;
                if (bit_count==23) begin
                    busy<=0;sample_valid<=1;mean_luma<=next_quotient[7:0];
                end else bit_count<=bit_count+1'b1;
            end
        end
    end
endmodule

// AE operates in the sensor-configuration clock domain. Requests and payloads
// stay stable through the real I2C busy/done handshake. Three subsequent frame
// statistics are discarded after each write, allowing exposure/group hold and
// the pre-AWB statistics pipeline to settle. Disabling AE freezes sensor settings.
module camera_auto_exposure #(
    parameter integer CLK_HZ=23_809_524,
    parameter integer TARGET_LUMA=128,
    parameter integer DEADBAND=8,
    parameter integer MIN_EXPOSURE=8,
    parameter integer MAX_EXPOSURE=1991,
    parameter integer MIN_GAIN=16,
    parameter integer MAX_GAIN=176,
    parameter integer INITIAL_EXPOSURE=1991,
    parameter integer INITIAL_GAIN=144,
    parameter integer SETTLE_FRAMES=3,
    parameter integer WRITE_TIMEOUT_MS=500,
    parameter integer STATS_TIMEOUT_MS=250
)(
    input wire clk, rst_n, enabled,
    input wire sensor_ready, write_ready, i2c_error,
    input wire sample_valid,
    input wire [7:0] mean_luma,
    output reg request,
    output reg [15:0] exposure, gain,
    output reg [15:0] applied_exposure,
    output reg [7:0] applied_gain,
    output reg applied_valid, mean_valid,
    output reg [7:0] measured_luma,
    output reg [2:0] state, // WAIT/ADJUST/STABLE/DARK_LIMIT/BRIGHT_LIMIT/OFF/FAULT
    output wire report_tick
);
    localparam integer MS_CYCLES=(CLK_HZ/1000>0) ? CLK_HZ/1000 : 1;
    localparam integer TICK_BITS=(MS_CYCLES>1) ? $clog2(MS_CYCLES) : 1;
    reg [TICK_BITS-1:0] ticks;
    wire ms_tick=(ticks==MS_CYCLES-1);
    assign report_tick=ms_tick;
    reg initialized, fault;
    reg [1:0] pending;
    reg [9:0] write_age, stats_age;
    reg [3:0] settling;
    reg [15:0] proposed_exposure, proposed_gain, step;
    // One combinational proposal; all changes use the last acknowledged values.
    always @(*) begin
        proposed_exposure=exposure;proposed_gain=gain;step=1;
        if (mean_luma<TARGET_LUMA-DEADBAND) begin
            if (exposure<MAX_EXPOSURE) begin
                step=(mean_luma+40<TARGET_LUMA) ? (exposure>>2) : (exposure>>4);
                if (step==0) step=1;
                proposed_exposure=(exposure+step>MAX_EXPOSURE) ? MAX_EXPOSURE : exposure+step;
            end else if (gain<MAX_GAIN)
                proposed_gain=(gain+2>MAX_GAIN) ? MAX_GAIN : gain+2;
        end else if (mean_luma>TARGET_LUMA+DEADBAND) begin
            if (gain>MIN_GAIN) begin
                step=(mean_luma>TARGET_LUMA+40) ? 8 : 2;
                proposed_gain=(gain<MIN_GAIN+step) ? MIN_GAIN : gain-step;
            end else if (exposure>MIN_EXPOSURE) begin
                step=(mean_luma>TARGET_LUMA+40) ? (exposure>>2) : (exposure>>4);
                if (step==0) step=1;
                proposed_exposure=(exposure<MIN_EXPOSURE+step) ? MIN_EXPOSURE : exposure-step;
            end
        end
    end
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ticks<=0;initialized<=0;fault<=0;pending<=0;write_age<=0;
            stats_age<=STATS_TIMEOUT_MS;settling<=0;request<=0;
            exposure<=INITIAL_EXPOSURE;gain<=INITIAL_GAIN;
            applied_exposure<=0;applied_gain<=0;applied_valid<=0;
            mean_valid<=0;measured_luma<=0;state<=0;
        end else begin
            request<=0;
            if (ms_tick) begin
                ticks<=0;
                if (stats_age<STATS_TIMEOUT_MS) stats_age<=stats_age+1'b1;
                if (pending!=0 && write_age<WRITE_TIMEOUT_MS) write_age<=write_age+1'b1;
                if (stats_age>=STATS_TIMEOUT_MS-1) mean_valid<=0;
            end else ticks<=ticks+1'b1;
            if (sample_valid) begin
                measured_luma<=mean_luma;mean_valid<=1;stats_age<=0;
            end
            if (i2c_error || fault || (pending!=0 && write_age>=WRITE_TIMEOUT_MS)) begin
                fault<=1;pending<=0;state<=6;applied_valid<=0;
            end else if (!sensor_ready) begin
                state<=0;
            end else if (!initialized) begin
                state<=0;
                // Always establish a known initial exposure/gain after the ROM.
                if (write_ready) begin
                    initialized<=1;request<=1;pending<=1;write_age<=0;state<=1;
                end
            end else if (pending==1) begin
                state<=enabled ? 3'd1 : 3'd5;
                if (!write_ready) pending<=2;
            end else if (pending==2) begin
                state<=enabled ? 3'd1 : 3'd5;
                if (write_ready) begin
                    pending<=0;applied_exposure<=exposure;applied_gain<=gain[7:0];
                    applied_valid<=1;settling<=SETTLE_FRAMES;
                end
            end else if (!enabled) begin
                state<=5;
            end else if (sample_valid) begin
                if (settling!=0) begin settling<=settling-1'b1;state<=1;end
                else if (proposed_exposure!=exposure || proposed_gain!=gain) begin
                    exposure<=proposed_exposure;gain<=proposed_gain;
                    request<=1;pending<=1;write_age<=0;state<=1;
                end else if (mean_luma<TARGET_LUMA-DEADBAND) state<=3;
                else if (mean_luma>TARGET_LUMA+DEADBAND) state<=4;
                else state<=2;
            end else if (!mean_valid) state<=0;
        end
    end
endmodule
