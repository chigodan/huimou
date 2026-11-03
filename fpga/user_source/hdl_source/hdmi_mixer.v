module hdmi_mixer #(
    parameter H_OFFSET = 128,
    parameter V_OFFSET = 60,
    parameter IMG_WIDTH = 1280,
    parameter IMG_HEIGHT = 720,
    parameter integer DEBUG_MODE = 0,
    parameter integer STATUS_OSD_ENABLE = 1,
    parameter integer TELEMETRY_OSD_ENABLE = 1,
    parameter integer TARGET_OSD_ENABLE = 1,
    parameter integer IMAGE_OSD_ENABLE = 0,
    // Circle detector segmentation:
    //   0: dark object on a bright background (default)
    //   1: bright object on a dark background
    //   2: highly saturated coloured object
    parameter integer CIRCLE_SEGMENT_MODE = 0,
    // A lower default avoids selecting grey shadows and phone bezels as
    // black.  Tune in the 35..85 range for the actual camera exposure.
    parameter [7:0] CIRCLE_LUMA_THRESHOLD = 8'd55,
    parameter [7:0] CIRCLE_SAT_THRESHOLD  = 8'd55,
    parameter integer CIRCLE_MIN_DIAMETER = 20,
    parameter integer CIRCLE_MAX_DIAMETER = 650,
    parameter integer CIRCLE_MIN_AREA     = 250,
    // Ignore very short threshold runs when allocating a new blob.  Six
    // pixels still preserves the cap of a 20-pixel circle, while preventing
    // isolated dark texture from consuming all tracker slots.
    parameter integer CIRCLE_MIN_RUN      = 6
)(
    input wire        I_clk,
    input wire        I_rst_n,
    input wire[1:0]   I_mode,
    // Coherent ISP report already transferred into I_clk at the top level:
    // 1=edge, 2=grayscale, 3=region fill. Other modes use S_mode_latched.
    input wire[2:0]   I_process_algo,
    // Passive real-time monitor outputs, all in the HDMI clock domain.
    input wire [9:0] I_capture_fps, I_display_fps, I_new_fps,
    input wire [15:0] I_capture_id, I_display_id, I_latency_ms,
    input wire I_latency_valid,
    input wire [1:0] I_video_state,
    input wire [1:0] I_image_flags,
    input wire [2:0] I_ae_state,
    input wire        I_uart_override,
    input wire        I_uart_token,
    input wire[2:0]   I_uart_algo,
    output wire       O_uart_ack,
    output wire[1:0]  O_uart_report_mode,

    input wire        I_video_vsync,
    input wire        I_video_hsync,
    input wire        I_video_de,
    input wire        I_video_user,
    input wire        I_video_last,

    input wire[3:0]   I_debug_status,

    output wire       O_video_rd_en,
    input wire[23:0]  I_video_rd_data,

    output reg        O_hdmi_vsync,
    output reg        O_hdmi_hsync,
    output reg        O_hdmi_de,
    output reg[23:0]  O_hdmi_data,
    // One result packet per frame, including invalid/missing-target frames.
    // The on-screen box may be retained; that box is NOT a fresh measurement.
    output reg        O_circle_sample,
    output wire       O_circle_fresh_valid,
    output wire[11:0] O_circle_center_x,
    output wire[11:0] O_circle_center_y
);

    reg        S_video_vsync_1d;
    reg        S_video_vsync_2d;
    reg        S_video_hsync_1d;
    reg        S_video_hsync_2d;
    reg        S_video_de_1d;
    reg        S_video_de_2d;
    reg [11:0] S_x;
    reg [11:0] S_y;
    reg [11:0] S_x_1d;
    reg [11:0] S_x_2d;
    reg [11:0] S_y_1d;
    reg [11:0] S_y_2d;

    // Synchronize the asynchronous board switches into the HDMI pixel-clock
    // domain.  A requested mode is applied only at the start of a new frame.
    reg [1:0]  S_mode_meta;
    reg [1:0]  S_mode_sync;
    reg [1:0]  S_mode_latched;
    reg [2:0]  S_process_algo_latched;
    wire uart_frame_owner;
    wire [2:0] uart_frame_algo;
    uart_frame_command #(.SETTLE_FRAMES(1)) u_uart_frame_command (
        .clk(I_clk),.rst_n(I_rst_n),.frame_start(I_video_user),
        .override_async(I_uart_override),.token_async(I_uart_token),.algo_async(I_uart_algo),
        .owner(uart_frame_owner),.algo(uart_frame_algo),.ack(O_uart_ack));
    assign O_uart_report_mode=S_mode_latched;

    // Persistent upper-right status panel. It is composed after detection,
    // so the characters never enter the circle detector or servo feedback.
    // Small diagnostic frames do not have room for this full-size panel.
    // SimSun size 5: 14 px/em, native pixels, 8-pixel Latin advance and
    // 16-pixel line spacing. Eighteen columns plus 8-pixel padding per side.
    localparam integer STATUS_W = 160;
    localparam integer STATUS_X = IMG_WIDTH - STATUS_W - 16;
    localparam integer STATUS_Y = 16;
    localparam integer STATUS_H = (TELEMETRY_OSD_ENABLE != 0) ? ((IMAGE_OSD_ENABLE != 0) ? 112 : 96) : 32;
    // 25% neutral grey over 75% live video. Blend each RGB channel separately
    // so carries cannot spill into a neighbouring colour channel.
    localparam [23:0] STATUS_BG = 24'h808080;
    function [23:0] F_panel_background;
        input [23:0] pixel;
        reg [9:0] r,g,b;
        begin
            r={1'b0,pixel[23:16],1'b0}+{2'b0,pixel[23:16]}+{2'b0,STATUS_BG[23:16]}+10'd2;
            g={1'b0,pixel[15:8],1'b0}+{2'b0,pixel[15:8]}+{2'b0,STATUS_BG[15:8]}+10'd2;
            b={1'b0,pixel[7:0],1'b0}+{2'b0,pixel[7:0]}+{2'b0,STATUS_BG[7:0]}+10'd2;
            F_panel_background={r[9:2],g[9:2],b[9:2]};
        end
    endfunction
    wire [23:0] S_panel_background=F_panel_background(I_video_rd_data);
    localparam [23:0] STATUS_FG = 24'h00ff00;
    wire S_status_panel = (STATUS_OSD_ENABLE != 0) && (DEBUG_MODE == 0) &&
        (IMG_WIDTH >= 640) && (IMG_HEIGHT >= STATUS_Y+STATUS_H) && S_video_de_2d &&
        (S_x_2d >= STATUS_X) && (S_x_2d < STATUS_X + STATUS_W) &&
        (S_y_2d >= STATUS_Y) && (S_y_2d < STATUS_Y + STATUS_H);
    wire S_status_text_area = S_status_panel &&
        (S_x_2d >= STATUS_X + 8) && (S_x_2d < STATUS_X + 152) &&
        (S_y_2d >= STATUS_Y + 8) && (S_y_2d < STATUS_Y + STATUS_H - 8);
    wire [11:0] S_status_local_x = S_x_2d - (STATUS_X + 8);
    wire [11:0] S_status_local_y = S_y_2d - (STATUS_Y + 8);
    wire [4:0] S_status_col = S_status_local_x[7:3];
    reg [143:0] S_status_text;
    reg [143:0] S_telemetry_text;
    wire [2:0] S_status_row=S_status_local_y[6:4];
    wire [143:0] S_status_line=(TELEMETRY_OSD_ENABLE != 0 && S_status_row!=0) ?
                               S_telemetry_text : S_status_text;
    reg [9:0] S_capture_fps_latched, S_display_fps_latched, S_new_fps_latched;
    reg [15:0] S_capture_id_latched, S_display_id_latched, S_latency_latched;
    wire [11:0] S_capture_fps_bcd, S_display_fps_bcd, S_new_fps_bcd;
    wire [15:0] S_latency_bcd, S_target_x_bcd, S_target_y_bcd;
    reg S_decimal_start;
    reg S_latency_valid_latched;
    reg [1:0] S_video_state_latched;
    reg [1:0] S_image_flags_latched;
    reg [2:0] S_ae_state_latched;
    reg [3:0] S_status_frame_delay;
    reg [7:0] S_status_char;
    wire [15:0] S_status_bits;
    wire S_status_ink = S_status_text_area &&
                        S_status_bits[4'd15 - {1'b0,S_status_local_x[2:0]}];
    always @(*) begin
        case(S_mode_latched)
            2'b00: S_status_text = "MODE: CAMERA      ";
            2'b10: S_status_text = "MODE: OSD         ";
            2'b11: S_status_text = "MODE: CIRCLE TRACK";
            default: begin
                case(S_process_algo_latched)
                    3'd2: S_status_text = "MODE: GRAYSCALE   ";
                    3'd3: S_status_text = "MODE: REGION FILL ";
                    default: S_status_text = "MODE: EDGE        ";
                endcase
            end
        endcase
    end
    function [7:0] F_status_digit;
        input [3:0] value;
        begin F_status_digit=8'h30+value; end
    endfunction
    function [7:0] F_status_hex;
        input [3:0] value;
        begin F_status_hex=(value<10) ? 8'h30+value : 8'h41+(value-10); end
    endfunction
    // Snapshot after the monitor measures the first output pixel. The text
    // starts 16 lines down, so all five rows are stable before any are drawn.
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            S_status_frame_delay<=0;S_capture_fps_latched<=0;S_display_fps_latched<=0;
            S_new_fps_latched<=0;S_capture_id_latched<=0;S_display_id_latched<=0;
            S_latency_latched<=0;S_latency_valid_latched<=0;S_video_state_latched<=0;
            S_decimal_start<=0;S_image_flags_latched<=3;S_ae_state_latched<=0;
        end else begin
            S_status_frame_delay<={S_status_frame_delay[2:0],I_video_user};
            S_decimal_start<=S_status_frame_delay[3];
            if(S_status_frame_delay[3]) begin
                S_capture_fps_latched<=I_capture_fps;
                S_display_fps_latched<=I_display_fps;S_new_fps_latched<=I_new_fps;
                S_capture_id_latched<=I_capture_id;S_display_id_latched<=I_display_id;
                S_latency_latched<=I_latency_ms;S_latency_valid_latched<=I_latency_valid;
                S_video_state_latched<=I_video_state;
                S_image_flags_latched<=I_image_flags;S_ae_state_latched<=I_ae_state;
            end
        end
    end
    always @(*) begin
        S_telemetry_text="                  ";
        case(S_status_row)
            1: S_telemetry_text={"FPS C",
                F_status_digit(S_capture_fps_bcd[11:8]),F_status_digit(S_capture_fps_bcd[7:4]),F_status_digit(S_capture_fps_bcd[3:0]),
                " D",F_status_digit(S_display_fps_bcd[11:8]),F_status_digit(S_display_fps_bcd[7:4]),F_status_digit(S_display_fps_bcd[3:0]),
                " N",F_status_digit(S_new_fps_bcd[11:8]),F_status_digit(S_new_fps_bcd[7:4]),F_status_digit(S_new_fps_bcd[3:0])};
            2: begin
                if(S_latency_valid_latched)
                    S_telemetry_text={"LATENCY: ",F_status_digit(S_latency_bcd[15:12]),
                        F_status_digit(S_latency_bcd[11:8]),F_status_digit(S_latency_bcd[7:4]),
                        F_status_digit(S_latency_bcd[3:0])," MS  "};
                else S_telemetry_text="LATENCY: NA       ";
            end
            3: if (IMAGE_OSD_ENABLE!=0)
                S_telemetry_text={"AWB:",S_image_flags_latched[0] ? "ON " : "OFF",
                    " AE:",S_image_flags_latched[1] ? "ON " : "OFF","    "};
               else S_telemetry_text={"ID HEX C",F_status_hex(S_capture_id_latched[15:12]),F_status_hex(S_capture_id_latched[11:8]),
                F_status_hex(S_capture_id_latched[7:4]),F_status_hex(S_capture_id_latched[3:0])," D",
                F_status_hex(S_display_id_latched[15:12]),F_status_hex(S_display_id_latched[11:8]),
                F_status_hex(S_display_id_latched[7:4]),F_status_hex(S_display_id_latched[3:0])};
            4: begin
                case(S_video_state_latched)
                    1: S_telemetry_text="VIDEO: LIVE       ";
                    2: S_telemetry_text="VIDEO: STALE      ";
                    3: S_telemetry_text="VIDEO: TAG ERROR  ";
                    default: S_telemetry_text="VIDEO: WAIT       ";
                endcase
            end
            5: if (IMAGE_OSD_ENABLE!=0) begin
                case(S_ae_state_latched)
                    1: S_telemetry_text="AE: ADJUSTING     ";
                    2: S_telemetry_text="AE: STABLE        ";
                    3: S_telemetry_text="AE: DARK LIMIT    ";
                    4: S_telemetry_text="AE: BRIGHT LIMIT  ";
                    5: S_telemetry_text="AE: HOLD / OFF    ";
                    6: S_telemetry_text="AE: I2C ERROR     ";
                    default: S_telemetry_text="AE: WAIT          ";
                endcase
            end
            default: S_telemetry_text="                  ";
        endcase
    end
    always @(*) begin
        S_status_char=8'h20;
        if(S_status_col<18) S_status_char=S_status_line[143-(S_status_col*8) -: 8];
    end
    reg        S_osd_hit;
    reg [23:0] S_osd_color;
    reg [7:0]  S_logo_x;
    reg [7:0]  S_logo_y;
    reg [14:0] S_logo_addr;
    wire[24:0] S_logo_pixel;
    wire [15:0] S_shared_bits;

    // Mode 11 circle detector.  The detector assumes one main solid object
    // against a reasonably uniform background.  It first segments the
    // foreground, then checks bounding-box aspect ratio and fill ratio.
    wire [7:0] S_circle_r;
    wire [7:0] S_circle_g;
    wire [7:0] S_circle_b;
    wire [9:0] S_circle_luma_sum;
    wire [7:0] S_circle_luma;
    wire [7:0] S_circle_rgb_max;
    wire [7:0] S_circle_rgb_min;
    wire [7:0] S_circle_saturation;
    wire       S_circle_segment_hit;
    wire       S_circle_pixel;

    reg [11:0] S_circle_xmin;
    reg [11:0] S_circle_xmax;
    reg [11:0] S_circle_ymin;
    reg [11:0] S_circle_ymax;
    reg        S_circle_valid;
    // Number of consecutive frames for which a valid box may be retained
    // when the thresholded Blob momentarily fails its circularity gate.
    reg [3:0]  S_circle_lost_frames;
    wire [12:0] S_circle_center_x_sum;
    wire [12:0] S_circle_center_y_sum;
    wire [11:0] S_circle_center_x;
    wire [11:0] S_circle_center_y;
    wire        S_circle_box_hit;
    wire        S_circle_center_hit;

    // Publish after the detector's frame-boundary nonblocking assignments.
    // Existing recognition and smoothing are untouched. This is the tracked
    // bounding-box centre, not a new foreground-area centroid calculation.
    reg S_circle_publish_pending;
    assign O_circle_center_x = S_circle_center_x;
    assign O_circle_center_y = S_circle_center_y;
    assign O_circle_fresh_valid = (S_mode_latched == 2'b11) &&
                                  S_circle_valid && (S_circle_lost_frames == 0);
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            S_circle_publish_pending <= 1'b0;
            O_circle_sample <= 1'b0;
        end else begin
            S_circle_publish_pending <= I_video_user;
            O_circle_sample <= S_circle_publish_pending;
        end
    end

    // Target annotation uses exactly the bounding-box centre published to
    // the servo controllers. Coordinates are zero-based image pixels:
    // origin at top-left; X increases right, Y increases down.
    // Snapshot after SOF publication; keep text unchanged within the frame.
    // 0=no target, 1=fresh tracking observation, 2=retained (lost) target.
    reg [1:0] S_target_state_latched;
    reg [11:0] S_target_x_latched, S_target_y_latched;
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            S_target_state_latched<=0;S_target_x_latched<=0;S_target_y_latched<=0;
        end else if(S_status_frame_delay[3]) begin
            S_target_state_latched<=0;
            S_target_x_latched<=0;S_target_y_latched<=0;
            if(S_mode_latched==2'b11 && S_circle_valid &&
               S_circle_center_x<IMG_WIDTH && S_circle_center_y<IMG_HEIGHT) begin
                S_target_state_latched<=(S_circle_lost_frames==0) ? 2'd1 : 2'd2;
                S_target_x_latched<=S_circle_center_x;
                S_target_y_latched<=S_circle_center_y;
            end
        end
    end
    localparam integer TARGET_X=16;
    localparam integer TARGET_Y=IMG_HEIGHT-64;
    localparam integer TARGET_W=160;
    // Keep this lower-left panel separate from the upper-right telemetry.
    // Small detector-only diagnostic frames do not draw the full-size panel.
    wire S_target_panel=(TARGET_OSD_ENABLE!=0) && (DEBUG_MODE==0) &&
        (IMG_WIDTH>=640) && (IMG_HEIGHT>=320) && (S_mode_latched==2'b11) &&
        S_video_de_2d && S_x_2d>=TARGET_X && S_x_2d<TARGET_X+TARGET_W &&
        S_y_2d>=TARGET_Y && S_y_2d<TARGET_Y+48;
    wire S_target_text_area=S_target_panel &&
        S_x_2d>=TARGET_X+8 && S_x_2d<TARGET_X+152 &&
        S_y_2d>=TARGET_Y+8 && S_y_2d<TARGET_Y+40;
    wire [11:0] S_target_local_x=S_x_2d-(TARGET_X+8);
    wire [11:0] S_target_local_y=S_y_2d-(TARGET_Y+8);
    wire [4:0] S_target_col=S_target_local_x[7:3];
    reg [143:0] S_target_text;
    reg [7:0] S_target_char;
    wire [15:0] S_target_bits;
    wire [23:0] S_target_color=24'h00ff00;
    wire S_target_ink=S_target_text_area && S_target_bits[4'd15-{1'b0,S_target_local_x[2:0]}];
    always @(*) begin
        if(!S_target_local_y[4]) begin
            case(S_target_state_latched)
                1: S_target_text="TARGET: TRACKING  ";
                2: S_target_text="TARGET: LOST      ";
                default: S_target_text="TARGET: NOT FOUND ";
            endcase
        end else if(S_target_state_latched!=0) begin
            S_target_text={(S_target_state_latched==1) ? "LIVE " : "LAST ",
                "X:",F_status_digit(S_target_x_bcd[15:12]),F_status_digit(S_target_x_bcd[11:8]),
                F_status_digit(S_target_x_bcd[7:4]),F_status_digit(S_target_x_bcd[3:0]),
                " Y:",F_status_digit(S_target_y_bcd[15:12]),F_status_digit(S_target_y_bcd[11:8]),
                F_status_digit(S_target_y_bcd[7:4]),F_status_digit(S_target_y_bcd[3:0])};
        end else S_target_text="X:NA Y:NA         ";
    end
    always @(*) begin
        S_target_char=8'h20;
        if(S_target_col<18) S_target_char=S_target_text[143-S_target_col*8 -: 8];
    end
    // All six values use one 16-cycle shift/add-3 converter, once per frame.
    // Start one clock after both snapshots settle. All digits are ready in
    // 96 clocks, well before the first panel at image line 16 (>=640 px/line).
    // Numeric division is absent from the per-pixel text rendering path.
    osd_decimal_formatter u_osd_decimal_formatter (
        .I_clk(I_clk),.I_rst_n(I_rst_n),.I_start(S_decimal_start),
        .I_values({6'd0,S_capture_fps_latched,6'd0,S_display_fps_latched,
                   6'd0,S_new_fps_latched,S_latency_latched,
                   4'd0,S_target_x_latched,4'd0,S_target_y_latched}),
        .O_capture_bcd(S_capture_fps_bcd),.O_display_bcd(S_display_fps_bcd),
        .O_new_bcd(S_new_fps_bcd),.O_latency_bcd(S_latency_bcd),
        .O_x_bcd(S_target_x_bcd),.O_y_bcd(S_target_y_bcd),.O_done());

    // Full-frame multi-object tracker.  A solid object contributes one
    // horizontal run per row; overlapping runs on adjacent rows are joined
    // into one blob.  Eight slots retain several independent candidates while
    // keeping the run-matching path short enough for the 75 MHz pixel clock.
    localparam integer CIRCLE_BLOB_COUNT = 8;
    localparam integer CIRCLE_BLOB_INDEX_WIDTH = 3;
    reg        S_blob_used [0:CIRCLE_BLOB_COUNT-1];
    reg [11:0] S_blob_xmin [0:CIRCLE_BLOB_COUNT-1];
    reg [11:0] S_blob_xmax [0:CIRCLE_BLOB_COUNT-1];
    reg [11:0] S_blob_ymin [0:CIRCLE_BLOB_COUNT-1];
    reg [11:0] S_blob_ymax [0:CIRCLE_BLOB_COUNT-1];
    reg [19:0] S_blob_area [0:CIRCLE_BLOB_COUNT-1];
    reg [11:0] S_blob_last_run_xmin [0:CIRCLE_BLOB_COUNT-1];
    reg [11:0] S_blob_last_run_xmax [0:CIRCLE_BLOB_COUNT-1];
    reg [11:0] S_blob_last_y [0:CIRCLE_BLOB_COUNT-1];

    reg        S_circle_run_active;
    reg        S_circle_gap_pending;
    reg [11:0] S_circle_run_start;
    wire       S_circle_run_end_event;
    wire [11:0] S_circle_run_end_x;
    wire [12:0] S_circle_run_length;
    reg        S_blob_match_found;
    reg [CIRCLE_BLOB_INDEX_WIDTH-1:0] S_blob_match_index;
    reg        S_blob_free_found;
    reg [CIRCLE_BLOB_INDEX_WIDTH-1:0] S_blob_free_index;
    reg        S_blob_recycle_found;
    reg [CIRCLE_BLOB_INDEX_WIDTH-1:0] S_blob_recycle_index;

    // Candidate checks are performed sequentially during vertical blanking.
    // This shares one width*height multiplier across all blob slots instead
    // of instantiating one multiplier and several wide comparators per slot.
    reg        S_blob_scan_busy;
    reg        S_blob_scan_done;
    reg [CIRCLE_BLOB_INDEX_WIDTH-1:0] S_blob_scan_index;
    reg        S_blob_best_valid;
    reg [3:0]  S_blob_best_rank;
    reg [19:0] S_blob_best_area;
    reg [11:0] S_blob_best_xmin;
    reg [11:0] S_blob_best_xmax;
    reg [11:0] S_blob_best_ymin;
    reg [11:0] S_blob_best_ymax;
    wire [12:0] S_blob_scan_width;
    wire [12:0] S_blob_scan_height;
    wire [25:0] S_blob_scan_bbox_area;
    wire [31:0] S_blob_scan_area_ext;
    wire [31:0] S_blob_scan_bbox_ext;
    wire [31:0] S_blob_scan_area_x100;
    wire [31:0] S_blob_scan_bbox_x60;
    wire [31:0] S_blob_scan_bbox_x90;
    wire [15:0] S_blob_scan_width_x5;
    wire [15:0] S_blob_scan_height_x5;
    wire [15:0] S_blob_scan_width_x4;
    wire [15:0] S_blob_scan_height_x4;
    wire        S_blob_scan_circle_ok;
    wire        S_blob_scan_round_ok;
    wire        S_blob_scan_aspect_tight;
    wire        S_blob_scan_fill_tight;
    wire [31:0] S_blob_scan_bbox_x68;
    wire [31:0] S_blob_scan_bbox_x86;
    wire [16:0] S_blob_scan_width_x10;
    wire [16:0] S_blob_scan_height_x10;
    wire [16:0] S_blob_scan_width_x9;
    wire [16:0] S_blob_scan_height_x9;
    wire [12:0] S_blob_scan_center_x;
    wire [12:0] S_blob_scan_center_y;
    wire [12:0] S_blob_scan_dx;
    wire [12:0] S_blob_scan_dy;
    wire [1:0]  S_blob_scan_track_bonus;
    wire [3:0]  S_blob_scan_rank;
    wire [12:0] S_blob_best_center_x_sum;
    wire [12:0] S_blob_best_center_y_sum;
    wire [11:0] S_blob_best_center_x;
    wire [11:0] S_blob_best_center_y;
    wire [11:0] S_blob_best_dx;
    wire [11:0] S_blob_best_dy;
    integer S_blob_match_i;
    integer S_blob_reset_i;

    localparam LOGO_X = 12'd0;
    localparam LOGO_Y = 12'd0;
    localparam LOGO_W = 12'd160;
    localparam LOGO_H = 12'd160;
    // video_out presents a requested pixel one clock after I_video_de.  The
    // sync/DE and coordinate pipelines below add the matching two edge-to-edge
    // clocks at this module boundary, so I_video_rd_data is already aligned
    // with S_video_de_2d/S_x_2d/S_y_2d.  A further local data pipeline blanks
    // the first two pixels and drops the last two pixels of every line.
    assign S_circle_r = I_video_rd_data[23:16];
    assign S_circle_g = I_video_rd_data[15:8];
    assign S_circle_b = I_video_rd_data[7:0];

    // Approximate luminance = (R + 2G + B) / 4.  This uses only adders.
    assign S_circle_luma_sum = {2'b00, S_circle_r} +
                               {1'b0, S_circle_g, 1'b0} +
                               {2'b00, S_circle_b};
    assign S_circle_luma = S_circle_luma_sum[9:2];

    assign S_circle_rgb_max = (S_circle_r >= S_circle_g) ?
                              ((S_circle_r >= S_circle_b) ? S_circle_r : S_circle_b) :
                              ((S_circle_g >= S_circle_b) ? S_circle_g : S_circle_b);
    assign S_circle_rgb_min = (S_circle_r <= S_circle_g) ?
                              ((S_circle_r <= S_circle_b) ? S_circle_r : S_circle_b) :
                              ((S_circle_g <= S_circle_b) ? S_circle_g : S_circle_b);
    assign S_circle_saturation = S_circle_rgb_max - S_circle_rgb_min;

    // For dark-object mode require all three channels to be dark as well as
    // the luminance test.  This rejects coloured/grey background areas that
    // can have a low approximate luminance after camera processing.
    assign S_circle_segment_hit =
        (CIRCLE_SEGMENT_MODE == 0) ?
            ((S_circle_luma < CIRCLE_LUMA_THRESHOLD) &&
             (S_circle_r < 8'd90) &&
             (S_circle_g < 8'd90) &&
             (S_circle_b < 8'd90)) :
        (CIRCLE_SEGMENT_MODE == 1) ? (S_circle_luma > CIRCLE_LUMA_THRESHOLD) :
                                     ((S_circle_saturation > CIRCLE_SAT_THRESHOLD) &&
                                      (S_circle_luma > 8'd35));

    // Full-frame detection.  No central ROI is imposed; every active pixel
    // except the two-pixel timing margin may participate in the candidate.
    assign S_circle_pixel = (S_mode_latched == 2'b11) &&
                            S_video_de_2d &&
                            (S_x_2d >= 12'd2) && (S_x_2d < IMG_WIDTH - 2) &&
                            (S_y_2d >= 12'd2) && (S_y_2d < IMG_HEIGHT - 2) &&
                            S_circle_segment_hit;

    // Bridge one isolated background pixel inside a foreground run.  This
    // small horizontal closing filter suppresses threshold speckle without a
    // line buffer.  The two-pixel margin still closes the run on every line.
    assign S_circle_run_end_event = S_circle_run_active &&
                                    S_video_de_2d && !S_circle_pixel &&
                                    S_circle_gap_pending;
    assign S_circle_run_end_x = S_x_2d - 12'd2;
    assign S_circle_run_length = {1'b0, S_circle_run_end_x} -
                                 {1'b0, S_circle_run_start} + 13'd1;

    // Join this run to a blob whose run on the preceding row overlaps it.
    // A one-pixel tolerance keeps diagonally adjacent boundary pixels joined.
    always @(*) begin
        S_blob_match_found = 1'b0;
        S_blob_match_index = {CIRCLE_BLOB_INDEX_WIDTH{1'b0}};
        S_blob_free_found  = 1'b0;
        S_blob_free_index  = {CIRCLE_BLOB_INDEX_WIDTH{1'b0}};
        S_blob_recycle_found = 1'b0;
        S_blob_recycle_index = {CIRCLE_BLOB_INDEX_WIDTH{1'b0}};

        for(S_blob_match_i = 0; S_blob_match_i < CIRCLE_BLOB_COUNT; S_blob_match_i = S_blob_match_i + 1) begin
            if(!S_blob_free_found && !S_blob_used[S_blob_match_i]) begin
                S_blob_free_found = 1'b1;
                S_blob_free_index = S_blob_match_i;
            end

            // A sub-threshold component which has missed two rows can no
            // longer become a useful candidate.  Reusing that slot prevents
            // small marks at the top of the image from exhausting all slots.
            if(!S_blob_recycle_found && S_blob_used[S_blob_match_i] &&
               (S_blob_area[S_blob_match_i] < CIRCLE_MIN_AREA) &&
               ((S_blob_last_y[S_blob_match_i] + 12'd2) < S_y_2d)) begin
                S_blob_recycle_found = 1'b1;
                S_blob_recycle_index = S_blob_match_i;
            end

            if(!S_blob_match_found && S_blob_used[S_blob_match_i] &&
               (S_y_2d > S_blob_last_y[S_blob_match_i]) &&
               (S_y_2d <= (S_blob_last_y[S_blob_match_i] + 12'd2)) &&
               ((S_circle_run_start <= (S_blob_last_run_xmax[S_blob_match_i] + 12'd2))) &&
               ((S_circle_run_end_x + 12'd2) >= S_blob_last_run_xmin[S_blob_match_i])) begin
                S_blob_match_found = 1'b1;
                S_blob_match_index = S_blob_match_i;
            end
        end
    end

    assign S_blob_scan_width =
        {1'b0, S_blob_xmax[S_blob_scan_index]} -
        {1'b0, S_blob_xmin[S_blob_scan_index]} + 13'd1;
    assign S_blob_scan_height =
        {1'b0, S_blob_ymax[S_blob_scan_index]} -
        {1'b0, S_blob_ymin[S_blob_scan_index]} + 13'd1;
    assign S_blob_scan_bbox_area = S_blob_scan_width * S_blob_scan_height;
    assign S_blob_scan_area_ext = {12'd0, S_blob_area[S_blob_scan_index]};
    assign S_blob_scan_bbox_ext = {6'd0, S_blob_scan_bbox_area};

    // Constant ratios are expressed as shifts/adds so the only DSP required
    // by the detector is the shared bounding-box area multiplier above.
    assign S_blob_scan_area_x100 =
        (S_blob_scan_area_ext << 6) +
        (S_blob_scan_area_ext << 5) +
        (S_blob_scan_area_ext << 2);
    assign S_blob_scan_bbox_x60 =
        (S_blob_scan_bbox_ext << 6) - (S_blob_scan_bbox_ext << 2);
    assign S_blob_scan_bbox_x90 =
        (S_blob_scan_bbox_ext << 6) +
        (S_blob_scan_bbox_ext << 4) +
        (S_blob_scan_bbox_ext << 3) +
        (S_blob_scan_bbox_ext << 1);
    assign S_blob_scan_bbox_x68 =
        (S_blob_scan_bbox_ext << 6) +
        (S_blob_scan_bbox_ext << 2);
    assign S_blob_scan_bbox_x86 =
        (S_blob_scan_bbox_ext << 6) +
        (S_blob_scan_bbox_ext << 4) +
        (S_blob_scan_bbox_ext << 2) +
        (S_blob_scan_bbox_ext << 1);
    assign S_blob_scan_width_x5 =
        ({3'd0, S_blob_scan_width} << 2) + {3'd0, S_blob_scan_width};
    assign S_blob_scan_height_x5 =
        ({3'd0, S_blob_scan_height} << 2) + {3'd0, S_blob_scan_height};
    assign S_blob_scan_width_x4 = {3'd0, S_blob_scan_width} << 2;
    assign S_blob_scan_height_x4 = {3'd0, S_blob_scan_height} << 2;
    assign S_blob_scan_width_x10 =
        ({4'd0, S_blob_scan_width} << 3) +
        ({4'd0, S_blob_scan_width} << 1);
    assign S_blob_scan_height_x10 =
        ({4'd0, S_blob_scan_height} << 3) +
        ({4'd0, S_blob_scan_height} << 1);
    assign S_blob_scan_width_x9 =
        ({4'd0, S_blob_scan_width} << 3) +
         {4'd0, S_blob_scan_width};
    assign S_blob_scan_height_x9 =
        ({4'd0, S_blob_scan_height} << 3) +
         {4'd0, S_blob_scan_height};

    // The hard gate below remains tolerant of lens perspective and ragged
    // threshold edges.  These tighter tests are used only to rank candidates,
    // so a clean circle wins over a similarly sized rounded rectangle.
    assign S_blob_scan_aspect_tight =
        (S_blob_scan_width_x10 >= S_blob_scan_height_x9) &&
        (S_blob_scan_height_x10 >= S_blob_scan_width_x9);
    assign S_blob_scan_fill_tight =
        (S_blob_scan_area_x100 >= S_blob_scan_bbox_x68) &&
        (S_blob_scan_area_x100 <= S_blob_scan_bbox_x86);
    assign S_blob_scan_center_x = {1'b0, S_blob_xmin[S_blob_scan_index]} +
                                  {1'b0, S_blob_xmax[S_blob_scan_index]};
    assign S_blob_scan_center_y = {1'b0, S_blob_ymin[S_blob_scan_index]} +
                                  {1'b0, S_blob_ymax[S_blob_scan_index]};
    assign S_blob_scan_dx = (S_blob_scan_center_x[12:1] >= S_circle_center_x) ?
                            (S_blob_scan_center_x[12:1] - S_circle_center_x) :
                            (S_circle_center_x - S_blob_scan_center_x[12:1]);
    assign S_blob_scan_dy = (S_blob_scan_center_y[12:1] >= S_circle_center_y) ?
                            (S_blob_scan_center_y[12:1] - S_circle_center_y) :
                            (S_circle_center_y - S_blob_scan_center_y[12:1]);
    assign S_blob_scan_track_bonus =
        (S_circle_valid && (S_blob_scan_dx <= 13'd96) &&
         (S_blob_scan_dy <= 13'd96)) ? 2'd2 : 2'd0;
    assign S_blob_scan_rank =
        (S_blob_scan_aspect_tight ? 4'd2 : 4'd0) +
        (S_blob_scan_fill_tight   ? 4'd3 : 4'd0) +
        S_blob_scan_track_bonus;

    assign S_blob_scan_circle_ok =
        S_blob_used[S_blob_scan_index] &&
        (S_blob_area[S_blob_scan_index] >= CIRCLE_MIN_AREA) &&
        (S_blob_scan_width >= CIRCLE_MIN_DIAMETER) &&
        (S_blob_scan_width <= CIRCLE_MAX_DIAMETER) &&
        (S_blob_scan_height >= CIRCLE_MIN_DIAMETER) &&
        (S_blob_scan_height <= CIRCLE_MAX_DIAMETER) &&
        (S_blob_scan_width_x5 >= S_blob_scan_height_x4) &&
        (S_blob_scan_height_x5 >= S_blob_scan_width_x4) &&
        (S_blob_scan_area_x100 >= S_blob_scan_bbox_x60) &&
        (S_blob_scan_area_x100 <= S_blob_scan_bbox_x90);

    // Hard circularity gate: accept mildly distorted circles (aspect
    // 0.8..1.25, fill 60..90%).  The rank above then prefers candidates with
    // aspect 0.9..1.11 and fill 68..86%, centred on a disk's 78.5% fill.
    assign S_blob_scan_round_ok = S_blob_scan_circle_ok;

    assign S_circle_center_x_sum = {1'b0, S_circle_xmin} + {1'b0, S_circle_xmax};
    assign S_circle_center_y_sum = {1'b0, S_circle_ymin} + {1'b0, S_circle_ymax};
    assign S_circle_center_x = S_circle_center_x_sum[12:1];
    assign S_circle_center_y = S_circle_center_y_sum[12:1];
    assign S_blob_best_center_x_sum = {1'b0, S_blob_best_xmin} +
                                      {1'b0, S_blob_best_xmax};
    assign S_blob_best_center_y_sum = {1'b0, S_blob_best_ymin} +
                                      {1'b0, S_blob_best_ymax};
    assign S_blob_best_center_x = S_blob_best_center_x_sum[12:1];
    assign S_blob_best_center_y = S_blob_best_center_y_sum[12:1];
    assign S_blob_best_dx = (S_blob_best_center_x >= S_circle_center_x) ?
                            (S_blob_best_center_x - S_circle_center_x) :
                            (S_circle_center_x - S_blob_best_center_x);
    assign S_blob_best_dy = (S_blob_best_center_y >= S_circle_center_y) ?
                            (S_blob_best_center_y - S_circle_center_y) :
                            (S_circle_center_y - S_blob_best_center_y);

    // Three-pixel red rectangle.
    assign S_circle_box_hit = S_circle_valid && S_video_de_2d &&
        (S_x_2d >= S_circle_xmin) && (S_x_2d <= S_circle_xmax) &&
        (S_y_2d >= S_circle_ymin) && (S_y_2d <= S_circle_ymax) &&
        ((S_x_2d <= S_circle_xmin + 12'd2) ||
         (S_x_2d + 12'd2 >= S_circle_xmax) ||
         (S_y_2d <= S_circle_ymin + 12'd2) ||
         (S_y_2d + 12'd2 >= S_circle_ymax));

    // A 17x3 plus-shaped centroid marker.
    assign S_circle_center_hit = S_circle_valid && S_video_de_2d &&
        ((((S_y_2d + 12'd1) >= S_circle_center_y) &&
          (S_y_2d <= (S_circle_center_y + 12'd1)) &&
          ((S_x_2d + 12'd8) >= S_circle_center_x) &&
          (S_x_2d <= (S_circle_center_x + 12'd8))) ||
         (((S_x_2d + 12'd1) >= S_circle_center_x) &&
          (S_x_2d <= (S_circle_center_x + 12'd1)) &&
          ((S_y_2d + 12'd8) >= S_circle_center_y) &&
          (S_y_2d <= (S_circle_center_y + 12'd8))));

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            S_mode_meta    <= 2'b00;
            S_mode_sync    <= 2'b00;
            S_mode_latched <= 2'b00;
            S_process_algo_latched <= 3'd1;
        end
        else begin
            S_mode_meta <= I_mode;
            S_mode_sync <= S_mode_meta;

            if(I_video_user) begin
                // Freeze the text for the whole output frame. This is the
                // ISP's applied state, including physical-switch submodes,
                // rather than the requested UART algorithm or raw switches.
                if(I_process_algo >= 3'd1 && I_process_algo <= 3'd3)
                    S_process_algo_latched <= I_process_algo;
                if (uart_frame_owner) begin
                    case(uart_frame_algo)
                        0: S_mode_latched<=2'b00;
                        4: S_mode_latched<=2'b10;
                        5: S_mode_latched<=2'b11;
                        default: S_mode_latched<=2'b01;
                    endcase
                end else S_mode_latched <= S_mode_sync;
            end
        end
    end

    assign O_video_rd_en = I_video_de;

    anlogic_logo_rom u_anlogic_logo_rom(
        .I_addr  ( S_logo_addr  ),
        .O_pixel ( S_logo_pixel )
    );

    // These panels never need different glyphs at the same output pixel.
    // Match output-compositor priority and share one font lookup across mode
    // status and target annotation.
    wire [7:0] S_shared_char = S_status_panel ? S_status_char :
                              S_target_panel ? S_target_char : 8'h20;
    wire [3:0] S_shared_row = S_status_panel ? S_status_local_y[3:0] :
                             S_target_panel ? S_target_local_y[3:0] : 4'd0;
    assign S_status_bits = S_shared_bits;
    assign S_target_bits = S_shared_bits;
    osd_char_lib u_osd_char_lib(
        .I_char     ( S_shared_char ),
        .I_row      ( S_shared_row  ),
        .O_row_bits ( S_shared_bits )
    );

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            S_x <= 12'd0;
            S_y <= 12'd0;
        end
        else if(I_video_user) begin
            S_x <= 12'd0;
            S_y <= 12'd0;
        end
        else if(I_video_de) begin
            if(I_video_last) begin
                S_x <= 12'd0;
                S_y <= S_y + 12'd1;
            end
            else begin
                S_x <= S_x + 12'd1;
            end
        end
    end

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            S_x_1d <= 12'd0;
            S_x_2d <= 12'd0;
            S_y_1d <= 12'd0;
            S_y_2d <= 12'd0;
        end
        else begin
            S_x_1d <= S_x;
            S_x_2d <= S_x_1d;
            S_y_1d <= S_y;
            S_y_2d <= S_y_1d;
        end
    end

    // Track independent horizontal-run blobs over the complete active frame.
    // Publish the largest blob that passes the circle tests at the next frame
    // start, then clear all blob slots for the new frame.
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            S_circle_run_active <= 1'b0;
            S_circle_gap_pending <= 1'b0;
            S_circle_run_start  <= 12'd0;
            S_circle_xmin     <= 12'd0;
            S_circle_xmax     <= 12'd0;
            S_circle_ymin     <= 12'd0;
            S_circle_ymax     <= 12'd0;
            S_circle_valid    <= 1'b0;
            S_circle_lost_frames <= 4'd0;
            S_blob_scan_busy  <= 1'b0;
            S_blob_scan_done  <= 1'b0;
            S_blob_scan_index <= {CIRCLE_BLOB_INDEX_WIDTH{1'b0}};
            S_blob_best_valid <= 1'b0;
            S_blob_best_rank  <= 4'd0;
            S_blob_best_area  <= 20'd0;
            S_blob_best_xmin  <= 12'd0;
            S_blob_best_xmax  <= 12'd0;
            S_blob_best_ymin  <= 12'd0;
            S_blob_best_ymax  <= 12'd0;

            for(S_blob_reset_i = 0; S_blob_reset_i < CIRCLE_BLOB_COUNT; S_blob_reset_i = S_blob_reset_i + 1) begin
                S_blob_used[S_blob_reset_i]          <= 1'b0;
                S_blob_xmin[S_blob_reset_i]          <= 12'd0;
                S_blob_xmax[S_blob_reset_i]          <= 12'd0;
                S_blob_ymin[S_blob_reset_i]          <= 12'd0;
                S_blob_ymax[S_blob_reset_i]          <= 12'd0;
                S_blob_area[S_blob_reset_i]          <= 20'd0;
                S_blob_last_run_xmin[S_blob_reset_i] <= 12'd0;
                S_blob_last_run_xmax[S_blob_reset_i] <= 12'd0;
                S_blob_last_y[S_blob_reset_i]        <= 12'd0;
            end
        end
        else if(I_video_user) begin
            S_circle_run_active <= 1'b0;
            S_circle_gap_pending <= 1'b0;

            if(S_mode_latched == 2'b11) begin
                if(S_blob_scan_done && S_blob_best_valid) begin
                    // First detection is applied immediately.  Afterwards
                    // move only one quarter of the distance per frame, which
                    // suppresses edge-threshold jitter without freezing motion.
                    // Smooth a continuing target, but snap to a new target
                    // after a large jump so reacquisition is not sluggish.
                    if(S_circle_valid && (S_blob_best_dx <= 12'd96) &&
                       (S_blob_best_dy <= 12'd96)) begin
                        if(S_blob_best_xmin >= S_circle_xmin)
                            S_circle_xmin <= S_circle_xmin + ((S_blob_best_xmin - S_circle_xmin + 12'd3) >> 2);
                        else
                            S_circle_xmin <= S_circle_xmin - ((S_circle_xmin - S_blob_best_xmin + 12'd3) >> 2);
                        if(S_blob_best_xmax >= S_circle_xmax)
                            S_circle_xmax <= S_circle_xmax + ((S_blob_best_xmax - S_circle_xmax + 12'd3) >> 2);
                        else
                            S_circle_xmax <= S_circle_xmax - ((S_circle_xmax - S_blob_best_xmax + 12'd3) >> 2);
                        if(S_blob_best_ymin >= S_circle_ymin)
                            S_circle_ymin <= S_circle_ymin + ((S_blob_best_ymin - S_circle_ymin + 12'd3) >> 2);
                        else
                            S_circle_ymin <= S_circle_ymin - ((S_circle_ymin - S_blob_best_ymin + 12'd3) >> 2);
                        if(S_blob_best_ymax >= S_circle_ymax)
                            S_circle_ymax <= S_circle_ymax + ((S_blob_best_ymax - S_circle_ymax + 12'd3) >> 2);
                        else
                            S_circle_ymax <= S_circle_ymax - ((S_circle_ymax - S_blob_best_ymax + 12'd3) >> 2);
                    end
                    else begin
                        S_circle_xmin  <= S_blob_best_xmin;
                        S_circle_xmax  <= S_blob_best_xmax;
                        S_circle_ymin  <= S_blob_best_ymin;
                        S_circle_ymax  <= S_blob_best_ymax;
                    end
                    S_circle_valid      <= 1'b1;
                    S_circle_lost_frames <= 4'd0;
                end
                else if(S_circle_valid && (S_circle_lost_frames < 4'd8)) begin
                    // Keep the last box for at most eight missing frames.
                    // This bridges short threshold/circularity dropouts and
                    // prevents the displayed target from blinking.
                    S_circle_lost_frames <= S_circle_lost_frames + 1'b1;
                end
                else begin
                    S_circle_valid       <= 1'b0;
                    S_circle_lost_frames <= 4'd0;
                end
            end
            else begin
                S_circle_valid       <= 1'b0;
                S_circle_lost_frames <= 4'd0;
            end

            S_blob_scan_busy  <= 1'b0;
            S_blob_scan_done  <= 1'b0;
            S_blob_scan_index <= {CIRCLE_BLOB_INDEX_WIDTH{1'b0}};
            S_blob_best_valid <= 1'b0;
            S_blob_best_rank  <= 4'd0;
            S_blob_best_area  <= 20'd0;
            S_blob_best_xmin  <= 12'd0;
            S_blob_best_xmax  <= 12'd0;
            S_blob_best_ymin  <= 12'd0;
            S_blob_best_ymax  <= 12'd0;

            for(S_blob_reset_i = 0; S_blob_reset_i < CIRCLE_BLOB_COUNT; S_blob_reset_i = S_blob_reset_i + 1) begin
                S_blob_used[S_blob_reset_i]          <= 1'b0;
                S_blob_xmin[S_blob_reset_i]          <= 12'd0;
                S_blob_xmax[S_blob_reset_i]          <= 12'd0;
                S_blob_ymin[S_blob_reset_i]          <= 12'd0;
                S_blob_ymax[S_blob_reset_i]          <= 12'd0;
                S_blob_area[S_blob_reset_i]          <= 20'd0;
                S_blob_last_run_xmin[S_blob_reset_i] <= 12'd0;
                S_blob_last_run_xmax[S_blob_reset_i] <= 12'd0;
                S_blob_last_y[S_blob_reset_i]        <= 12'd0;
            end
        end
        else begin
            // Start an eight-cycle candidate scan after the last active pixel.
            // The following vertical blanking interval provides ample time.
            if(S_video_de_2d &&
               (S_x_2d == IMG_WIDTH - 1) &&
               (S_y_2d == IMG_HEIGHT - 1)) begin
                S_blob_scan_busy  <= 1'b1;
                S_blob_scan_done  <= 1'b0;
                S_blob_scan_index <= {CIRCLE_BLOB_INDEX_WIDTH{1'b0}};
                S_blob_best_valid <= 1'b0;
                S_blob_best_rank  <= 4'd0;
                S_blob_best_area  <= 20'd0;
            end
            else if(S_blob_scan_busy) begin
                // A new target must pass both tight shape tests (rank 5).
                // A tracked target may temporarily pass one tight test plus
                // the continuity bonus (rank >= 4), avoiding visible flicker.
                if(S_blob_scan_round_ok && (S_blob_scan_rank >= 4'd4) &&
                   (!S_blob_best_valid ||
                    (S_blob_scan_rank > S_blob_best_rank) ||
                    ((S_blob_scan_rank == S_blob_best_rank) &&
                     (S_blob_area[S_blob_scan_index] > S_blob_best_area)))) begin
                    S_blob_best_valid <= 1'b1;
                    S_blob_best_rank  <= S_blob_scan_rank;
                    S_blob_best_area  <= S_blob_area[S_blob_scan_index];
                    S_blob_best_xmin  <= S_blob_xmin[S_blob_scan_index];
                    S_blob_best_xmax  <= S_blob_xmax[S_blob_scan_index];
                    S_blob_best_ymin  <= S_blob_ymin[S_blob_scan_index];
                    S_blob_best_ymax  <= S_blob_ymax[S_blob_scan_index];
                end

                if(S_blob_scan_index == CIRCLE_BLOB_COUNT - 1) begin
                    S_blob_scan_busy <= 1'b0;
                    S_blob_scan_done <= 1'b1;
                end
                else begin
                    S_blob_scan_index <= S_blob_scan_index + 1'b1;
                end
            end

            if(S_circle_pixel && !S_circle_run_active) begin
                S_circle_run_active <= 1'b1;
                S_circle_gap_pending <= 1'b0;
                S_circle_run_start  <= S_x_2d;
            end
            else if(S_circle_pixel) begin
                S_circle_gap_pending <= 1'b0;
            end
            else if(S_circle_run_end_event) begin
                S_circle_run_active <= 1'b0;
                S_circle_gap_pending <= 1'b0;

                if(S_blob_match_found) begin
                    if(S_circle_run_start < S_blob_xmin[S_blob_match_index])
                        S_blob_xmin[S_blob_match_index] <= S_circle_run_start;
                    if(S_circle_run_end_x > S_blob_xmax[S_blob_match_index])
                        S_blob_xmax[S_blob_match_index] <= S_circle_run_end_x;
                    S_blob_ymax[S_blob_match_index] <= S_y_2d;
                    S_blob_area[S_blob_match_index] <=
                        S_blob_area[S_blob_match_index] + S_circle_run_length;
                    S_blob_last_run_xmin[S_blob_match_index] <= S_circle_run_start;
                    S_blob_last_run_xmax[S_blob_match_index] <= S_circle_run_end_x;
                    S_blob_last_y[S_blob_match_index]        <= S_y_2d;
                end
                else if((S_blob_free_found || S_blob_recycle_found) &&
                        (S_circle_run_length >= CIRCLE_MIN_RUN)) begin
                    if(S_blob_free_found) begin
                        S_blob_used[S_blob_free_index]          <= 1'b1;
                        S_blob_xmin[S_blob_free_index]          <= S_circle_run_start;
                        S_blob_xmax[S_blob_free_index]          <= S_circle_run_end_x;
                        S_blob_ymin[S_blob_free_index]          <= S_y_2d;
                        S_blob_ymax[S_blob_free_index]          <= S_y_2d;
                        S_blob_area[S_blob_free_index]          <= S_circle_run_length;
                        S_blob_last_run_xmin[S_blob_free_index] <= S_circle_run_start;
                        S_blob_last_run_xmax[S_blob_free_index] <= S_circle_run_end_x;
                        S_blob_last_y[S_blob_free_index]        <= S_y_2d;
                    end
                    else begin
                        S_blob_used[S_blob_recycle_index]          <= 1'b1;
                        S_blob_xmin[S_blob_recycle_index]          <= S_circle_run_start;
                        S_blob_xmax[S_blob_recycle_index]          <= S_circle_run_end_x;
                        S_blob_ymin[S_blob_recycle_index]          <= S_y_2d;
                        S_blob_ymax[S_blob_recycle_index]          <= S_y_2d;
                        S_blob_area[S_blob_recycle_index]          <= S_circle_run_length;
                        S_blob_last_run_xmin[S_blob_recycle_index] <= S_circle_run_start;
                        S_blob_last_run_xmax[S_blob_recycle_index] <= S_circle_run_end_x;
                        S_blob_last_y[S_blob_recycle_index]        <= S_y_2d;
                    end
                end
            end
            else if(S_circle_run_active && S_video_de_2d &&
                    !S_circle_pixel) begin
                S_circle_gap_pending <= 1'b1;
            end
        end
    end

    always @(*) begin
        S_osd_hit   = 1'b0;
        S_osd_color = 24'd0;
        S_logo_addr = 15'd0;

        S_logo_x = 8'd0;
        S_logo_y = 8'd0;

        if(S_video_de_2d) begin
            if((S_x_2d < LOGO_W) && (S_y_2d < LOGO_H)) begin
                S_logo_x = S_x_2d - LOGO_X;
                S_logo_y = S_y_2d - LOGO_Y;
                S_logo_addr = ({7'd0, S_logo_y} << 7) +
                              ({7'd0, S_logo_y} << 5) +
                              {7'd0, S_logo_x};
                if(S_logo_pixel[24]) begin
                    S_osd_hit   = 1'b1;
                    S_osd_color = S_logo_pixel[23:0];
                end
            end


        end
    end

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            S_video_vsync_1d <= 1'b0;
            S_video_vsync_2d <= 1'b0;
            S_video_hsync_1d <= 1'b0;
            S_video_hsync_2d <= 1'b0;
            S_video_de_1d    <= 1'b0;
            S_video_de_2d    <= 1'b0;
            O_hdmi_vsync     <= 1'b0;
            O_hdmi_hsync     <= 1'b0;
            O_hdmi_de        <= 1'b0;
            O_hdmi_data      <= 24'd0;
        end
        else begin
            S_video_vsync_1d <= I_video_vsync;
            S_video_vsync_2d <= S_video_vsync_1d;
            O_hdmi_vsync     <= S_video_vsync_2d;

            S_video_hsync_1d <= I_video_hsync;
            S_video_hsync_2d <= S_video_hsync_1d;
            O_hdmi_hsync     <= S_video_hsync_2d;

            S_video_de_1d    <= I_video_de;
            S_video_de_2d    <= S_video_de_1d;
            O_hdmi_de        <= S_video_de_2d;

            // Mode 11: restore the normal camera image and overlay the
            // accepted circular target.  The three-pixel outline is red and
            // the centroid cross is green; the image remains visible inside
            // the box so motion and false detections can be judged directly.
            if(S_status_panel) begin
                O_hdmi_data <= S_status_ink ? STATUS_FG : S_panel_background;
            end
            else if(S_target_panel) begin
                O_hdmi_data <= S_target_ink ? S_target_color : S_panel_background;
            end
            else if(S_mode_latched == 2'b11) begin
                if(!S_video_de_2d)
                    O_hdmi_data <= 24'd0;
                else if(S_circle_center_hit)
                    O_hdmi_data <= 24'h00ff00;
                else if(S_circle_box_hit)
                    O_hdmi_data <= 24'hff0000;
                else
                    O_hdmi_data <= I_video_rd_data;
            end
            else if(DEBUG_MODE == 1) begin
                if(S_video_de_2d) begin
                    if(S_x_2d < 12'd128)
                        O_hdmi_data <= 24'hff0000;
                    else if(S_x_2d < 12'd256)
                        O_hdmi_data <= 24'h00ff00;
                    else if(S_x_2d < 12'd384)
                        O_hdmi_data <= 24'h0000ff;
                    else if(S_x_2d < 12'd512)
                        O_hdmi_data <= 24'hffff00;
                    else if(S_x_2d < 12'd640)
                        O_hdmi_data <= 24'h00ffff;
                    else if(S_x_2d < 12'd768)
                        O_hdmi_data <= 24'hff00ff;
                    else if(S_x_2d < 12'd896)
                        O_hdmi_data <= 24'hffffff;
                    else
                        O_hdmi_data <= 24'h202020;
                end
                else begin
                    O_hdmi_data <= 24'd0;
                end
            end
            else if(DEBUG_MODE == 2) begin
                if(S_video_de_2d) begin
                    case (I_debug_status)
                        4'b0000: O_hdmi_data <= 24'h000000;
                        4'b0001: O_hdmi_data <= 24'hff0000;
                        4'b0011: O_hdmi_data <= 24'hffff00;
                        4'b0111: O_hdmi_data <= 24'h00ff00;
                        4'b1111: O_hdmi_data <= I_video_rd_data;
                        default: O_hdmi_data <= 24'h0000ff;
                    endcase
                end
                else begin
                    O_hdmi_data <= 24'd0;
                end
            end
            else if(DEBUG_MODE == 3) begin
                if(S_video_de_2d) begin
                    if(S_x_2d < 12'd256)
                        O_hdmi_data <= I_debug_status[0] ? 24'hff8000 : 24'h201000;
                    else if(S_x_2d < 12'd512)
                        O_hdmi_data <= I_debug_status[1] ? 24'hffff00 : 24'h202000;
                    else if(S_x_2d < 12'd768)
                        O_hdmi_data <= I_debug_status[2] ? 24'hff0000 : 24'h200000;
                    else
                        O_hdmi_data <= I_debug_status[3] ? 24'h00ffff : 24'h002020;
                end
                else begin
                    O_hdmi_data <= 24'd0;
                end
            end
            else if(S_mode_latched == 2'b10) begin
                if(!S_video_de_2d)
                    O_hdmi_data <= 24'd0;
                else if(S_osd_hit)
                    O_hdmi_data <= S_osd_color;
                else
                    O_hdmi_data <= I_video_rd_data;
            end
            else begin
                // Modes 00 and 01 display the frame read from DDR.  The ISP
                // selects normal camera or edge data before the DDR writer.
                O_hdmi_data <= S_video_de_2d ? I_video_rd_data : 24'd0;
            end
        end
    end

endmodule

// Low-resource decimal formatting for slow, frame-stable OSD metadata.
// I_values holds six 16-bit fields (capture FPS first, Y last) stable from
// I_start until O_done. A new start restarts conversion at the newest snapshot.
// FPS inputs are bounded to 0..999 by video_realtime_monitor. Other fields show
// their low four decimal digits, matching the original modulo-10000 display.
module osd_decimal_formatter (
    input wire I_clk, I_rst_n, I_start,
    input wire [95:0] I_values,
    output reg [11:0] O_capture_bcd, O_display_bcd, O_new_bcd,
    output reg [15:0] O_latency_bcd, O_x_bcd, O_y_bcd,
    output reg O_done
);
    reg busy;
    reg [2:0] field;
    reg [3:0] bit_count;
    reg [15:0] binary_value, decimal_value;
    reg [15:0] adjusted, next_binary;
    wire [15:0] next_decimal = {adjusted[14:0],binary_value[15]};
    integer digit;
    always @(*) begin
        adjusted=decimal_value;
        for(digit=0;digit<4;digit=digit+1)
            if(decimal_value[digit*4 +: 4]>=4'd5)
                adjusted[digit*4 +: 4]=decimal_value[digit*4 +: 4]+4'd3;
        case(field)
            0: next_binary=I_values[79:64];
            1: next_binary=I_values[63:48];
            2: next_binary=I_values[47:32];
            3: next_binary=I_values[31:16];
            4: next_binary=I_values[15:0];
            default: next_binary=16'd0;
        endcase
    end
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            busy<=0;field<=0;bit_count<=0;binary_value<=0;decimal_value<=0;
            O_capture_bcd<=0;O_display_bcd<=0;O_new_bcd<=0;
            O_latency_bcd<=0;O_x_bcd<=0;O_y_bcd<=0;O_done<=0;
        end else begin
            O_done<=0;
            if(I_start) begin
                busy<=1;field<=0;bit_count<=0;
                binary_value<=I_values[95:80];decimal_value<=0;
            end else if(busy) begin
                binary_value<={binary_value[14:0],1'b0};
                decimal_value<=next_decimal;
                if(bit_count==15) begin
                    case(field)
                        0: O_capture_bcd<=next_decimal[11:0];
                        1: O_display_bcd<=next_decimal[11:0];
                        2: O_new_bcd<=next_decimal[11:0];
                        3: O_latency_bcd<=next_decimal;
                        4: O_x_bcd<=next_decimal;
                        5: O_y_bcd<=next_decimal;
                    endcase
                    binary_value<=next_binary;decimal_value<=0;bit_count<=0;
                    if(field==5) begin busy<=0;O_done<=1; end
                    else field<=field+1'b1;
                end else bit_count<=bit_count+1'b1;
            end
        end
    end
endmodule
