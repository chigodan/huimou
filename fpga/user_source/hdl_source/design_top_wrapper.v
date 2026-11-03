module design_top_wrapper #(
    // 0: continuous-speed pan + position pitch; 1: horizontal speed calibration.
    parameter integer SERVO_MANUAL_TEST = 0,
    // Continuous servo: pulse width commands SPEED, not shaft angle.
    // TRACK_CENTER_US is the adjustable neutral (STOP) pulse, not an angle.
    // Start with a limited speed-command range; these are NOT angular limits.
    parameter integer TRACK_CENTER_US = 1500,
    parameter integer TRACK_MIN_US = 1300,
    parameter integer TRACK_MAX_US = 1700,
    parameter integer TRACK_DEADBAND_PIXELS = 12,
    // Desired speed offset = deadband-adjusted x error / 2**GAIN_SHIFT.
    // MIN_DRIVE_US compensates motor dead zone; tune on the actual servo.
    // MAX_STEP_US limits acceleration per 20 ms; stopping bypasses this ramp.
    parameter integer TRACK_GAIN_SHIFT = 2,
    parameter integer TRACK_MIN_DRIVE_US = 40,
    parameter integer TRACK_MAX_STEP_US = 8,
    // Smooth motion: filter coordinates, confirm starts/reversals on fresh
    // frames, and tolerate one missed detection without stop/start chatter.
    parameter integer TRACK_STABILITY_ENABLE = 1,
    parameter integer TRACK_FILTER_SHIFT = 1,
    parameter integer TRACK_RESTART_MARGIN_PIXELS = 12,
    parameter integer TRACK_CONFIRM_SAMPLES = 2,
    parameter integer TRACK_LOST_CONFIRM_SAMPLES = 2,
    // A confirmed far-away target must cross the motor's start dead zone.
    // Only boost from STOP; subsequent changes retain the 8 us/frame ramp.
    parameter integer TRACK_START_BOOST_US = 60,
    parameter integer TRACK_START_BOOST_ERROR_PIXELS = 160,
    // Continue between camera frames, but stop if no valid result for 100 ms.
    parameter integer TRACK_SAMPLE_TIMEOUT_US = 100_000,
    // 0: increasing pulse turns camera right; set 1 for reversed image/mechanics.
    parameter integer TRACK_REVERSE = 0,
    // Independent pitch settings. 1500 us is the calibrated forward position.
    // Use the confirmed DS3120 full pulse range; no extra mechanical travel cap.
    parameter integer PITCH_MIN_US = 500,
    parameter integer PITCH_MAX_US = 2500,
    parameter integer PITCH_DEADBAND_PIXELS = 12,
    // 8x the previous small-error gain (1/8 instead of 1/64 us per pixel).
    parameter integer PITCH_GAIN_SHIFT = 3,
    // 6x the previous rate cap: up to 24 us per fresh 20 ms control update.
    parameter integer PITCH_MAX_STEP_US = 24,
    // 0: increasing pulse tilts up. Set 1 if image/mechanics reverse this sign.
    parameter integer PITCH_REVERSE = 0
) (
    input wire        I_sys_clk,
    input wire        I_rst_n,
    input wire        I_uart_rx,
    output wire       O_uart_tx,

    output wire       O_cam_scl,
    inout  wire       IO_cam_sda,
    output wire       O_cam_24m,
    output wire       O_cam_rst,

    // Two on-board toggle switches.  The pin constraints keep the existing names:
    // I_button[0] -> SW1 (A4) and I_button[1] -> SW2 (B4).
    input wire [1:0]  I_button,
    // K1..K4 are active-low push buttons: decrease, increase, step, center.
    input wire [3:0]  I_key,

    output wire       O_screen_pwm,
    output wire       O_servo_pwm,
    output wire       O_servo_pitch_pwm,
    output wire       O_tmds_ch0_p,
    output wire       O_tmds_ch1_p,
    output wire       O_tmds_ch2_p,
    output wire       O_tmds_clk_p,

    inout wire        IO_rx_clk_pad_n,
    inout wire        IO_rx_clk_pad_p,
    inout wire[3:0]   IO_rx_data_pad_n,
    inout wire[3:0]   IO_rx_data_pad_p,

    output wire[12:0] ddr_addr,
    output wire[ 1:0] ddr_ba,
    output wire[ 0:0] ddr_cke,
    output wire[ 0:0] ddr_odt,
    output wire[ 0:0] ddr_cs_n,
    output wire       ddr_ras_n,
    output wire       ddr_cas_n,
    output wire       ddr_we_n,
    output wire[ 0:0] ddr_ck_p,
    output wire[ 0:0] ddr_ck_n,
    inout wire[1:0]   ddr_dm,
    inout wire[15:0]  ddr_dq,
    inout wire[1:0]   ddr_dqs_p,
    inout wire[1:0]   ddr_dqs_n
);

    wire        	S_100m_clk;
    wire        	S_24m_clk;

	wire        	S_pll_lock;
    wire        	S_rst_n;
	reg  [17:0] 	S_cam_power_cnt;
	reg         	S_cam_pwdnb;
	reg         	S_cam_i2c_ready;
	(* keep = "true" *) wire [15:0] S_cam_sensor_id;
	(* keep = "true" *) wire        S_cam_sensor_id_ok;
	(* keep = "true" *) wire        S_cam_i2c_error;
	localparam integer CAM_PWDNB_LOW_CYCLES = 23810;  // >= 1 ms at 23.809524 MHz
	localparam integer CAM_I2C_START_CYCLES = 142857; // release + >= 5 ms

	wire [15:0] 	S_ae;
    wire [15:0] 	S_ag;
    wire 			S_cam_cfg_done;
    wire 			S_ae_cfg_done;
    wire 			S_ae_req;

	wire        	S_csi_rx_clk;
    wire        	S_hs_rx_valid;
    wire[15:0]  	S_hs_rx_data;
	wire[1:0]   	S_lane_error;

	wire        	S_csi_frame_start;
	wire        	S_csi_frame_end;
	wire        	S_csi_valid;
	wire[31:0]  	S_csi_data;

	wire        	S_raw10_frame_start;
	wire        	S_raw10_frame_end;
	wire        	S_raw10_valid;
	wire [39:0] 	S_raw10_data;

	wire 			S_axis_tlast;
	wire 			S_axis_tuser;
	wire [39:0]		S_axis_tdata;
	wire 			S_axis_tvalid;

	wire [39:0]		S_raw_tdata ;
	wire			S_raw_tlast ;
	wire 			S_raw_tvalid;
	wire 			S_raw_tuser ;

    // The board inputs are active-low because their pins use PULLUP.
    // Logical mode values are therefore:
    // 00 camera, 01 image processing, 10 OSD, 11 circle detection.
    // Starting from edge after reset, each frame-confirmed 00 -> 01 change
    // selects the next processing effect: edge -> grayscale -> region fill.
    // Remain in 00 for at least two camera frames before returning to 01.
    wire [1:0]        S_mode;
    assign S_mode = I_button;

	wire			S_ISP_O_tready;
	wire [127:0]	S_ISP_O_tdata ;
	wire 			S_ISP_O_tlast ;
	wire 			S_ISP_O_tuser ;
	wire 			S_ISP_O_tvalid;

    wire        S_ddr_clk;
    wire        S_hdmi_pixel_clk;
    wire        S_hdmi_serial_clk;
    wire        S_hdmi_rst_n;
    wire        S_video_out_rst_n;

    wire        S_video_out_rd_busy;
    wire        S_video_in_wr_busy;
    wire[1:0]   S_video_out_rp;
    wire S_monitor_write_start, S_monitor_read_start;
    wire [1:0] S_monitor_write_slot, S_monitor_read_slot;
    wire [9:0] S_capture_fps, S_display_fps, S_new_fps;
    wire [15:0] S_capture_id, S_display_id, S_latency_ms;
    wire S_latency_valid;
    wire [1:0] S_image_flags;
    wire [7:0] S_image_status_flags;
    wire S_image_token, S_image_isp_ack, S_image_ae_ack;
    wire S_luma_sample, S_awb_report_event, S_ae_report_event;
    wire [7:0] S_luma_mean;
    wire [40:0] S_awb_report, S_awb_report_ref, S_awb_report_pixel;
    wire [37:0] S_ae_report_ref, S_ae_report_pixel;
    wire [1:0] S_realtime_state;

    wire        S_ddr_user_wr_en;
    wire        S_ddr_user_rd_en;
    wire[24:0]  S_ddr_user_addr;
    wire[127:0] S_ddr_user_wr_data;
    wire        S_ddr_user_ready;
    wire        S_ddr_user_rd_valid;
    wire[127:0] S_ddr_user_rd_data;

    wire        S_vi_ddr_wr_en;
    wire[24:0]  S_vi_ddr_wr_addr;
    wire[127:0] S_vi_ddr_wr_data;

    wire        S_vo_ddr_rd_en;
    wire[24:0]  S_vo_ddr_rd_addr;

    wire        S_init_calib_complete;
    wire[24:0]  S_mc_app_addr;
    wire[2:0]   S_mc_app_cmd;
    wire        S_mc_app_en;
    wire[127:0] S_mc_app_wdf_data;
    wire        S_mc_app_wdf_end;
    wire[15:0]  S_mc_app_wdf_mask;
    wire        S_mc_app_wdf_wren;
    wire[127:0] S_mc_app_rd_data;
    wire        S_mc_app_rd_data_end;
    wire        S_mc_app_rd_data_valid;
    wire        S_mc_app_rdy;
    wire        S_mc_app_wdf_rdy;

    wire        S_hdmi_vsync;
    wire        S_hdmi_hsync;
    wire        S_hdmi_de;
    wire        S_hdmi_user;
    wire        S_hdmi_last;
    wire        S_video_out_vsync;
    wire        S_hdmi_window_rd_en;
    wire        S_hdmi_out_vsync;
    wire        S_hdmi_out_hsync;
    wire        S_hdmi_out_de;
    wire[23:0]  S_hdmi_out_data;
    wire S_brightness_token, S_brightness_ack;
    wire [7:0] S_brightness_value;
    wire S_bright_vsync, S_bright_hsync, S_bright_de;
    wire [23:0] S_bright_data;

    wire[23:0]  S_video_rd_data;
    wire[3:0]   S_hdmi_debug_status;
    wire [11:0] S_servo_pulse_us;
    wire [11:0] S_servo_manual_us;
    wire [11:0] S_servo_track_us;
    wire [11:0] S_servo_pitch_track_us;
    wire [11:0] S_servo_pitch_us;
    wire [6:0]  S_servo_step_us;
    wire        S_servo_center_event;
    wire        S_circle_sample;
    wire        S_circle_fresh_valid;
    wire [11:0] S_circle_center_x;
    wire [11:0] S_circle_center_y;

    wire S_uart_owner, S_uart_pending, S_uart_token, S_uart_ms_tick;
    wire [2:0] S_uart_target_algo;
    wire S_uart_override=S_uart_owner || S_uart_pending;
    wire S_isp_uart_ack, S_hdmi_uart_ack, S_isp_uart_frame;
    wire [2:0] S_isp_uart_algo, S_uart_observed_algo;
    wire [2:0] S_hdmi_isp_algo;
    reg S_isp_status_publish;
    // Sample one camera clock after SOF, once the ISP's selected algorithm
    // registers have updated. Transfer the 3-bit state coherently to HDMI.
    always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
        if(!S_rst_n) S_isp_status_publish <= 1'b0;
        else S_isp_status_publish <= S_isp_uart_frame;
    end
    uart_snapshot_mailbox #(.WIDTH(3)) u_osd_isp_status (
        .src_clk(S_csi_rx_clk),.src_rst_n(S_rst_n),
        .src_valid(S_isp_status_publish),.src_data(S_isp_uart_algo),
        .dst_clk(S_hdmi_pixel_clk),.dst_rst_n(S_hdmi_rst_n),
        .dst_valid(),.dst_data(S_hdmi_isp_algo));
    wire [1:0] S_hdmi_uart_mode, S_uart_result_mode;
    reg S_servo_tracking_pixel;
    (* ASYNC_REG = "TRUE" *) reg [1:0] S_servo_tracking_sync;
    wire S_servo_home_hold = !S_servo_tracking_sync[1];
    wire S_servo_home = S_servo_center_event || S_servo_home_hold;
    wire S_servo_fresh_valid = S_circle_fresh_valid && (S_hdmi_uart_mode == 2'b11);
    // Use the display's committed mode, including UART overrides. Synchronize
    // one tracking-active bit rather than the two-bit mode bus into 100 MHz.
    always @(posedge S_hdmi_pixel_clk or negedge S_hdmi_rst_n) begin
        if(!S_hdmi_rst_n) S_servo_tracking_pixel <= 1'b0;
        else S_servo_tracking_pixel <= (S_hdmi_uart_mode == 2'b11);
    end
    always @(posedge S_100m_clk or negedge S_rst_n) begin
        if(!S_rst_n) S_servo_tracking_sync <= 2'b00;
        else S_servo_tracking_sync <= {S_servo_tracking_sync[0],S_servo_tracking_pixel};
    end
    // Hold home throughout non-tracking modes to discard pending coordinates.
    // Pan requests STOP and pitch returns home; target loss holds pitch only.
    wire S_uart_video_alive, S_uart_result_fresh, S_uart_result_token;
    wire [11:0] S_uart_center_x, S_uart_center_y;
    wire [15:0] S_uart_sample_age;
    wire [31:0] S_uart_frame_id, S_uart_sample_id;
    uart_video_status u_uart_video_status (
        .clk(S_100m_clk),.rst_n(S_rst_n),.ms_tick(S_uart_ms_tick),
        .pixel_clk(S_hdmi_pixel_clk),.pixel_rst_n(S_hdmi_rst_n),.frame_start(S_hdmi_user),
        .sample(S_circle_sample),.fresh_valid(S_circle_fresh_valid),
        .center_x(S_circle_center_x),.center_y(S_circle_center_y),
        .display_mode(S_hdmi_uart_mode),.display_token(S_hdmi_uart_ack),
        .isp_clk(S_csi_rx_clk),.isp_rst_n(S_rst_n),.isp_algo(S_isp_uart_algo),
        .isp_frame_start(S_isp_uart_frame),.observed_algo(S_uart_observed_algo),
        .video_alive(S_uart_video_alive),.frame_id(S_uart_frame_id),.sample_id(S_uart_sample_id),
        .x(S_uart_center_x),.y(S_uart_center_y),.fresh(S_uart_result_fresh),
        .token(S_uart_result_token),.mode(S_uart_result_mode),.sample_age(S_uart_sample_age));
    uart_control #(.CLK_HZ(100_000_000),.BAUD(115200),
        .ALGO_ENABLED(SERVO_MANUAL_TEST==0),.TRACKING_ENABLED(SERVO_MANUAL_TEST==0),
        .IMAGE_AUTO_ENABLED(SERVO_MANUAL_TEST==0)) u_uart_control (
        .clk(S_100m_clk),.rst_n(S_rst_n),.rx(I_uart_rx),.tx(O_uart_tx),.ms_tick(S_uart_ms_tick),
        .owner(S_uart_owner),.pending(S_uart_pending),.token(S_uart_token),.target_algo(S_uart_target_algo),
        .target_brightness(S_brightness_value),.brightness_token(S_brightness_token),
        .brightness_ack(S_brightness_ack),
        .target_image_flags(S_image_flags),.image_token(S_image_token),
        .image_isp_ack(S_image_isp_ack),.image_ae_ack(S_image_ae_ack),
        .image_flags(S_image_status_flags),.image_state(S_ae_report_ref[34:32]),
        .image_mean(S_ae_report_ref[31:24]),.image_exposure(S_ae_report_ref[23:8]),
        .image_gain(S_ae_report_ref[7:0]),
        .image_r_gain(S_awb_report_ref[39:20]),.image_b_gain(S_awb_report_ref[19:0]),
        .isp_ack(S_isp_uart_ack),.display_ack(S_hdmi_uart_ack),.observed_algo(S_uart_observed_algo),
        .video_alive(S_uart_video_alive),.result_fresh(S_uart_result_fresh),
        .result_token(S_uart_result_token),.result_mode(S_uart_result_mode),
        .center_x(S_uart_center_x),.center_y(S_uart_center_y),.sample_age(S_uart_sample_age),
        .frame_id(S_uart_frame_id),.sample_id(S_uart_sample_id));

    // Mixer output is registered after two pixel delays; brightness adds one.
    // Three delays of input SOF identify the first pixel handed to HDMI TX.
    reg [2:0] S_monitor_display_delay;
    always @(posedge S_hdmi_pixel_clk or negedge S_rst_n) begin
        if(!S_rst_n) S_monitor_display_delay <= 0;
        else S_monitor_display_delay <= {S_monitor_display_delay[1:0],S_hdmi_user};
    end
    video_realtime_monitor #(.REF_CLK_HZ(100_000_000),.FRAME_WORDS(172800)) u_realtime_monitor (
        .I_rst_n(S_rst_n),.I_ref_clk(S_100m_clk),.I_camera_clk(S_csi_rx_clk),
        .I_ddr_clk(S_ddr_clk),.I_pixel_clk(S_hdmi_pixel_clk),
        .I_capture_start(S_raw_tuser && S_raw_tvalid),
        // The 96-to-128 packer announces SOF before its first valid word.
        // Match video_in's frame event; qualifying with valid would lose tags.
        .I_processed_start(S_ISP_O_tuser),
        .I_write_frame_start(S_monitor_write_start),.I_write_slot(S_monitor_write_slot),
        .I_write_word(S_vi_ddr_wr_en),.I_write_addr(S_vi_ddr_wr_addr),
        .I_read_frame_start(S_monitor_read_start),.I_read_slot(S_monitor_read_slot),
        .I_display_start(S_monitor_display_delay[2]),
        .O_capture_fps(S_capture_fps),.O_display_fps(S_display_fps),.O_new_fps(S_new_fps),
        .O_capture_id(S_capture_id),.O_display_id(S_display_id),
        .O_latency_ms(S_latency_ms),.O_latency_valid(S_latency_valid),.O_video_state(S_realtime_state));

    assign O_screen_pwm = 1'b1;
    assign S_video_out_vsync = ~S_hdmi_vsync;
    assign S_servo_pulse_us = SERVO_MANUAL_TEST ? S_servo_manual_us : S_servo_track_us;
    // Manual test controls horizontal speed only; pitch stays forward.
    assign S_servo_pitch_us = SERVO_MANUAL_TEST ? 12'd1500 : S_servo_pitch_track_us;

    servo_test_control #(
        .CLK_FREQ_HZ ( 100_000_000 ),
        .CENTER_US ( TRACK_CENTER_US ),
        .MIN_US ( TRACK_MIN_US ),
        .MAX_US ( TRACK_MAX_US )
    ) u_servo_test_control (
        .I_clk      ( S_100m_clk       ),
        .I_rst_n    ( S_rst_n           ),
        .I_key_n    ( I_key             ),
        .O_pulse_us ( S_servo_manual_us ),
        .O_step_us  ( S_servo_step_us   ),
        .O_center_event ( S_servo_center_event )
    );

    // Circle X is visual feedback for pan speed, not absolute shaft angle.
    // Outside the deadband command proportional speed; inside it command STOP.
    // Isolated missed results are tolerated; repeated loss or timeout stops.
    servo_circle_track #(
        .CLK_FREQ_HZ ( 100_000_000 ),
        .IMAGE_WIDTH ( 1280 ),
        .CENTER_X ( 640 ),
        .CONTROL_PERIOD_US ( 20_000 ),
        .CENTER_US ( TRACK_CENTER_US ),
        .MIN_US ( TRACK_MIN_US ),
        .MAX_US ( TRACK_MAX_US ),
        .DEADBAND_PIXELS ( TRACK_DEADBAND_PIXELS ),
        .GAIN_SHIFT ( TRACK_GAIN_SHIFT ),
        .MAX_STEP_US ( TRACK_MAX_STEP_US ),
        .REVERSE ( TRACK_REVERSE ),
        .SPEED_CONTROL ( 1 ),
        .STABILITY_ENABLE ( TRACK_STABILITY_ENABLE ),
        .FILTER_SHIFT ( TRACK_FILTER_SHIFT ),
        .RESTART_MARGIN_PIXELS ( TRACK_RESTART_MARGIN_PIXELS ),
        .CONFIRM_SAMPLES ( TRACK_CONFIRM_SAMPLES ),
        .LOST_CONFIRM_SAMPLES ( TRACK_LOST_CONFIRM_SAMPLES ),
        .START_BOOST_US ( TRACK_START_BOOST_US ),
        .START_BOOST_ERROR_PIXELS ( TRACK_START_BOOST_ERROR_PIXELS ),
        .MIN_DRIVE_US ( TRACK_MIN_DRIVE_US ),
        .SAMPLE_TIMEOUT_US ( TRACK_SAMPLE_TIMEOUT_US )
    ) u_servo_circle_track (
        .I_pixel_clk ( S_hdmi_pixel_clk ),
        .I_pixel_rst_n ( S_hdmi_rst_n ),
        .I_sample ( S_circle_sample ),
        .I_fresh_valid ( S_servo_fresh_valid ),
        .I_center_x ( S_circle_center_x ),
        .I_clk ( S_100m_clk ),
        .I_rst_n ( S_rst_n ),
        .I_center_event ( S_servo_home ),
        .O_pulse_us ( S_servo_track_us )
    );

    // Reuse the tested coordinate controller unchanged. Its X-named ports now
    // carry Y for this instance. Image Y grows DOWN, so default REVERSE is 1:
    // pitch error = 360 - cy, positive above centre -> larger pulse -> tilt UP.
    // Pitch retains position control and consumes each fresh result once.
    servo_circle_track #(
        .CLK_FREQ_HZ ( 100_000_000 ),
        .IMAGE_WIDTH ( 720 ),
        .CENTER_X ( 360 ),
        .CONTROL_PERIOD_US ( 20_000 ),
        .CENTER_US ( 1500 ),
        .MIN_US ( PITCH_MIN_US ),
        .MAX_US ( PITCH_MAX_US ),
        .DEADBAND_PIXELS ( PITCH_DEADBAND_PIXELS ),
        .GAIN_SHIFT ( PITCH_GAIN_SHIFT ),
        .MAX_STEP_US ( PITCH_MAX_STEP_US ),
        .REVERSE ( PITCH_REVERSE == 0 ? 1 : 0 )
    ) u_servo_pitch_track (
        .I_pixel_clk ( S_hdmi_pixel_clk ),
        .I_pixel_rst_n ( S_hdmi_rst_n ),
        .I_sample ( S_circle_sample ),
        .I_fresh_valid ( S_servo_fresh_valid ),
        .I_center_x ( S_circle_center_y ),
        .I_clk ( S_100m_clk ),
        .I_rst_n ( S_rst_n ),
        .I_center_event ( S_servo_home ),
        .O_pulse_us ( S_servo_pitch_track_us )
    );

    servo_pwm #(
        .CLK_FREQ_HZ ( 100_000_000 ),
        .FRAME_US    ( 20_000      ),
        .MIN_PULSE_US( TRACK_MIN_US ),
        .MAX_PULSE_US( TRACK_MAX_US ),
        .START_PULSE_US( TRACK_CENTER_US )
    ) u_servo_pwm (
        .I_clk      ( S_100m_clk      ),
        .I_rst_n    ( S_rst_n          ),
        .I_pulse_us ( S_servo_pulse_us ),
        .O_pwm      ( O_servo_pwm      )
    );

    // Separate 50 Hz pulse channel; never share one PWM signal between axes.
    servo_pwm #(
        .CLK_FREQ_HZ ( 100_000_000 ),
        .FRAME_US ( 20_000 ),
        .MIN_PULSE_US ( 500 ),
        .MAX_PULSE_US ( 2500 )
    ) u_servo_pitch_pwm (
        .I_clk ( S_100m_clk ),
        .I_rst_n ( S_rst_n ),
        .I_pulse_us ( S_servo_pitch_us ),
        .O_pwm ( O_servo_pitch_pwm )
    );

    assign S_rst_n 		= S_pll_lock;
    assign S_hdmi_rst_n = S_pll_lock;

    assign S_hdmi_debug_status = {
        (|S_ddr_user_rd_data),
        S_ddr_user_rd_valid,
        S_vo_ddr_rd_en,
        S_vi_ddr_wr_en
    };

	// SC520CS上电后先保持PWDNB/XSHUTDN为低，再释放并等待内部上电完成，
	// 避免PLL刚锁定就开始I2C初始化而造成寄存器偶发写入失败。
	always @(posedge S_24m_clk or negedge S_pll_lock) begin
		if (!S_pll_lock) begin
			S_cam_power_cnt <= 18'd0;
			S_cam_pwdnb <= 1'b0;
			S_cam_i2c_ready <= 1'b0;
		end else begin
			if (S_cam_power_cnt < CAM_I2C_START_CYCLES)
				S_cam_power_cnt <= S_cam_power_cnt + 1'b1;
			if (S_cam_power_cnt >= CAM_PWDNB_LOW_CYCLES)
				S_cam_pwdnb <= 1'b1;
			if (S_cam_power_cnt >= CAM_I2C_START_CYCLES - 1)
				S_cam_i2c_ready <= 1'b1;
		end
	end

	assign O_cam_rst 	= S_cam_pwdnb;
    assign O_cam_24m 	= S_24m_clk;

    PLL u_PLL(
        .refclk      ( I_sys_clk         ),
        .reset    	 ( ~I_rst_n          ),

        .clk0_out    ( S_100m_clk        ),
        .clk1_out    ( S_24m_clk         ),

        .clk4_out    ( S_hdmi_pixel_clk  ),
        .clk5_out    ( S_hdmi_serial_clk ),

        .lock        ( S_pll_lock        )

    );

    // Image statistics and sensor control are independent of the HDMI-only
    // display brightness. Each cross-domain report is a held coherent mailbox.
    wire [1:0] S_ae_image_flags;
    wire S_mean_event, S_ae_report_tick;
    wire [7:0] S_mean_sensor;
    wire [15:0] S_applied_exposure;
    wire [7:0] S_applied_gain, S_measured_luma;
    wire S_sensor_values_valid, S_mean_valid;
    wire [2:0] S_ae_state;
    wire [37:0] S_sensor_report={S_ae_image_flags[1],S_mean_valid,
        S_sensor_values_valid,S_ae_state,S_measured_luma,S_applied_exposure,S_applied_gain};
    uart_snapshot_mailbox #(.WIDTH(8)) u_mean_mailbox (
        .src_clk(S_csi_rx_clk),.src_rst_n(S_rst_n),.src_valid(S_luma_sample),.src_data(S_luma_mean),
        .dst_clk(S_24m_clk),.dst_rst_n(S_rst_n),.dst_valid(S_mean_event),.dst_data(S_mean_sensor));
    image_frame_settings u_ae_image_settings (
        .clk(S_24m_clk),.rst_n(S_rst_n),.frame_start(S_mean_event),
        .token_async(S_image_token),.flags_async(S_image_flags),
        .flags(S_ae_image_flags),.ack(S_image_ae_ack));
    camera_auto_exposure u_camera_auto_exposure (
        .clk(S_24m_clk),.rst_n(S_rst_n),.enabled(S_ae_image_flags[1]),
        .sensor_ready(S_cam_cfg_done),.write_ready(S_ae_cfg_done),.i2c_error(S_cam_i2c_error),
        .sample_valid(S_mean_event),.mean_luma(S_mean_sensor),
        .request(S_ae_req),.exposure(S_ae),.gain(S_ag),
        .applied_exposure(S_applied_exposure),.applied_gain(S_applied_gain),
        .applied_valid(S_sensor_values_valid),.mean_valid(S_mean_valid),
        .measured_luma(S_measured_luma),.state(S_ae_state),.report_tick(S_ae_report_tick));
    uart_snapshot_mailbox #(.WIDTH(41)) u_awb_report_ref (
        .src_clk(S_csi_rx_clk),.src_rst_n(S_rst_n),.src_valid(S_luma_sample),.src_data(S_awb_report),
        .dst_clk(S_100m_clk),.dst_rst_n(S_rst_n),.dst_valid(S_awb_report_event),.dst_data(S_awb_report_ref));
    uart_snapshot_mailbox #(.WIDTH(38)) u_ae_report_ref (
        .src_clk(S_24m_clk),.src_rst_n(S_rst_n),.src_valid(S_ae_report_tick),.src_data(S_sensor_report),
        .dst_clk(S_100m_clk),.dst_rst_n(S_rst_n),.dst_valid(S_ae_report_event),.dst_data(S_ae_report_ref));
    uart_snapshot_mailbox #(.WIDTH(41)) u_awb_report_pixel (
        .src_clk(S_csi_rx_clk),.src_rst_n(S_rst_n),.src_valid(S_luma_sample),.src_data(S_awb_report),
        .dst_clk(S_hdmi_pixel_clk),.dst_rst_n(S_hdmi_rst_n),.dst_valid(),.dst_data(S_awb_report_pixel));
    uart_snapshot_mailbox #(.WIDTH(38)) u_ae_report_pixel (
        .src_clk(S_24m_clk),.src_rst_n(S_rst_n),.src_valid(S_ae_report_tick),.src_data(S_sensor_report),
        .dst_clk(S_hdmi_pixel_clk),.dst_rst_n(S_hdmi_rst_n),.dst_valid(),.dst_data(S_ae_report_pixel));
    reg S_have_awb_report;
    reg [9:0] S_image_report_age;
    always @(posedge S_100m_clk or negedge S_rst_n) begin
        if (!S_rst_n) begin S_have_awb_report<=0;S_image_report_age<=500;end
        else begin
            if (S_awb_report_event) S_have_awb_report<=1;
            if (S_ae_report_event) S_image_report_age<=0;
            else if (S_uart_ms_tick && S_image_report_age<500) S_image_report_age<=S_image_report_age+1'b1;
        end
    end
    assign S_image_status_flags={3'd0,
        (S_have_awb_report && S_image_report_age<500),S_ae_report_ref[35],S_ae_report_ref[36],
        S_ae_report_ref[37],S_awb_report_ref[40]};

    // SC520CS原厂真实HD模式：1280x720@60、RAW10、2-Lane、828 Mbps/Lane。
    // 当前寄存器表已关闭Slave/FSIN等待，Sensor以free-run方式连续输出。
    uicfgcs520 #(
      .CLK_DIV(23809524 / 100000 - 1)
  ) u_uicfgcs520 (
      .I_clk(S_24m_clk),  //系统时钟输入
      .I_rst_n(S_cam_i2c_ready), // SC520CS power-up delay completed
      .I_ae_req(S_ae_req),
      .I_ae(S_ae),
      .I_ag(S_ag),
      .O_cam_scl(O_cam_scl),  //I2C总线，SCL时钟
      .IO_cam_sda(IO_cam_sda),  //I2C总线，SDA数据
      .O_cfg_done(S_cam_cfg_done),  //摄像头寄存器初始化完成
      .O_ae_cfg_done(S_ae_cfg_done),  //AE配置完成
      .O_sensor_id(S_cam_sensor_id),
      .O_sensor_id_ok(S_cam_sensor_id_ok),
      .O_iic_error(S_cam_i2c_error)
  );

    mipi_dphy_rx_ph1p_mipiio_wrapper#(
        .DPHY_RX_LOCATION      ( "DPHY0" ),
        .HS_EQUALIZER          ( "3dB"   ),// 828 Mbps/Lane接收均衡
        .HS_VGA_GAIN           ( "8dB" ),//"-3dB","-1.5dB","0dB","1.5dB","3dB","4.5dB","6dB","7.5dB"
        .LANE_NUM              ( 2       ),
        .BYTE_NUM              ( 1       )
    )u_mipi_dphy_rx_ph1p_mipiio_wrapper(
        .I_lp_clk              ( S_100m_clk       ),
        .I_rst                 ( ~S_rst_n         ),

        .I_clk_lane_in_delay   ( 6'd0             ),
        .I_data_lane0_in_delay ( 6'd0             ),
        .I_data_lane1_in_delay ( 6'd0             ),
        .I_data_lane2_in_delay ( 6'd0             ),
        .I_data_lane3_in_delay ( 6'd0             ),

        .I_lane_invert         ( 4'b0000          ),

        .O_hs_rx_clk           ( S_csi_rx_clk     ),
        .O_hs_rx_valid         ( S_hs_rx_valid    ),
        .O_hs_rx_data          ( S_hs_rx_data     ),

        .O_lp_rx_lane0_p       (  ),
        .O_lp_rx_lane0_n       (  ),

        .I_lp_tx_en            ( 1'b0             ),
        .I_lp_tx_lane0_p       ( 1'b1             ),
        .I_lp_tx_lane0_n       ( 1'b1             ),

        .O_lane_match_error    (                  ),
        .O_lane_error          ( S_lane_error     ),

        .IO_rx_clk_pad_n       ( IO_rx_clk_pad_n  ),
        .IO_rx_clk_pad_p       ( IO_rx_clk_pad_p  ),
        .IO_rx_data_pad_n      ( IO_rx_data_pad_n ),
        .IO_rx_data_pad_p      ( IO_rx_data_pad_p )
    );

 //csi 解码为RAW数据
csi_unpacket_2lane u_csi_unpacket(
.I_clk                 ( S_csi_rx_clk       ),
.I_rst_n               ( S_rst_n        	),
.I_hs_valid            ( S_hs_rx_valid     ),
.I_hs_data             ( S_hs_rx_data      ),

.O_csi_frame_start     ( S_csi_frame_start ),
.O_csi_frame_end       ( S_csi_frame_end   ),
.O_csi_valid           ( S_csi_valid       ),
.O_csi_data            ( S_csi_data        )
);

//解码为RAW8
raw10_unpacket_2lane u_raw10_unpacket (
.I_clk  (S_csi_rx_clk),
.I_rst_n(S_rst_n),

.I_csi_frame_start(S_csi_frame_start),
.I_csi_frame_end  (S_csi_frame_end),
.I_csi_valid      (S_csi_valid),
.I_csi_data       (S_csi_data),

.O_raw10_frame_start(S_raw10_frame_start),
.O_raw10_frame_end  (S_raw10_frame_end),
.O_raw10_valid      (S_raw10_valid),
.O_raw10_data       (S_raw10_data)
  );

//将数据转为stream流
uial2axis #(
.IMG_WIDTH(1280),
.IMG_HEIGHT(720),
.INPUT_DATA_WIDTH(40)
)
u_uial2axis (
.I_native_clk(S_csi_rx_clk),
.I_rst_n     (S_rst_n),
.I_data      (S_raw10_data       ),
.I_data_valid(S_raw10_valid      ),
.I_data_start(S_raw10_frame_start),
.I_data_end  (S_raw10_frame_end  ),
.axis_tvalid (S_axis_tvalid),
.axis_tdata  (S_axis_tdata ),
.axis_tuser  (S_axis_tuser ),
.axis_tlast  (S_axis_tlast )
);

// 原厂HD模式直接输出1280x720 RAW10，不再进行FPGA固定窗口裁剪。
assign S_raw_tdata  = S_axis_tdata;
assign S_raw_tlast  = S_axis_tlast;
assign S_raw_tvalid = S_axis_tvalid;
assign S_raw_tuser  = S_axis_tuser;

//ISP算法顶层模块
isp_top #(.IMAGE_AUTO_ENABLE(1)) u_isp_top (
.axi4s_video_aclk(S_csi_rx_clk),
.I_rst_n         (S_rst_n),
.I_mode          (S_mode),
.I_uart_override (S_uart_override),
.I_uart_token    (S_uart_token),
.I_uart_algo     (S_uart_target_algo),
.O_uart_ack      (S_isp_uart_ack),
.O_uart_report_algo (S_isp_uart_algo),
.O_uart_frame_start (S_isp_uart_frame),
.I_image_flags(S_image_flags),.I_image_token(S_image_token),.O_image_ack(S_image_isp_ack),
.O_luma_sample(S_luma_sample),.O_mean_luma(S_luma_mean),.O_awb_report(S_awb_report),
.I_tlast         (S_raw_tlast	),
.I_tuser         (S_raw_tuser	),
.I_tdata         (S_raw_tdata	),
.I_tvalid        (S_raw_tvalid	),
.I_tdest         (),
.O_tready        (S_ISP_O_tready),
.O_tdata         (S_ISP_O_tdata ),
.O_tlast         (S_ISP_O_tlast ),
.O_tuser         (S_ISP_O_tuser ),
.O_tvalid        (S_ISP_O_tvalid),
.I_tready        ()
  );

    video_in u_video_in(
        .I_rst_n              (  S_rst_n             ),

        .I_camera_clk         ( S_csi_rx_clk          ),
        .I_camera_frame_start ( S_ISP_O_tuser  ),
        .I_camera_valid       ( S_ISP_O_tvalid ),
        .I_camera_data        ( S_ISP_O_tdata  ),
        .I_mipi_rx_error      ( 1'b0			      ),

        .I_ddr_clk            ( S_ddr_clk             ),
		.I_display_pause      ( 1'b0                  ),
        .I_video_out_rd_busy  ( S_video_out_rd_busy   ),
        .O_video_in_wr_busy   ( S_video_in_wr_busy    ),
        .O_video_out_rp       ( S_video_out_rp        ),
        .O_monitor_frame_start(S_monitor_write_start),.O_monitor_slot(S_monitor_write_slot),

        .O_ddr_user_wr_en     ( S_vi_ddr_wr_en        ),
        .O_ddr_user_addr      ( S_vi_ddr_wr_addr      ),
        .O_ddr_user_wr_data   ( S_vi_ddr_wr_data      ),
        .I_ddr_user_ready     ( S_ddr_user_ready      )
    );

    assign S_ddr_user_wr_en    = S_vi_ddr_wr_en;

    assign S_ddr_user_rd_en    = S_vo_ddr_rd_en;

    assign S_ddr_user_addr     = S_vi_ddr_wr_en ? S_vi_ddr_wr_addr :
                                 S_vo_ddr_rd_en ? S_vo_ddr_rd_addr : 'd0;

    assign S_ddr_user_wr_data  = S_vi_ddr_wr_data;

    assign S_video_out_rst_n = S_hdmi_rst_n;

    video_out u_video_out(
        .I_rst_n             ( S_video_out_rst_n    ),
        .I_ddr_clk           ( S_ddr_clk            ),

        .O_video_out_rd_busy ( S_video_out_rd_busy  ),
        .I_video_in_wr_busy  ( S_video_in_wr_busy   ),
        .I_video_out_rp      ( S_video_out_rp       ),
        .O_monitor_frame_start(S_monitor_read_start),.O_monitor_slot(S_monitor_read_slot),

        .O_ddr_user_rd_en    ( S_vo_ddr_rd_en       ),
        .O_ddr_user_addr     ( S_vo_ddr_rd_addr     ),
        .I_ddr_user_ready    ( S_ddr_user_ready     ),
        .I_ddr_user_rd_valid ( S_ddr_user_rd_valid  ),
        .I_ddr_user_rd_data  ( S_ddr_user_rd_data   ),

        .I_dsi_clk           ( S_hdmi_pixel_clk     ),
        .I_video_vsync       ( S_video_out_vsync    ),
        .I_video_rd_en       ( S_hdmi_window_rd_en  ),
        .O_vdieo_data        ( S_video_rd_data      )
    );

    mc_to_user_interface u_mc_to_user_interface(
        .I_clk                   ( S_ddr_clk               ),
        .I_rst_n                 ( S_rst_n                 ),

        .I_ddr_user_wr_en        ( S_ddr_user_wr_en        ),
        .I_ddr_user_rd_en        ( S_ddr_user_rd_en        ),
        .I_ddr_user_addr         ( S_ddr_user_addr         ),
        .I_ddr_user_wr_data      ( S_ddr_user_wr_data      ),
        .O_ddr_user_ready        ( S_ddr_user_ready        ),
        .O_ddr_user_rd_valid     ( S_ddr_user_rd_valid     ),
        .O_ddr_user_rd_data      ( S_ddr_user_rd_data      ),

        .O_mc_app_en             ( S_mc_app_en             ),
        .O_mc_app_addr           ( S_mc_app_addr           ),
        .O_mc_app_cmd            ( S_mc_app_cmd            ),
        .I_mc_app_rdy            ( S_mc_app_rdy            ),
        .O_mc_app_wdf_wren       ( S_mc_app_wdf_wren       ),
        .O_mc_app_wdf_data       ( S_mc_app_wdf_data       ),
        .O_mc_app_wdf_end        ( S_mc_app_wdf_end        ),
        .O_mc_app_wdf_mask       ( S_mc_app_wdf_mask       ),
        .I_mc_app_wdf_rdy        ( S_mc_app_wdf_rdy        ),
        .I_mc_app_rd_data        ( S_mc_app_rd_data        ),
        .I_mc_app_rd_data_end    ( S_mc_app_rd_data_end    ),
        .I_mc_app_rd_data_valid  ( S_mc_app_rd_data_valid  )
    );

    ph1p35_324_ddr_wrapper u_ph1p35_324_ddr_wrapper(
        .I_sys_clk              ( I_sys_clk              ),
        .I_sys_rst_n            ( S_rst_n                ),

        .O_ddr_clk              ( S_ddr_clk              ),
        .O_init_calib_complete  ( S_init_calib_complete  ),
        .I_mc_app_addr          ( S_mc_app_addr          ),
        .I_mc_app_cmd           ( S_mc_app_cmd           ),
        .I_mc_app_en            ( S_mc_app_en            ),
        .I_mc_app_wdf_data      ( S_mc_app_wdf_data      ),
        .I_mc_app_wdf_end       ( S_mc_app_wdf_end       ),
        .I_mc_app_wdf_mask      ( S_mc_app_wdf_mask      ),
        .I_mc_app_wdf_wren      ( S_mc_app_wdf_wren      ),
        .O_mc_app_rd_data       ( S_mc_app_rd_data       ),
        .O_mc_app_rd_data_end   ( S_mc_app_rd_data_end   ),
        .O_mc_app_rd_data_valid ( S_mc_app_rd_data_valid ),
        .O_mc_app_rdy           ( S_mc_app_rdy           ),
        .O_mc_app_wdf_rdy       ( S_mc_app_wdf_rdy       ),

        .ddr_addr               ( ddr_addr               ),
        .ddr_ba                 ( ddr_ba                 ),
        .ddr_cke                ( ddr_cke                ),
        .ddr_odt                ( ddr_odt                ),
        .ddr_cs_n               ( ddr_cs_n               ),
        .ddr_ras_n              ( ddr_ras_n              ),
        .ddr_cas_n              ( ddr_cas_n              ),
        .ddr_we_n               ( ddr_we_n               ),
        .ddr_ck_p               ( ddr_ck_p               ),
        .ddr_ck_n               ( ddr_ck_n               ),
        .ddr_dm                 ( ddr_dm                 ),
        .ddr_dq                 ( ddr_dq                 ),
        .ddr_dqs_p              ( ddr_dqs_p              ),
        .ddr_dqs_n              ( ddr_dqs_n              )
    );

    uivtc #(
        .H_ActiveSize ( 1280 ),
        .H_FrameSize  ( 1650 ),
        .H_SyncStart  ( 1390 ),
        .H_SyncEnd    ( 1430 ),
        .V_ActiveSize ( 720  ),
        .V_FrameSize  ( 750  ),
        .V_SyncStart  ( 725  ),
        .V_SyncEnd    ( 730  )
    )u_hdmi_vtc(
        .I_vtc_rstn    ( S_hdmi_rst_n    ),
        .I_vtc_clk     ( S_hdmi_pixel_clk ),
        .O_vtc_vs      ( S_hdmi_vsync     ),
        .O_vtc_hs      ( S_hdmi_hsync     ),
        .O_vtc_de_valid( S_hdmi_de        ),
        .O_vtc_user    ( S_hdmi_user      ),
        .O_vtc_last    ( S_hdmi_last      )
    );

    generate if(SERVO_MANUAL_TEST != 0) begin : g_servo_manual
    assign S_hdmi_uart_ack=1'b0;
    assign S_hdmi_uart_mode=2'b00;
    assign S_hdmi_window_rd_en = 1'b0;
    assign S_circle_sample = 1'b0;
    assign S_circle_fresh_valid = 1'b0;
    assign S_circle_center_x = 12'd640;
    assign S_circle_center_y = 12'd360;
    servo_test_display #(.CENTER_US(TRACK_CENTER_US)) u_servo_test_display (
        .I_clk      ( S_hdmi_pixel_clk  ),
        .I_rst_n    ( S_hdmi_rst_n      ),
        .I_vsync    ( S_hdmi_vsync      ),
        .I_hsync    ( S_hdmi_hsync      ),
        .I_de       ( S_hdmi_de         ),
        .I_user     ( S_hdmi_user       ),
        .I_last     ( S_hdmi_last       ),
        .I_pulse_us ( S_servo_pulse_us  ),
        .I_step_us  ( S_servo_step_us   ),
        .O_vsync    ( S_hdmi_out_vsync  ),
        .O_hsync    ( S_hdmi_out_hsync  ),
        .O_de       ( S_hdmi_out_de     ),
        .O_data     ( S_hdmi_out_data   )
    );
    end else begin : g_servo_track
    // Restore the camera/display path. Modes 00/01/10 keep their original
    // camera, arithmetic and OSD functions; only mode 11 can move the servo.
    hdmi_mixer #(
        .H_OFFSET ( 0 ), .V_OFFSET ( 0 ),
        .IMG_WIDTH ( 1280 ), .IMG_HEIGHT ( 720 ),
        .DEBUG_MODE ( 0 ),.IMAGE_OSD_ENABLE(1),
        .CIRCLE_SEGMENT_MODE ( 0 ), .CIRCLE_LUMA_THRESHOLD ( 8'd55 )
    ) u_hdmi_mixer (
        .I_clk ( S_hdmi_pixel_clk ), .I_rst_n ( S_hdmi_rst_n ),
        .I_mode ( S_mode ),
        .I_process_algo ( S_hdmi_isp_algo ),
        .I_capture_fps(S_capture_fps),.I_display_fps(S_display_fps),.I_new_fps(S_new_fps),
        .I_capture_id(S_capture_id),.I_display_id(S_display_id),
        .I_latency_ms(S_latency_ms),.I_latency_valid(S_latency_valid),.I_video_state(S_realtime_state),
        .I_image_flags({S_ae_report_pixel[37],S_awb_report_pixel[40]}),
        .I_ae_state(S_ae_report_pixel[34:32]),
        .I_uart_override(S_uart_override),.I_uart_token(S_uart_token),.I_uart_algo(S_uart_target_algo),
        .O_uart_ack(S_hdmi_uart_ack),.O_uart_report_mode(S_hdmi_uart_mode),
        .I_video_vsync ( S_hdmi_vsync ), .I_video_hsync ( S_hdmi_hsync ),
        .I_video_de ( S_hdmi_de ), .I_video_user ( S_hdmi_user ),
        .I_video_last ( S_hdmi_last ), .I_debug_status ( S_hdmi_debug_status ),
        .O_video_rd_en ( S_hdmi_window_rd_en ), .I_video_rd_data ( S_video_rd_data ),
        .O_hdmi_vsync ( S_hdmi_out_vsync ), .O_hdmi_hsync ( S_hdmi_out_hsync ),
        .O_hdmi_de ( S_hdmi_out_de ), .O_hdmi_data ( S_hdmi_out_data ),
        .O_circle_sample ( S_circle_sample ),
        .O_circle_fresh_valid ( S_circle_fresh_valid ),
        .O_circle_center_x ( S_circle_center_x ),
        .O_circle_center_y ( S_circle_center_y )
    );
    end endgenerate

    // Post-processing only: recognition/ISP/servo inputs are untouched.
    video_brightness u_video_brightness (
        .clk(S_hdmi_pixel_clk),.rst_n(S_hdmi_rst_n && S_rst_n),.frame_start(S_hdmi_user),
        .value_async(S_brightness_value),.token_async(S_brightness_token),.ack(S_brightness_ack),
        .vsync(S_hdmi_out_vsync),.hsync(S_hdmi_out_hsync),.de(S_hdmi_out_de),.rgb(S_hdmi_out_data),
        .out_vsync(S_bright_vsync),.out_hsync(S_bright_hsync),.out_de(S_bright_de),.out_rgb(S_bright_data));

    hdmi_tx u_hdmi_tx(
        .I_pixel_clk        ( S_hdmi_pixel_clk  ),
        .I_serial_clk       ( S_hdmi_serial_clk ),
        .I_rst              ( ~S_hdmi_rst_n     ),
        .I_key_in           ( 1'b0              ),
        .I_edid_read_trig   ( 1'b0              ),
        .O_edid_read_valid  (                   ),
        .O_edid_read_data   (                   ),
        .I_video_rgb_enable ( 1'b1              ),
        .I_video_in_vs      ( S_bright_vsync    ),
        .I_video_in_de      ( S_bright_de       ),
        .I_video_in_user    ( 1'b0              ),
        .I_video_in_valid   ( 1'b0              ),
        .I_video_in_last    ( 1'b0              ),
        .O_video_in_ready   (                   ),
        .I_video_in_data    ( S_bright_data     ),
        .I_audio_valid      ( 1'b0              ),
        .I_audio_left_data  ( 24'd0             ),
        .I_audio_right_data ( 24'd0             ),
        .I_i2s_BCLK         ( 1'b0              ),
        .I_i2s_LRCK         ( 1'b0              ),
        .I_i2s_DOUT         ( 1'b0              ),
        .O_ddc_scl          (                   ),
        .O_hdmi_clk_p       (                   ),
        .O_hdmi_tx_p        (                   ),
        .O_tmds_ch0_p       ( O_tmds_ch0_p      ),
        .O_tmds_ch1_p       ( O_tmds_ch1_p      ),
        .O_tmds_ch2_p       ( O_tmds_ch2_p      ),
        .O_tmds_clk_p       ( O_tmds_clk_p      )
    );

endmodule

// Kept in the top-level source so project-list refreshes cannot omit it.
`timescale 1ns/1ps
// Display-only signed RGB offset. One registered output pixel, no frame RAM.
module video_brightness (
    input wire clk, rst_n, frame_start,
    input wire [7:0] value_async,
    input wire token_async,
    output reg ack,
    input wire vsync, hsync, de,
    input wire [23:0] rgb,
    output reg out_vsync, out_hsync, out_de,
    output reg [23:0] out_rgb
);
    reg [7:0] value_meta, value_sync, brightness;
    (* async_reg="true" *) reg token_meta, token_sync, token_settled;
    // The source holds the value through ACK; the token gets an extra cycle
    // so the bundled data settles before a frame-boundary update.
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            value_meta<=0;value_sync<=0;brightness<=0;
            token_meta<=0;token_sync<=0;token_settled<=0;ack<=0;
        end else begin
            value_meta<=value_async;value_sync<=value_meta;
            token_meta<=token_async;token_sync<=token_meta;token_settled<=token_sync;
            if(frame_start && token_settled!=ack) begin
                brightness<=value_sync;ack<=token_settled;
            end
        end
    end
    function [7:0] adjust;
        input [7:0] channel, offset;
        reg signed [9:0] sum;
        begin
            sum=$signed({2'b00,channel})+$signed({{2{offset[7]}},offset});
            adjust=sum[9] ? 8'd0 : sum[8] ? 8'd255 : sum[7:0];
        end
    endfunction
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin out_vsync<=0;out_hsync<=0;out_de<=0;out_rgb<=0;end
        else begin
            out_vsync<=vsync;out_hsync<=hsync;out_de<=de;
            out_rgb<=de ? {adjust(rgb[23:16],brightness),
                          adjust(rgb[15:8],brightness),
                          adjust(rgb[7:0],brightness)} : 24'd0;
        end
    end
endmodule
