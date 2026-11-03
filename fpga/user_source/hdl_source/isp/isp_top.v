module isp_top #(parameter integer IMAGE_AUTO_ENABLE=0) (
    input 			axi4s_video_aclk,
    input 			I_rst_n			,
    input [1:0]     I_mode          ,
    input wire     I_uart_override,
    input wire     I_uart_token,
    input wire [2:0] I_uart_algo,
    output wire    O_uart_ack,
    output wire [2:0] O_uart_report_algo,
    output wire    O_uart_frame_start,
    input wire [1:0] I_image_flags,
    input wire I_image_token,
    output wire O_image_ack,
    output wire O_luma_sample,
    output wire [7:0] O_mean_luma,
    output wire [40:0] O_awb_report,
    input 			I_tlast			,
    input 			I_tuser			,
    input [39:0] 	I_tdata			,
    input 			I_tvalid		,
    input [9:0] 	I_tdest			,
    input 			O_tready		,
    output [127:0] 	O_tdata			,
    output 			O_tlast			,
    output 			O_tuser			,
    output 			O_tvalid		,
    output 			I_tready
);
 
wire         m_aixs_tvalid;  //synthesis keep 
  wire [127:0] m_aixs_tdata;  //synthesis keep 
  wire         m_aixs_tuser;  //synthesis keep 
  wire         m_aixs_tlast;  //synthesis keep 
  wire         m_aixs_tready;  //synthesis keep 
  
  wire [ 95:0] m_aixs_tdata_96;  //synthesis keep 


  wire         awb_O_tlast;  //synthesis keep 
  wire         awb_O_tuser;  //synthesis keep 
  wire [ 95:0] awb_O_tdata;  //synthesis keep 
  wire         awb_O_tvalid;  //synthesis keep 
  wire         awb_O_tready;  //synthesis keep 

  // Low-cost colour-shading correction for the SC520/lens.
  // Horizontal magenta and vertical green casts are corrected separately;
  // this avoids the visible diamond contours caused by radial banding.
  wire [95:0]  lsc_O_tdata;   //synthesis keep
  reg  [95:0]  lsc_data_r;
  reg          lsc_tlast_r;
  reg          lsc_tuser_r;
  reg          lsc_tvalid_r;
  wire [95:0]  sat_O_tdata;
  reg  [95:0]  sat_data_r;
  reg          sat_tlast_r;
  reg          sat_tuser_r;
  reg          sat_tvalid_r;
  reg  [10:0]  lsc_x_grp;
  reg  [10:0]  lsc_y_pos;
  wire [10:0]  lsc_dx = (lsc_x_grp >= 11'd160) ?
                         (lsc_x_grp - 11'd160) : (11'd160 - lsc_x_grp);
  wire [10:0]  lsc_dy = (lsc_y_pos >= 11'd360) ?
                         (lsc_y_pos - 11'd360) : (11'd360 - lsc_y_pos);
  // One correction step is only one RGB code value, so transitions are much
  // finer than the former 3.125/6.25-percent gain bands.  x is measured in
  // four-pixel groups: dx/16 changes once per 64 pixels; dy/64 once per 64
  // lines.  Positive bias adds green at the left/right edges; negative bias
  // removes green at the centre and at the top/bottom edges.
  wire signed [6:0] lsc_bias =
      $signed({1'b0,lsc_dx[9:4]}) - 7'sd3 -
      $signed({1'b0,lsc_dy[8:6]});

  // Apply spatial chroma correction first.  In dark areas the correction is
  // reduced to avoid coloured blacks.
  // In dark areas the spatial correction is reduced to avoid coloured blacks.
  function [23:0] spatial_correct;
    input [23:0] pixel;
    input signed [6:0] spatial_bias;
    reg [9:0] lum_sum;
    reg [7:0] lum;
    reg signed [7:0] bias_eff;
    reg signed [10:0] r_work;
    reg signed [10:0] g_work;
    reg signed [10:0] b_work;
    reg [7:0] r_lsc;
    reg [7:0] g_lsc;
    reg [7:0] b_lsc;
    begin
      lum_sum = {2'b0,pixel[23:16]} +
                ({2'b0,pixel[15:8]} << 1) + {2'b0,pixel[7:0]};
      lum = lum_sum[9:2];
      if (lum < 8'd32)
        bias_eff = spatial_bias >>> 2;
      else if (lum < 8'd64)
        bias_eff = spatial_bias >>> 1;
      else
        bias_eff = spatial_bias;

      r_work = $signed({1'b0,pixel[23:16]}) - bias_eff;
      g_work = $signed({1'b0,pixel[15:8]})  + bias_eff;
      b_work = $signed({1'b0,pixel[7:0]})   - bias_eff;
      r_lsc = (r_work < 0) ? 8'd0 :
              (r_work > 11'sd255) ? 8'd255 : r_work[7:0];
      g_lsc = (g_work < 0) ? 8'd0 :
              (g_work > 11'sd255) ? 8'd255 : g_work[7:0];
      b_lsc = (b_work < 0) ? 8'd0 :
              (b_work > 11'sd255) ? 8'd255 : b_work[7:0];

      spatial_correct = {r_lsc,g_lsc,b_lsc};
    end
  endfunction

  assign lsc_O_tdata = {
      spatial_correct(awb_O_tdata[95:72], lsc_bias),
      spatial_correct(awb_O_tdata[71:48], lsc_bias),
      spatial_correct(awb_O_tdata[47:24], lsc_bias),
      spatial_correct(awb_O_tdata[23:0],  lsc_bias)
  };

  // No extra saturation gain. Keep the existing register stage so RGB and
  // SOF/EOL/valid retain their original pipeline alignment.
  assign sat_O_tdata = lsc_data_r;

  // Keep the normal camera branch registered.  The effect branch has an
  // additional line-memory stage; mode changes are applied only at a frame
  // boundary, so the two branches do not mix within a frame.
  reg          camera_O_tlast;
  reg          camera_O_tuser;
  reg  [95:0]  camera_O_tdata;
  reg          camera_O_tvalid;

  wire         edge_O_tlast;  //synthesis keep
  wire         edge_O_tuser;  //synthesis keep
  wire [95:0]  edge_O_tdata;  //synthesis keep
  wire [95:0]  gray_O_tdata;  //synthesis keep
  wire [95:0]  fill_O_tdata;  //synthesis keep
  wire         edge_O_tvalid; //synthesis keep
  wire         edge_O_tready; //synthesis keep

  wire         selected_tlast;
  wire         selected_tuser;
  wire [95:0]  selected_tdata;
  wire         selected_tvalid;

  // I_mode comes from the board switches.  Synchronize it into the camera
  // clock domain and only apply a new value at a frame boundary.
  reg [1:0] mode_meta;
  reg [1:0] mode_sync;
  reg [1:0] mode_latched;
  // Two switches keep their original four functions.  Re-entering 01 from
  // a frame-confirmed 00 cycles its processing submode without extra pins.
  localparam [1:0] EFFECT_EDGE = 2'd0;
  localparam [1:0] EFFECT_GRAY = 2'd1;
  localparam [1:0] EFFECT_FILL = 2'd2;
  reg [1:0] effect_latched;
  reg       effect_seen;
  // Only arm effect cycling after two frame-start samples of 00.  Ordinary
  // camera/OSD/circle mode changes retain the base project's frame timing.
  reg [1:0] camera_hold_frames;
  wire uart_frame_owner;
  wire [2:0] uart_frame_algo;
  uart_frame_command #(.SETTLE_FRAMES(2)) u_uart_frame_command (
      .clk(axi4s_video_aclk),.rst_n(I_rst_n),.frame_start(awb_O_tuser),
      .override_async(I_uart_override),.token_async(I_uart_token),.algo_async(I_uart_algo),
      .owner(uart_frame_owner),.algo(uart_frame_algo),.ack(O_uart_ack));
  assign O_uart_frame_start=awb_O_tuser;
  assign O_uart_report_algo=(mode_latched==0) ? 3'd0 :
      (mode_latched==2) ? 3'd4 : (mode_latched==3) ? 3'd5 : {1'b0,effect_latched}+3'd1;

  wire         awb_O_tuser_128;  //synthesis keep 
  wire         awb_O_tvalid_128;  //synthesis keep 
  wire [127:0] awb_O_tdata_128;  //synthesis keep 

  assign awb_O_tready = edge_O_tready;
  assign O_tdata  = awb_O_tdata_128;
  assign O_tlast  = selected_tlast ;
  assign O_tuser  = awb_O_tuser_128    ;
  assign O_tvalid = awb_O_tvalid_128   ;

  // Preserve the base project: 00/10/11 all write the normal RGB camera
  // stream, so the HDMI circle detector always analyses unmodified pixels.
  // Only 01 selects an effect.  Its three outputs share one-cycle timing.
  assign selected_tdata  = (mode_latched != 2'b01) ? camera_O_tdata :
                           (effect_latched == EFFECT_EDGE) ? edge_O_tdata :
                           (effect_latched == EFFECT_GRAY) ? gray_O_tdata :
                                                             fill_O_tdata;
  assign selected_tlast  = (mode_latched == 2'b01) ? edge_O_tlast  : camera_O_tlast;
  assign selected_tuser  = (mode_latched == 2'b01) ? edge_O_tuser  : camera_O_tuser;
  assign selected_tvalid = (mode_latched == 2'b01) ? edge_O_tvalid : camera_O_tvalid;

  always @(posedge axi4s_video_aclk or negedge I_rst_n) begin
    if (!I_rst_n) begin
      mode_meta    <= 2'b00;
      mode_sync    <= 2'b00;
      mode_latched <= 2'b00;
      camera_hold_frames <= 2'd0;
      effect_latched <= EFFECT_EDGE;
      effect_seen <= 1'b0;
    end
    else begin
      mode_meta <= I_mode;
      mode_sync <= mode_meta;

      if (awb_O_tuser) begin
        if (uart_frame_owner) begin
          case(uart_frame_algo)
            0: mode_latched<=2'b00;
            4: mode_latched<=2'b10;
            5: mode_latched<=2'b11;
            default: mode_latched<=2'b01;
          endcase
          if (uart_frame_algo>=1 && uart_frame_algo<=3)
            effect_latched<=uart_frame_algo-3'd1;
          camera_hold_frames<=0;
        end else begin
        mode_latched <= mode_sync;
        if (mode_sync == 2'b00) begin
          if (camera_hold_frames < 2'd2)
            camera_hold_frames <= camera_hold_frames + 2'd1;
        end
        else
          camera_hold_frames <= 2'd0;
        if ((mode_sync == 2'b01) && (mode_latched != 2'b01)) begin
          if (!effect_seen) begin
            effect_latched <= EFFECT_EDGE;
            effect_seen <= 1'b1;
          end
          else if ((mode_latched == 2'b00) &&
                   (camera_hold_frames == 2'd2)) begin
            if (effect_latched == EFFECT_FILL)
              effect_latched <= EFFECT_EDGE;
            else
              effect_latched <= effect_latched + 2'd1;
          end
        end
        end
      end
    end
  end

  always @(posedge axi4s_video_aclk or negedge I_rst_n) begin
    if (!I_rst_n) begin
      lsc_x_grp <= 11'd0;
      lsc_y_pos <= 11'd0;
      lsc_data_r <= 96'd0;
      lsc_tlast_r <= 1'b0;
      lsc_tuser_r <= 1'b0;
      lsc_tvalid_r <= 1'b0;
    end
    else begin
      lsc_tlast_r <= awb_O_tlast;
      lsc_tuser_r <= awb_O_tuser;
      lsc_tvalid_r <= awb_O_tvalid;
      if (awb_O_tvalid) begin
        // Register the corrected pixels before Sobel.  This keeps the colour
        // correction and edge detector in separate timing stages.
        lsc_data_r <= lsc_O_tdata;
        if (awb_O_tuser) begin
          // The current transfer is the first four pixels of the new frame.
          lsc_x_grp <= 11'd1;
          lsc_y_pos <= 11'd0;
        end
        else if (awb_O_tlast || (lsc_x_grp == 11'd319)) begin
          lsc_x_grp <= 11'd0;
          if (lsc_y_pos == 11'd719)
            lsc_y_pos <= 11'd0;
          else
            lsc_y_pos <= lsc_y_pos + 1'b1;
        end
        else begin
          lsc_x_grp <= lsc_x_grp + 1'b1;
        end
      end
      else begin
        lsc_data_r <= 96'd0;
      end
    end
  end

  always @(posedge axi4s_video_aclk or negedge I_rst_n) begin
    if (!I_rst_n) begin
      sat_data_r <= 96'd0;
      sat_tlast_r <= 1'b0;
      sat_tuser_r <= 1'b0;
      sat_tvalid_r <= 1'b0;
    end
    else begin
      sat_tlast_r <= lsc_tlast_r;
      sat_tuser_r <= lsc_tuser_r;
      sat_tvalid_r <= lsc_tvalid_r;
      if (lsc_tvalid_r)
        sat_data_r <= sat_O_tdata;
      else
        sat_data_r <= 96'd0;
    end
  end

  always @(posedge axi4s_video_aclk or negedge I_rst_n) begin
    if (!I_rst_n) begin
      camera_O_tlast  <= 1'b0;
      camera_O_tuser  <= 1'b0;
      camera_O_tdata  <= 96'd0;
      camera_O_tvalid <= 1'b0;
    end
    else begin
      camera_O_tlast  <= sat_tlast_r;
      camera_O_tuser  <= sat_tuser_r;
      camera_O_tvalid <= sat_tvalid_r;

      if (sat_tvalid_r)
        camera_O_tdata <= sat_data_r;
      else
        camera_O_tdata <= 96'd0;
    end
  end
  
  //assign m_aixs_tready = O_tready;
  //assign O_tdata  = m_aixs_tdata;
  //assign O_tlast  = m_aixs_tlast    ;
  //assign O_tuser  = m_aixs_tuser    ;
  //assign O_tvalid = m_aixs_tvalid   ;
 
 
demosaic #(
    .IMG_HEIGHT          (720 ), // 图像高度
    .IMG_WIDTH           (1280), // 图像宽度
    .data_complete_delay (50  ),
    .BAYER_MODE          ("BGGR")
)
u_demosaic
(
    .I_clk   			(axi4s_video_aclk)	,   // 时钟信号
    .I_rst_n 			(I_rst_n)	,   // 复位信号，低有效
    .axi4s_video_tdata 	(I_tdata)	,  // AXI4-Stream视频数据
    .axi4s_video_tdest 	(I_tdest)	,
    .axi4s_video_tlast 	(I_tlast)	,  // 行结束信号
    .axi4s_video_tvalid	(I_tvalid)	,  // 数据有效信号
    .axi4s_video_tuser 	(I_tuser)	,  // 帧开始信号
    .axi4s_video_tready	(I_tready)	,  // 从模块准备好接受数据
    .O_tlast  			(m_aixs_tlast)	,  // 输出行结束信号
    .O_tuser  			(m_aixs_tuser)	,  // 输出帧开始信号
    .O_tdata  			(m_aixs_tdata)	,  // 输出数据
    .O_tvalid 			(m_aixs_tvalid)	,  // 输出数据有效信号
    .O_tready   		(m_aixs_tready)	   // 输出数据准备好信号
);
 
data128_96 u_data128_96 (
    .I_tdata(m_aixs_tdata),
    .O_tdata(m_aixs_tdata_96)
);

wire [1:0] image_flags;
wire [19:0] awb_r_gain, awb_b_gain;
wire awb_enabled;
image_frame_settings u_image_settings (
    .clk(axi4s_video_aclk),.rst_n(I_rst_n),.frame_start(m_aixs_tuser),
    .token_async(I_image_token),.flags_async(I_image_flags),
    .flags(image_flags),.ack(O_image_ack));
isp_luma_stats u_luma_stats (
    .clk(axi4s_video_aclk),.rst_n(I_rst_n),.frame_start(m_aixs_tuser),
    .valid(m_aixs_tvalid),.rgb(m_aixs_tdata_96),
    .sample_valid(O_luma_sample),.mean_luma(O_mean_luma));
assign O_awb_report={awb_enabled,awb_r_gain,awb_b_gain};

awb #(
    .IMG_HEIGHT(720 ),
    .IMG_WIDTH (1280),
    .CONTROL_ENABLE(IMAGE_AUTO_ENABLE)
) u_awb (
    .I_clk   (axi4s_video_aclk),
    .I_rst_n (I_rst_n),
    .I_awb_enable(image_flags[0]),.O_awb_enabled(awb_enabled),
    .O_r_gain(awb_r_gain),.O_b_gain(awb_b_gain),
    .I_tlast (m_aixs_tlast),
    .I_tuser (m_aixs_tuser),
    .I_tdata (m_aixs_tdata_96),
    .I_tvalid(m_aixs_tvalid),
    .I_tready(m_aixs_tready),
    .O_tlast (awb_O_tlast ),
    .O_tuser (awb_O_tuser ),
    .O_tdata (awb_O_tdata ),
    .O_tvalid(awb_O_tvalid),
    .O_tready(awb_O_tready)
);

edge_detect_96 #(
    .IMG_WIDTH  (1280),
    .IMG_HEIGHT (720),
    .EDGE_TH     (11'd58)
) u_edge_detect_96 (
    .I_clk    (axi4s_video_aclk),
    .I_rst_n  (I_rst_n),
    .I_tlast  (sat_tlast_r),
    .I_tuser  (sat_tuser_r),
    .I_tdata  (sat_data_r),
    .I_tvalid (sat_tvalid_r),
    .I_tready (O_tready),
    .O_tlast  (edge_O_tlast),
    .O_tuser  (edge_O_tuser),
    .O_tdata  (edge_O_tdata),
    .O_gray_data(gray_O_tdata),
    .O_fill_data(fill_O_tdata),
    .O_tvalid (edge_O_tvalid),
    .O_tready (edge_O_tready)
);

    data_96bit_to_128bit u_data_96bit_to_128bit(
        .I_clk              ( axi4s_video_aclk  ),
        .I_rst_n            ( I_rst_n           ),
	
		.I_96b_frame_start  ( selected_tuser	),
		.I_96b_valid        ( selected_tvalid 	),
		.I_96b_data         ( selected_tdata		),
	
        .O_128b_frame_start ( awb_O_tuser_128  	),
        .O_128b_valid       ( awb_O_tvalid_128 	),
        .O_128b_data        ( awb_O_tdata_128  	)
    );



//data96_128 u_data96_128 (
//    .I_tdata(awb_O_tdata),
//    .O_tdata(awb_O_tdata_128)
//);
	
endmodule
