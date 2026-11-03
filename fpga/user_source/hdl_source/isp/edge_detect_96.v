module edge_detect_96 #(
    parameter IMG_WIDTH  = 1280,
    parameter IMG_HEIGHT = 720,
    parameter EDGE_TH    = 11'd58,
    parameter [7:0] FILL_COLOR_TH = 8'd24
)(
    input               I_clk,
    input               I_rst_n,
    input               I_tlast,
    input               I_tuser,
    input      [95:0]   I_tdata,
    input               I_tvalid,
    input               I_tready,
    output reg          O_tlast,
    output reg          O_tuser,
    output reg [95:0]   O_tdata,
    output reg [95:0]   O_gray_data,
    output reg [95:0]   O_fill_data,
    output reg          O_tvalid,
    output              O_tready
);

assign O_tready = I_tready;

wire [23:0] pix0 = I_tdata[95:72];
wire [23:0] pix1 = I_tdata[71:48];
wire [23:0] pix2 = I_tdata[47:24];
wire [23:0] pix3 = I_tdata[23:0];

// Low-cost luminance approximation: R/4 + G/2 + B/4.
wire [7:0] gray0 = {2'b0,pix0[23:18]} + {1'b0,pix0[15:9]} + {2'b0,pix0[7:2]};
wire [7:0] gray1 = {2'b0,pix1[23:18]} + {1'b0,pix1[15:9]} + {2'b0,pix1[7:2]};
wire [7:0] gray2 = {2'b0,pix2[23:18]} + {1'b0,pix2[15:9]} + {2'b0,pix2[7:2]};
wire [7:0] gray3 = {2'b0,pix3[23:18]} + {1'b0,pix3[15:9]} + {2'b0,pix3[7:2]};

wire [23:0] gray_pix0 = {gray0,gray0,gray0};
wire [23:0] gray_pix1 = {gray1,gray1,gray1};
wire [23:0] gray_pix2 = {gray2,gray2,gray2};
wire [23:0] gray_pix3 = {gray3,gray3,gray3};

// Horizontal 1-2-1 smoothing; the vertical pass uses the row history below.
reg [7:0] gray_prev2, gray_prev1;
wire [9:0] smooth_sum0 = {2'b0,gray_prev2} +
                          ({2'b0,gray_prev1} << 1) + {2'b0,gray0};
wire [9:0] smooth_sum1 = {2'b0,gray_prev1} +
                          ({2'b0,gray0} << 1) + {2'b0,gray1};
wire [9:0] smooth_sum2 = {2'b0,gray0} +
                          ({2'b0,gray1} << 1) + {2'b0,gray2};
wire [9:0] smooth_sum3 = {2'b0,gray1} +
                          ({2'b0,gray2} << 1) + {2'b0,gray3};
wire [7:0] smooth0 = smooth_sum0[9:2];
wire [7:0] smooth1 = smooth_sum1[9:2];
wire [7:0] smooth2 = smooth_sum2[9:2];
wire [7:0] smooth3 = smooth_sum3[9:2];
wire [31:0] smooth_word = {smooth0,smooth1,smooth2,smooth3};

localparam IMG_WIDTH_GRP = (IMG_WIDTH >> 2);

reg [10:0] x_grp;
reg [10:0] y_pos;
localparam ROW_ADDR_BITS = (IMG_WIDTH_GRP > 1) ? $clog2(IMG_WIDTH_GRP) : 1;
wire [ROW_ADDR_BITS-1:0] row_addr = I_tuser ? 0 : x_grp;
wire [1:0] row_phase = I_tuser ? 2'd0 : y_pos[1:0];
wire [31:0] line_r [0:3];
wire [31:0] line0_r = line_r[0], line1_r = line_r[1];
wire [31:0] line2_r = line_r[2], line3_r = line_r[3];
reg [31:0] current_word_r;
// Explicit native BRAM: inference otherwise falls back to LUT RAM. NOREG
// still has a synchronous read; READBEFOREWRITE retains the fourth-old row.
genvar row_bank;
generate for (row_bank = 0; row_bank < 4; row_bank = row_bank + 1) begin : rows
    PH1P_LOGIC_ERAM #(
        .DATA_WIDTH_A(32), .DATA_WIDTH_B(32),
        .ADDR_WIDTH_A(ROW_ADDR_BITS), .ADDR_WIDTH_B(ROW_ADDR_BITS),
        .DATA_DEPTH_A(2**ROW_ADDR_BITS), .DATA_DEPTH_B(2**ROW_ADDR_BITS),
        .MODE("DP"), .REGMODE_A("NOREG"), .REGMODE_B("NOREG"),
        .WRITEMODE_A("READBEFOREWRITE"), .CLKMODE("SYNC"),
        .IMPLEMENT("20K(FAST)")
    ) ram (
        .clka(I_clk), .cea(I_rst_n && I_tvalid), .ocea(1'b0),
        .wea(row_phase == row_bank), .addra(row_addr),
        .dia(smooth_word), .doa(line_r[row_bank]), .rsta(1'b0), .bea(1'b0),
        .clkb(I_clk), .ceb(1'b0), .oceb(1'b0), .web(1'b0),
        .addrb({ROW_ADDR_BITS{1'b0}}), .dib(32'd0), .dob(),
        .rstb(1'b0), .beb(1'b0),
        .ecc_sbiterrinj(1'b0), .ecc_dbiterrinj(1'b0),
        .ecc_sbiterr(), .ecc_dbiterr()
    );
end endgenerate
reg [1:0]    row_phase_r;
reg [95:0]   gray_stage_r;
reg          stage_tlast;
reg          stage_tuser;
reg          stage_valid;
reg          stage_window_valid;

wire [31:0] row4_word = (row_phase_r == 0) ? line0_r :
                       (row_phase_r == 1) ? line1_r :
                       (row_phase_r == 2) ? line2_r : line3_r;
wire [31:0] row3_word = (row_phase_r == 0) ? line1_r :
                       (row_phase_r == 1) ? line2_r :
                       (row_phase_r == 2) ? line3_r : line0_r;
wire [31:0] row2_word = (row_phase_r == 0) ? line2_r :
                       (row_phase_r == 1) ? line3_r :
                       (row_phase_r == 2) ? line0_r : line1_r;
wire [31:0] row1_word = (row_phase_r == 0) ? line3_r :
                       (row_phase_r == 1) ? line0_r :
                       (row_phase_r == 2) ? line1_r : line2_r;

function [31:0] vertical_smooth;
    input [31:0] upper, middle, lower;
    reg [9:0] sum;
    integer lane;
    begin
        for (lane = 0; lane < 4; lane = lane + 1) begin
            sum = {2'b0, upper[lane*8 +: 8]} +
                  {1'b0, middle[lane*8 +: 8], 1'b0} +
                  {2'b0, lower[lane*8 +: 8]};
            vertical_smooth[lane*8 +: 8] = sum[9:2];
        end
    end
endfunction

wire [31:0] top_word = vertical_smooth(row4_word, row3_word, row2_word);
wire [31:0] mid_word = vertical_smooth(row3_word, row2_word, row1_word);
wire [31:0] bot_word = vertical_smooth(row2_word, row1_word, current_word_r);

wire [7:0] top0 = top_word[31:24];
wire [7:0] top1 = top_word[23:16];
wire [7:0] top2 = top_word[15:8];
wire [7:0] top3 = top_word[7:0];
wire [7:0] mid0 = mid_word[31:24];
wire [7:0] mid1 = mid_word[23:16];
wire [7:0] mid2 = mid_word[15:8];
wire [7:0] mid3 = mid_word[7:0];
wire [7:0] bot0 = bot_word[31:24];
wire [7:0] bot1 = bot_word[23:16];
wire [7:0] bot2 = bot_word[15:8];
wire [7:0] bot3 = bot_word[7:0];

reg [7:0] top_prev2, top_prev1;
reg [7:0] mid_prev2, mid_prev1;
reg [7:0] bot_prev2, bot_prev1;

// |Gx| and |Gy| are at most 1020; their L1 magnitude fits in 11 bits.
function [10:0] sobel_mag;
    input [7:0] a00, a01, a02;
    input [7:0] a10, a11, a12;
    input [7:0] a20, a21, a22;
    reg signed [10:0] left_sum;
    reg signed [10:0] right_sum;
    reg signed [10:0] upper_sum;
    reg signed [10:0] lower_sum;
    reg signed [10:0] gx_f;
    reg signed [10:0] gy_f;
    reg [10:0] abs_gx_f;
    reg [10:0] abs_gy_f;
    begin
        left_sum  = {3'b000,a00} + {2'b00,a10,1'b0} + {3'b000,a20};
        right_sum = {3'b000,a02} + {2'b00,a12,1'b0} + {3'b000,a22};
        upper_sum = {3'b000,a00} + {2'b00,a01,1'b0} + {3'b000,a02};
        lower_sum = {3'b000,a20} + {2'b00,a21,1'b0} + {3'b000,a22};
        gx_f = right_sum - left_sum;
        gy_f = upper_sum - lower_sum;
        abs_gx_f = gx_f[10] ? (~gx_f + 1'b1) : gx_f;
        abs_gy_f = gy_f[10] ? (~gy_f + 1'b1) : gy_f;
        sobel_mag = abs_gx_f + abs_gy_f;
    end
endfunction

wire [10:0] mag0 = sobel_mag(top_prev2,top_prev1,top0,
                             mid_prev2,mid_prev1,mid0,
                             bot_prev2,bot_prev1,bot0);
wire [10:0] mag1 = sobel_mag(top_prev1,top0,top1,
                             mid_prev1,mid0,mid1,
                             bot_prev1,bot0,bot1);
wire [10:0] mag2 = sobel_mag(top0,top1,top2,
                             mid0,mid1,mid2,
                             bot0,bot1,bot2);
wire [10:0] mag3 = sobel_mag(top1,top2,top3,
                             mid1,mid2,mid3,
                             bot1,bot2,bot3);

// A single threshold is deterministic and direction-independent. The former
// causal weak-edge linking depended on scan direction and four-pixel packing.
wire edge0 = stage_window_valid && (mag0 >= EDGE_TH);
wire edge1 = stage_window_valid && (mag1 >= EDGE_TH);
wire edge2 = stage_window_valid && (mag2 >= EDGE_TH);
wire edge3 = stage_window_valid && (mag3 >= EDGE_TH);

wire [23:0] out0 = edge0 ? 24'hffffff : 24'h000000;
wire [23:0] out1 = edge1 ? 24'hffffff : 24'h000000;
wire [23:0] out2 = edge2 ? 24'hffffff : 24'h000000;
wire [23:0] out3 = edge3 ? 24'hffffff : 24'h000000;

// Reuse the seed above or to the left when RGB is similar. Keeping the raw
// seed avoids row-to-row quantization jumps and gradual colour drift.
function [23:0] region_colour;
    input [23:0] pixel;
    begin
        region_colour = {
            pixel[23:19], pixel[23:21],
            pixel[15:11], pixel[15:13],
            pixel[7:3],   pixel[7:5]
        };
    end
endfunction

function colour_close;
    input [23:0] pixel, seed;
    reg [8:0] dr, dg, db;
    begin
        // |pixel-seed| <= T, using one biased difference per channel.
        // Nine bits prevent opposite colours from wrapping into the window.
        dr = {1'b0,pixel[23:16]} + {1'b0,FILL_COLOR_TH} - {1'b0,seed[23:16]};
        dg = {1'b0,pixel[15:8]}  + {1'b0,FILL_COLOR_TH} - {1'b0,seed[15:8]};
        db = {1'b0,pixel[7:0]}   + {1'b0,FILL_COLOR_TH} - {1'b0,seed[7:0]};
        colour_close = (dr <= {FILL_COLOR_TH,1'b0}) &&
                       (dg <= {FILL_COLOR_TH,1'b0}) &&
                       (db <= {FILL_COLOR_TH,1'b0});
    end
endfunction

// Maximum RGB channel error; shared by eligibility and candidate selection.
function [7:0] colour_distance;
    input [23:0] pixel, seed;
    reg [8:0] delta;
    reg [7:0] magnitude, worst;
    integer channel;
    begin
        worst = 0;
        for (channel = 0; channel < 3; channel = channel + 1) begin
            delta = {1'b0,pixel[channel*8 +: 8]} -
                    {1'b0,seed[channel*8 +: 8]};
            magnitude = delta[8] ? -delta : delta;
            if (magnitude > worst) worst = magnitude;
        end
        colour_distance = worst;
    end
endfunction

function [23:0] fill_average2;
    input [23:0] a, b;
    reg [8:0] r, g, blue;
    begin
        r = {1'b0,a[23:16]} + {1'b0,b[23:16]};
        g = {1'b0,a[15:8]}  + {1'b0,b[15:8]};
        blue = {1'b0,a[7:0]} + {1'b0,b[7:0]};
        fill_average2 = {r[8:1],g[8:1],blue[8:1]};
    end
endfunction

reg [23:0] fill_seed, fill_seed_next;
reg fill_active, fill_active_next;
reg stage_fill_first;
reg stage_fill_row_valid;
wire [95:0] fill_row_rgb;
reg [47:0] fill_rgb_prev;
// The causal Gaussian/Sobel centre is (x-2,y-2), not the input pixel.
// Two alternating RGB rows plus two pixels align only the fill branch.
PH1P_LOGIC_ERAM #(
    .DATA_WIDTH_A(96), .DATA_WIDTH_B(96),
    .ADDR_WIDTH_A(ROW_ADDR_BITS+1), .ADDR_WIDTH_B(ROW_ADDR_BITS+1),
    .DATA_DEPTH_A(2**(ROW_ADDR_BITS+1)), .DATA_DEPTH_B(2**(ROW_ADDR_BITS+1)),
    .MODE("DP"), .REGMODE_A("NOREG"), .REGMODE_B("NOREG"),
    .WRITEMODE_A("READBEFOREWRITE"), .CLKMODE("SYNC"),
    .IMPLEMENT("20K(FAST)")
) fill_rgb_ram (
    .clka(I_clk), .cea(I_rst_n && I_tvalid), .ocea(1'b0), .wea(1'b1),
    .addra({I_tuser ? 1'b0 : y_pos[0],row_addr}),
    .dia(I_tdata), .doa(fill_row_rgb), .rsta(1'b0), .bea(1'b0),
    .clkb(I_clk), .ceb(1'b0), .oceb(1'b0), .web(1'b0),
    .addrb({(ROW_ADDR_BITS+1){1'b0}}), .dib(96'd0), .dob(),
    .rstb(1'b0), .beb(1'b0),
    .ecc_sbiterrinj(1'b0), .ecc_dbiterrinj(1'b0),
    .ecc_sbiterr(), .ecc_dbiterr()
);
wire [95:0] fill_rgb_word = {stage_fill_first ? 48'd0 : fill_rgb_prev,
                            fill_row_rgb[95:48]};
// Pad the unavailable top two rows/left two pixels; never cache stale RAM.
wire [3:0] fill_valid_mask = !stage_fill_row_valid ? 4'b0000 :
                           stage_fill_first ? 4'b0011 : 4'b1111;
// One unreset row of {valid, raw RGB seed} per pixel, read in the RAM stage.
// The first row of each frame ignores old contents; contours store valid=0.
wire [99:0] fill_above_word;
reg [99:0] fill_store_word;
reg [10:0] stage_fill_addr;
reg stage_fill_above;
reg [95:0] fill_word;
reg [23:0] fill_pixel;
reg [23:0] fill_above_seed;
reg [7:0] fill_left_distance, fill_above_distance;
reg fill_left_match, fill_above_match;
wire [3:0] fill_edges = {edge0,edge1,edge2,edge3};
// Lightweight wxr local averaging, only when starting a new seed. Pairs
// never cross a contour/colour boundary; raw pixels still gate inheritance.
wire fill_pair01 = !(|fill_edges[3:2]) &&
                   colour_close(fill_rgb_word[95:72],fill_rgb_word[71:48]);
wire fill_pair23 = !(|fill_edges[1:0]) &&
                   colour_close(fill_rgb_word[47:24],fill_rgb_word[23:0]);
wire [23:0] fill_mean01 = fill_average2(fill_rgb_word[95:72],fill_rgb_word[71:48]);
wire [23:0] fill_mean23 = fill_average2(fill_rgb_word[47:24],fill_rgb_word[23:0]);
wire [95:0] fill_local_word = {
    fill_pair01 ? {2{fill_mean01}} : fill_rgb_word[95:48],
    fill_pair23 ? {2{fill_mean23}} : fill_rgb_word[47:0]
};

// Read this word while writing the preceding pipeline word. Keep all 24 seed
// bits, so moving storage to BRAM cannot introduce colour bands or drift.
PH1P_LOGIC_ERAM #(
    .DATA_WIDTH_A(100), .DATA_WIDTH_B(100),
    .ADDR_WIDTH_A(ROW_ADDR_BITS), .ADDR_WIDTH_B(ROW_ADDR_BITS),
    .DATA_DEPTH_A(2**ROW_ADDR_BITS), .DATA_DEPTH_B(2**ROW_ADDR_BITS),
    .MODE("DP"), .REGMODE_A("NOREG"), .REGMODE_B("NOREG"),
    .WRITEMODE_B("READBEFOREWRITE"), .CLKMODE("SYNC"),
    .IMPLEMENT("20K(FAST)")
) fill_row_ram (
    .clka(I_clk), .cea(I_rst_n && I_tvalid), .ocea(1'b0), .wea(1'b0),
    .addra(row_addr), .dia(100'd0), .doa(fill_above_word),
    .rsta(1'b0), .bea(1'b0),
    .clkb(I_clk), .ceb(I_rst_n && stage_valid), .oceb(1'b0), .web(1'b1),
    .addrb(stage_fill_addr[ROW_ADDR_BITS-1:0]), .dib(fill_store_word), .dob(),
    .rstb(1'b0), .beb(1'b0),
    .ecc_sbiterrinj(1'b0), .ecc_dbiterrinj(1'b0),
    .ecc_sbiterr(), .ecc_dbiterr()
);

integer fill_lane;
always @* begin
    fill_seed_next = fill_seed;
    fill_active_next = fill_active && !stage_fill_first;
    fill_word = 96'd0;
    fill_store_word = 100'd0;
    fill_pixel = 24'd0;
    fill_above_seed = 24'd0;
    fill_left_distance = 0; fill_above_distance = 0;
    fill_left_match = 0; fill_above_match = 0;
    // Unrolled four-pixel scan, continuous across packed-word boundaries.
    for (fill_lane = 0; fill_lane < 4; fill_lane = fill_lane + 1) begin
        fill_pixel = fill_rgb_word[95 - fill_lane*24 -: 24];
        fill_above_seed = fill_above_word[98 - fill_lane*25 -: 24];
        fill_left_distance = colour_distance(fill_pixel,fill_seed_next);
        fill_above_distance = colour_distance(fill_pixel,fill_above_seed);
        fill_left_match = fill_active_next && (fill_left_distance <= FILL_COLOR_TH);
        fill_above_match = stage_fill_above && fill_above_word[99 - fill_lane*25] &&
                           (fill_above_distance <= FILL_COLOR_TH);
        if (!fill_valid_mask[3 - fill_lane] || fill_edges[3 - fill_lane]) begin
            fill_active_next = 1'b0;
        end else begin
            // Equal/near-equal candidates keep the carried seed. Switch only
            // when above is at least four RGB levels closer (no column bias).
            if (fill_above_match && (!fill_left_match ||
                ({1'b0,fill_above_distance} + 9'd4 <= {1'b0,fill_left_distance})))
                fill_seed_next = fill_above_seed;
            else if (!fill_left_match)
                fill_seed_next = fill_local_word[95 - fill_lane*24 -: 24];
            fill_active_next = 1'b1;
            fill_word[95 - fill_lane*24 -: 24] = region_colour(fill_seed_next);
            fill_store_word[99 - fill_lane*25 -: 25] = {1'b1,fill_seed_next};
        end
    end
end
always @(posedge I_clk or negedge I_rst_n) begin
    if(!I_rst_n) begin
        O_tlast  <= 1'b0;
        O_tuser  <= 1'b0;
        O_tdata  <= 96'd0;
        O_gray_data <= 96'd0;
        O_fill_data <= 96'd0;
        O_tvalid <= 1'b0;
        x_grp    <= 11'd0;
        y_pos    <= 11'd0;
        current_word_r <= 32'd0;
        row_phase_r <= 2'd0;
        gray_stage_r <= 96'd0;
        stage_tlast <= 1'b0;
        stage_tuser <= 1'b0;
        stage_valid <= 1'b0;
        stage_window_valid <= 1'b0;
        top_prev2 <= 8'd0; top_prev1 <= 8'd0;
        mid_prev2 <= 8'd0; mid_prev1 <= 8'd0;
        bot_prev2 <= 8'd0; bot_prev1 <= 8'd0;
        gray_prev2 <= 8'd0; gray_prev1 <= 8'd0;
        fill_seed <= 24'd0;
        fill_active <= 1'b0;
        fill_rgb_prev <= 48'd0;
        stage_fill_row_valid <= 1'b0;
        stage_fill_first <= 1'b0;
        stage_fill_addr <= 11'd0;
        stage_fill_above <= 1'b0;
    end else begin
        // Output stage: RAM output, Sobel and rendering share this timing.
        O_tlast  <= stage_tlast;
        O_tuser  <= stage_tuser;
        O_tvalid <= stage_valid;
        if(stage_valid) begin
            O_tdata     <= {out0,out1,out2,out3};
            O_gray_data <= gray_stage_r;
            O_fill_data <= fill_word;
            fill_seed <= fill_seed_next;
            fill_active <= !stage_tlast && fill_active_next;
            fill_rgb_prev <= stage_tlast ? 48'd0 : fill_row_rgb[47:0];

            top_prev2 <= top2; top_prev1 <= top3;
            mid_prev2 <= mid2; mid_prev1 <= mid3;
            bot_prev2 <= bot2; bot_prev1 <= bot3;
            if(stage_tlast) begin
                top_prev2 <= 8'd0; top_prev1 <= 8'd0;
                mid_prev2 <= 8'd0; mid_prev1 <= 8'd0;
                bot_prev2 <= 8'd0; bot_prev1 <= 8'd0;
            end
        end else begin
            O_tdata <= 96'd0;
            O_gray_data <= 96'd0;
            O_fill_data <= 96'd0;
        end

        // Read-before-write keeps the row from four lines earlier while
        // the same bank is overwritten by the current row.
        stage_tlast <= I_tlast;
        stage_tuser <= I_tuser;
        stage_valid <= I_tvalid;
        stage_fill_first <= I_tuser || (x_grp == 11'd0);
        if(I_tvalid) begin
            stage_fill_addr <= I_tuser ? 11'd0 : x_grp;
            stage_fill_row_valid <= !I_tuser && (y_pos >= 11'd2);
            stage_fill_above <= !I_tuser && (y_pos > 11'd2);
            current_word_r <= smooth_word;
            row_phase_r <= I_tuser ? 2'd0 : y_pos[1:0];
            gray_stage_r <= {gray_pix0,gray_pix1,gray_pix2,gray_pix3};
            stage_window_valid <= (y_pos >= 11'd4) &&
                                  (x_grp != 11'd0) && !I_tuser;

            gray_prev2 <= gray2;
            gray_prev1 <= gray3;

            if(I_tuser) begin
                x_grp <= 11'd1;
                y_pos <= 11'd0;
            end else if(I_tlast || (x_grp == IMG_WIDTH_GRP - 1)) begin
                x_grp <= 11'd0;
                if(y_pos == IMG_HEIGHT - 1)
                    y_pos <= 11'd0;
                else
                    y_pos <= y_pos + 1'b1;
                gray_prev2 <= 8'd0;
                gray_prev1 <= 8'd0;
            end else begin
                x_grp <= x_grp + 1'b1;
            end
        end else begin
            stage_window_valid <= 1'b0;
            gray_stage_r <= 96'd0;
        end
    end
end

endmodule
