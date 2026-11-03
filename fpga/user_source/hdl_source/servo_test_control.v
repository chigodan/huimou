// Four-key servo calibration controller.
// Keys are active-low and are debounced with 1 ms samples.
module servo_test_control #(
    parameter integer CLK_FREQ_HZ = 100_000_000,
    parameter integer CENTER_US = 1500,
    parameter integer MIN_US = 500,
    parameter integer MAX_US = 2500
)(
    input  wire       I_clk,
    input  wire       I_rst_n,
    input  wire [3:0] I_key_n,
    output reg [11:0] O_pulse_us,
    output reg [6:0]  O_step_us,
    output wire      O_center_event
);

    localparam integer MS_CYCLES = CLK_FREQ_HZ / 1000;

    reg [16:0] ms_div_cnt;
    reg        ms_tick;
    reg [3:0]  key_meta;
    reg [3:0]  key_sync;
    reg [3:0]  key_state;
    reg [3:0]  key_state_d;
    reg [4:0]  debounce_cnt [0:3];
    reg [9:0]  hold_ms [0:1];
    integer i;

    wire [3:0] key_press = key_state_d & ~key_state;
    assign O_center_event = key_press[3];
    wire dec_repeat = (!key_state[0]) && (hold_ms[0] >= 10'd500) &&
                      (((hold_ms[0] - 10'd500) % 10'd100) == 10'd0);
    wire inc_repeat = (!key_state[1]) && (hold_ms[1] >= 10'd500) &&
                      (((hold_ms[1] - 10'd500) % 10'd100) == 10'd0);
    wire dec_event = key_press[0] | (ms_tick & dec_repeat);
    wire inc_event = key_press[1] | (ms_tick & inc_repeat);

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            ms_div_cnt <= 17'd0;
            ms_tick    <= 1'b0;
        end else if(ms_div_cnt == MS_CYCLES - 1) begin
            ms_div_cnt <= 17'd0;
            ms_tick    <= 1'b1;
        end else begin
            ms_div_cnt <= ms_div_cnt + 1'b1;
            ms_tick    <= 1'b0;
        end
    end

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            key_meta    <= 4'hf;
            key_sync    <= 4'hf;
            key_state   <= 4'hf;
            key_state_d <= 4'hf;
            for(i = 0; i < 4; i = i + 1)
                debounce_cnt[i] <= 5'd0;
        end else begin
            key_meta    <= I_key_n;
            key_sync    <= key_meta;
            key_state_d <= key_state;

            if(ms_tick) begin
                for(i = 0; i < 4; i = i + 1) begin
                    if(key_sync[i] == key_state[i]) begin
                        debounce_cnt[i] <= 5'd0;
                    end else if(debounce_cnt[i] == 5'd19) begin
                        key_state[i]    <= key_sync[i];
                        debounce_cnt[i] <= 5'd0;
                    end else begin
                        debounce_cnt[i] <= debounce_cnt[i] + 1'b1;
                    end
                end
            end
        end
    end

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            hold_ms[0] <= 10'd0;
            hold_ms[1] <= 10'd0;
        end else if(ms_tick) begin
            if(key_state[0])
                hold_ms[0] <= 10'd0;
            else if(hold_ms[0] < 10'd999)
                hold_ms[0] <= hold_ms[0] + 1'b1;
            else
                hold_ms[0] <= 10'd500;

            if(key_state[1])
                hold_ms[1] <= 10'd0;
            else if(hold_ms[1] < 10'd999)
                hold_ms[1] <= hold_ms[1] + 1'b1;
            else
                hold_ms[1] <= 10'd500;
        end
    end

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            O_pulse_us <= CENTER_US;
            O_step_us  <= 7'd10;
        end else begin
            // K4 has highest priority: return the position servo to its center.
            if(key_press[3]) begin
                O_pulse_us <= CENTER_US;
            end else if(dec_event && !inc_event) begin
                if(O_pulse_us <= (MIN_US + O_step_us))
                    O_pulse_us <= MIN_US;
                else
                    O_pulse_us <= O_pulse_us - O_step_us;
            end else if(inc_event && !dec_event) begin
                if(O_pulse_us >= (MAX_US - O_step_us))
                    O_pulse_us <= MAX_US;
                else
                    O_pulse_us <= O_pulse_us + O_step_us;
            end

            // K3 cycles the adjustment step: 1 -> 5 -> 10 -> 50 -> 1 us.
            if(key_press[2]) begin
                case(O_step_us)
                    7'd1:    O_step_us <= 7'd5;
                    7'd5:    O_step_us <= 7'd10;
                    7'd10:   O_step_us <= 7'd50;
                    default: O_step_us <= 7'd1;
                endcase
            end
        end
    end

endmodule

// Single-axis image-error feedback on a moving camera.
// Default POSITION mode (both pan and pitch in the active top-level design):
// pulse[k+1] = clamp(pulse[k] + limit(deadzone(cx - CENTER_X) / 2**GAIN_SHIFT)).
// A centered target holds the CURRENT command; it does not command 1500 us.
// One fresh observation may be consumed only once. Invalid/no-new observations
// hold position, including the detector's eight-frame display-box retention.
// Q8 pulse storage preserves sub-microsecond corrections without a division IP.
// SPEED_CONTROL=1 (continuous pan): error commands speed relative to CENTER_US.
// Hold the latest speed between frames, stop on deadband/invalid/K4/timeout.
// Acceleration is limited; stop requests bypass the ramp, reversal stops first.
// Kept in this already-listed source so the TD project source list is unchanged.
module servo_circle_track #(
    parameter integer CLK_FREQ_HZ = 100_000_000,
    parameter integer IMAGE_WIDTH = 1280,
    parameter integer CENTER_X = 640,
    parameter integer CONTROL_PERIOD_US = 20_000,
    parameter integer CENTER_US = 1500,
    parameter integer MIN_US = 1000,
    parameter integer MAX_US = 2000,
    parameter integer DEADBAND_PIXELS = 12,
    parameter integer GAIN_SHIFT = 5,
    parameter integer MAX_STEP_US = 8,
    parameter integer REVERSE = 0,
    parameter integer SPEED_CONTROL = 0,
    parameter integer MIN_DRIVE_US = 40,
    parameter integer SAMPLE_TIMEOUT_US = 100_000,
    // Optional continuous-pan input conditioning. Disabled for legacy and
    // position-mode instances; the continuous top enables it explicitly.
    parameter integer STABILITY_ENABLE = 0,
    parameter integer FILTER_SHIFT = 1,
    parameter integer RESTART_MARGIN_PIXELS = 12,
    parameter integer CONFIRM_SAMPLES = 2,
    parameter integer LOST_CONFIRM_SAMPLES = 2,
    parameter integer START_BOOST_US = 0,
    parameter integer START_BOOST_ERROR_PIXELS = 160
)(
    input wire I_pixel_clk,
    input wire I_pixel_rst_n,
    input wire I_sample,
    input wire I_fresh_valid,
    input wire [11:0] I_center_x,
    input wire I_clk,
    input wire I_rst_n,
    input wire I_center_event,
    output wire [11:0] O_pulse_us
);
    localparam integer CONTROL_CYCLES = (CLK_FREQ_HZ / 1_000_000) * CONTROL_PERIOD_US;
    localparam integer MIN_Q8 = MIN_US * 256;
    localparam integer MAX_Q8 = MAX_US * 256;
    localparam integer MAX_STEP_Q8 = MAX_STEP_US * 256;
    localparam integer CENTER_Q8 = CENTER_US * 256;
    localparam integer MIN_DRIVE_Q8 = MIN_DRIVE_US * 256;
    localparam integer START_BOOST_Q8 = START_BOOST_US * 256;
    localparam integer TIMEOUT_CYCLES = (CLK_FREQ_HZ / 1_000_000) * SAMPLE_TIMEOUT_US;
    // Size timers to their reachable ranges; sample_age includes the saturated
    // TIMEOUT_CYCLES value, while control_count wraps at CONTROL_CYCLES-1.
    localparam integer CONTROL_COUNT_BITS = (CONTROL_CYCLES > 1) ? $clog2(CONTROL_CYCLES) : 1;
    localparam integer SAMPLE_AGE_BITS = (TIMEOUT_CYCLES > 0) ? $clog2(TIMEOUT_CYCLES + 1) : 1;
    localparam integer STABLE_SPEED = (SPEED_CONTROL != 0) && (STABILITY_ENABLE != 0);

    // Bundled-data request/acknowledge mailbox. Payload is held unchanged until
    // the destination has captured it and its acknowledgement has returned.
    // Both clocks use the same board PLL-lock reset in the top-level design.
    reg [12:0] sample_payload;
    reg sample_req;
    reg sample_ack;
    (* ASYNC_REG = "TRUE" *) reg [1:0] ack_sync;
    (* ASYNC_REG = "TRUE" *) reg [1:0] req_sync;
    reg [12:0] received_payload;
    reg received_event;
    always @(posedge I_pixel_clk or negedge I_pixel_rst_n) begin
        if(!I_pixel_rst_n) begin
            sample_payload <= 13'd0;
            sample_req <= 1'b0;
            ack_sync <= 2'b00;
        end else begin
            ack_sync <= {ack_sync[0], sample_ack};
            if(I_sample && (sample_req == ack_sync[1])) begin
                sample_payload <= {I_fresh_valid, I_center_x};
                sample_req <= ~sample_req;
            end
        end
    end
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            req_sync <= 2'b00;
            sample_ack <= 1'b0;
            received_payload <= 13'd0;
            received_event <= 1'b0;
        end else begin
            req_sync <= {req_sync[0], sample_req};
            received_event <= 1'b0;
            if(req_sync[1] != sample_ack) begin
                received_payload <= sample_payload;
                received_event <= 1'b1;
                sample_ack <= req_sync[1];
            end
        end
    end

    reg [CONTROL_COUNT_BITS-1:0] control_count;
    reg [SAMPLE_AGE_BITS-1:0] sample_age;
    reg [11:0] latest_x;
    reg fresh_pending;
    reg signed [31:0] pulse_q8;
    reg signed [31:0] pixel_error;
    reg signed [31:0] active_error;
    reg signed [31:0] step_q8;
    reg signed [31:0] next_q8;
    reg signed [31:0] desired_q8;
    reg signed [31:0] speed_change_q8;
    wire control_tick = (control_count == CONTROL_CYCLES - 1);
    wire received_valid = received_payload[12] &&
                          (received_payload[11:0] < IMAGE_WIDTH);
    wire [11:0] stable_x;
    wire motion_enabled;
    wire sample_expired = !received_event && (sample_age >= TIMEOUT_CYCLES - 1);
    generate if(STABLE_SPEED != 0) begin : g_stability
        servo_speed_stabilizer #(.CENTER_X(CENTER_X),.DEADBAND_PIXELS(DEADBAND_PIXELS),
            .FILTER_SHIFT(FILTER_SHIFT),.RESTART_MARGIN_PIXELS(RESTART_MARGIN_PIXELS),
            .CONFIRM_SAMPLES(CONFIRM_SAMPLES),.LOST_CONFIRM_SAMPLES(LOST_CONFIRM_SAMPLES)) filter (
            .I_clk(I_clk),.I_rst_n(I_rst_n),.I_clear(I_center_event || sample_expired),
            .I_sample(received_event),.I_valid(received_valid),.I_x(received_payload[11:0]),
            .O_x(stable_x),.O_motion_enable(motion_enabled));
    end else begin : g_unfiltered
        assign stable_x=12'd0;
        assign motion_enabled=1'b1;
    end endgenerate
    // Conditioned coordinates are registered before speed arithmetic, breaking
    // the raw mailbox-event -> coordinate selection -> speed feedback path.
    wire usable_sample = STABLE_SPEED ? (motion_enabled && sample_age<TIMEOUT_CYCLES) :
                         received_event ? received_valid :
                         (fresh_pending && ((SPEED_CONTROL == 0) || (sample_age < TIMEOUT_CYCLES)));
    wire [11:0] usable_x = STABLE_SPEED ? stable_x :
                          received_event ? received_payload[11:0] : latest_x;
    wire [31:0] rounded_pulse = pulse_q8 + 32'd128;
    assign O_pulse_us = rounded_pulse[19:8];

    always @(*) begin
        pixel_error = $signed({1'b0, usable_x}) - CENTER_X;
        if(REVERSE != 0) pixel_error = -pixel_error;
        if(pixel_error > DEADBAND_PIXELS)
            active_error = pixel_error - DEADBAND_PIXELS;
        else if(pixel_error < -DEADBAND_PIXELS)
            active_error = pixel_error + DEADBAND_PIXELS;
        else
            active_error = 0;
        // Shift a positive magnitude, then restore its sign: symmetric rounding.
        if(active_error >= 0)
            step_q8 = (active_error * 256) >>> GAIN_SHIFT;
        else
            step_q8 = -(((-active_error) * 256) >>> GAIN_SHIFT);
        desired_q8 = CENTER_Q8;
        speed_change_q8 = 0;
        if(SPEED_CONTROL != 0) begin
            // Proportional SPEED command; do not integrate image error as angle.
            if(active_error > 0 && step_q8 < MIN_DRIVE_Q8)
                step_q8 = MIN_DRIVE_Q8;
            else if(active_error < 0 && step_q8 > -MIN_DRIVE_Q8)
                step_q8 = -MIN_DRIVE_Q8;
            desired_q8 = CENTER_Q8 + step_q8;
            if(desired_q8 > MAX_Q8) desired_q8 = MAX_Q8;
            else if(desired_q8 < MIN_Q8) desired_q8 = MIN_Q8;
            speed_change_q8 = desired_q8 - pulse_q8;
            if(speed_change_q8 > MAX_STEP_Q8) speed_change_q8 = MAX_STEP_Q8;
            else if(speed_change_q8 < -MAX_STEP_Q8) speed_change_q8 = -MAX_STEP_Q8;
            if(active_error == 0 ||
               ((pulse_q8 > CENTER_Q8) && (desired_q8 < CENTER_Q8)) ||
               ((pulse_q8 < CENTER_Q8) && (desired_q8 > CENTER_Q8)))
                next_q8 = CENTER_Q8;
            else if(STABLE_SPEED && START_BOOST_US>0 && pulse_q8==CENTER_Q8 &&
                    (pixel_error>=START_BOOST_ERROR_PIXELS || pixel_error<=-START_BOOST_ERROR_PIXELS)) begin
                // Confirmation has already accepted fresh observations. Cross
                // static friction promptly for a far target, without exceeding
                // either the proportional request or the configured PWM limits.
                if(active_error>0)
                    next_q8=(desired_q8>CENTER_Q8+START_BOOST_Q8) ? CENTER_Q8+START_BOOST_Q8 : desired_q8;
                else
                    next_q8=(desired_q8<CENTER_Q8-START_BOOST_Q8) ? CENTER_Q8-START_BOOST_Q8 : desired_q8;
            end
            else
                next_q8 = pulse_q8 + speed_change_q8;
        end else begin
            if(step_q8 > MAX_STEP_Q8) step_q8 = MAX_STEP_Q8;
            else if(step_q8 < -MAX_STEP_Q8) step_q8 = -MAX_STEP_Q8;
            next_q8 = pulse_q8 + step_q8;
        end
        if(next_q8 > MAX_Q8) next_q8 = MAX_Q8;
        else if(next_q8 < MIN_Q8) next_q8 = MIN_Q8;
    end

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            control_count <= 0;
            sample_age <= TIMEOUT_CYCLES;
            latest_x <= CENTER_X;
            fresh_pending <= 1'b0;
            pulse_q8 <= CENTER_US * 256;
        end else begin
            if(control_tick) control_count <= 0;
            else control_count <= control_count + 1'b1;
            if(sample_age < TIMEOUT_CYCLES) sample_age <= sample_age + 1'b1;

            if(received_event) begin
                latest_x <= received_payload[11:0];
                fresh_pending <= received_valid;
                if(received_valid) sample_age <= 0;
                else if(STABLE_SPEED == 0) sample_age <= TIMEOUT_CYCLES;
            end
            if(control_tick) begin
                if(SPEED_CONTROL == 0) fresh_pending <= 1'b0;
                if(usable_sample) pulse_q8 <= next_q8;
                else if(SPEED_CONTROL != 0) pulse_q8 <= CENTER_Q8;
            end
            if(SPEED_CONTROL != 0) begin
                // Do not wait for the next control tick to request a stop.
                if(STABLE_SPEED ? !motion_enabled :
                   (received_event && (!received_valid || active_error == 0)))
                    pulse_q8 <= CENTER_Q8;
                if(!received_event && sample_age >= TIMEOUT_CYCLES - 1) begin
                    pulse_q8 <= CENTER_Q8;
                    fresh_pending <= 1'b0;
                end
            end
            // K4: position axis homes; speed axis stops at its CURRENT angle.
            // Both resume only after a new camera observation.
            if(I_center_event) begin
                pulse_q8 <= CENTER_US * 256;
                fresh_pending <= 1'b0;
            end
        end
    end
endmodule

// Frame-rate input conditioner for continuous-speed pan. All filtering is
// shift/add arithmetic; no frame buffer, division IP or new clock is needed.
module servo_speed_stabilizer #(
    parameter integer CENTER_X=640,
    parameter integer DEADBAND_PIXELS=12,
    parameter integer FILTER_SHIFT=1,
    parameter integer RESTART_MARGIN_PIXELS=12,
    parameter integer CONFIRM_SAMPLES=2,
    parameter integer LOST_CONFIRM_SAMPLES=2
)(
    input wire I_clk,I_rst_n,I_clear,I_sample,I_valid,
    input wire [11:0] I_x,
    output reg [11:0] O_x,
    output wire O_motion_enable
);
    localparam integer CONFIRM_BITS=(CONFIRM_SAMPLES>1) ? $clog2(CONFIRM_SAMPLES+1) : 1;
    localparam integer LOST_BITS=(LOST_CONFIRM_SAMPLES>1) ? $clog2(LOST_CONFIRM_SAMPLES+1) : 1;
    localparam signed [12:0] CENTER_13=CENTER_X;
    reg [15:0] filtered_q4;
    reg have_filter;
    reg [1:0] run_direction,pending_direction; // 00 stop, 01 right, 10 left
    reg [CONFIRM_BITS-1:0] confirm_count;
    reg [LOST_BITS-1:0] lost_count;
    wire signed [16:0] filter_delta=$signed({1'b0,I_x,4'b0000})-$signed({1'b0,filtered_q4});
    // Shift magnitudes symmetrically so negative noise has no rounding bias.
    wire signed [16:0] filter_step=(filter_delta>=0) ? (filter_delta>>>FILTER_SHIFT) :
                                            -((-filter_delta)>>>FILTER_SHIFT);
    wire [15:0] filter_next=have_filter ? filtered_q4+filter_step : {I_x,4'b0000};
    wire [15:0] rounded_filter=filter_next+16'd8;
    wire [11:0] next_x=rounded_filter[15:4];
    wire signed [12:0] raw_error=$signed({1'b0,I_x})-CENTER_13;
    wire signed [12:0] filtered_error=$signed({1'b0,next_x})-CENTER_13;
    wire [1:0] raw_direction=(raw_error>DEADBAND_PIXELS) ? 2'd1 :
                             (raw_error < -DEADBAND_PIXELS) ? 2'd2 : 2'd0;
    wire filtered_centered=(filtered_error<=DEADBAND_PIXELS && filtered_error>=-DEADBAND_PIXELS);
    wire [1:0] start_direction=(raw_error>DEADBAND_PIXELS+RESTART_MARGIN_PIXELS &&
                               filtered_error>DEADBAND_PIXELS+RESTART_MARGIN_PIXELS) ? 2'd1 :
                              (raw_error < -(DEADBAND_PIXELS+RESTART_MARGIN_PIXELS) &&
                               filtered_error < -(DEADBAND_PIXELS+RESTART_MARGIN_PIXELS)) ? 2'd2 : 2'd0;
    assign O_motion_enable=(run_direction!=0);
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            filtered_q4<=CENTER_X*16;have_filter<=0;O_x<=CENTER_X;
            run_direction<=0;pending_direction<=0;confirm_count<=0;lost_count<=0;
        end else if(I_clear) begin
            filtered_q4<=CENTER_X*16;have_filter<=0;O_x<=CENTER_X;
            run_direction<=0;pending_direction<=0;confirm_count<=0;lost_count<=0;
        end else if(I_sample) begin
            if(!I_valid) begin
                // Keep a pending start across one missed observation, just as
                // ongoing motion is kept. Do not count the missed frame as a
                // confirmation. Repeated loss or the last-valid watchdog clears
                // it, so intermittent border detection cannot starve startup.
                if(LOST_CONFIRM_SAMPLES<=1 || lost_count>=LOST_CONFIRM_SAMPLES-1) begin
                    run_direction<=0;have_filter<=0;pending_direction<=0;confirm_count<=0;
                end else lost_count<=lost_count+1'b1;
            end else begin
                lost_count<=0;filtered_q4<=filter_next;have_filter<=1;O_x<=next_x;
                if(run_direction!=0) begin
                    // Stop on reaching centre or crossing sides; clear the
                    // filter so old coordinates cannot delay a later restart.
                    if(raw_direction==0 || filtered_centered || raw_direction!=run_direction) begin
                        run_direction<=0;pending_direction<=0;confirm_count<=0;have_filter<=0;
                    end
                end else if(start_direction==0) begin
                    pending_direction<=0;confirm_count<=0;
                end else if(pending_direction==start_direction) begin
                    if(confirm_count>=CONFIRM_SAMPLES-1) begin
                        run_direction<=start_direction;pending_direction<=0;confirm_count<=0;
                    end else confirm_count<=confirm_count+1'b1;
                end else begin
                    pending_direction<=start_direction;confirm_count<=1;
                    if(CONFIRM_SAMPLES<=1) begin run_direction<=start_direction;pending_direction<=0;confirm_count<=0; end
                end
            end
        end
    end
endmodule
