// 50 Hz hobby-servo PWM generator.
//
// I_pulse_us is the high-level pulse width in microseconds.
// Both fitted servos use position control: 500..2500 us nominally maps to
// 0..180 degrees, with 1500 us at the 90-degree/home position.
//
// The requested pulse width is clamped to the configured safe range and is
// latched only at the start of a 20 ms frame, so a control update cannot make
// a shortened or malformed pulse.
module servo_pwm #(
    parameter integer CLK_FREQ_HZ = 100_000_000,
    parameter integer FRAME_US    = 20_000,
    parameter integer MIN_PULSE_US = 500,
    parameter integer MAX_PULSE_US = 2_500,
    parameter integer START_PULSE_US = 1500
)(
    input  wire        I_clk,
    input  wire        I_rst_n,
    input  wire [11:0] I_pulse_us,
    output reg         O_pwm
);

    localparam integer CYCLES_PER_US = CLK_FREQ_HZ / 1_000_000;
    localparam integer US_DIV_BITS = (CYCLES_PER_US > 1) ? $clog2(CYCLES_PER_US) : 1;
    localparam integer FRAME_COUNT_BITS = (FRAME_US > 1) ? $clog2(FRAME_US) : 1;
    localparam [11:0] MIN_PULSE_US_12 = MIN_PULSE_US;
    localparam [11:0] MAX_PULSE_US_12 = MAX_PULSE_US;

    reg [US_DIV_BITS-1:0] us_div_cnt;
    reg [FRAME_COUNT_BITS-1:0] frame_us_cnt;
    reg [11:0] pulse_us_latched;

    wire [11:0] pulse_us_limited =
        (I_pulse_us < MIN_PULSE_US_12) ? MIN_PULSE_US_12 :
        (I_pulse_us > MAX_PULSE_US_12) ? MAX_PULSE_US_12 :
                                         I_pulse_us;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            us_div_cnt       <= 16'd0;
            frame_us_cnt     <= 16'd0;
            pulse_us_latched <= START_PULSE_US;
            O_pwm            <= 1'b0;
        end else if (us_div_cnt == CYCLES_PER_US - 1) begin
            us_div_cnt <= 16'd0;

            if (frame_us_cnt == FRAME_US - 1) begin
                frame_us_cnt     <= 16'd0;
                pulse_us_latched <= pulse_us_limited;
                O_pwm            <= 1'b1;
            end else begin
                frame_us_cnt <= frame_us_cnt + 1'b1;
                O_pwm        <= ((frame_us_cnt + 1'b1) < pulse_us_latched);
            end
        end else begin
            us_div_cnt <= us_div_cnt + 1'b1;
        end
    end

endmodule
