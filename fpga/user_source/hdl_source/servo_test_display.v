// Standalone horizontal continuous-servo pulse calibration page.
// OFFSET is pulse distance from calibrated STOP, not measured angle or RPM.
// K1/K2 change speed pulse; K4 requests STOP and cannot home the shaft.
module servo_test_display #(
    parameter integer CENTER_US = 1500
)(
    input  wire        I_clk,
    input  wire        I_rst_n,
    input  wire        I_vsync,
    input  wire        I_hsync,
    input  wire        I_de,
    input  wire        I_user,
    input  wire        I_last,
    input  wire [11:0] I_pulse_us,
    input  wire [6:0]  I_step_us,
    output reg         O_vsync,
    output reg         O_hsync,
    output reg         O_de,
    output reg [23:0]  O_data
);

    localparam [11:0] TEXT_X = 12'd120;
    localparam [11:0] TEXT_Y = 12'd75;
    localparam integer CELL_W = 8;
    localparam integer CELL_H = 16;

    reg [11:0] x;
    reg [11:0] y;
    reg [11:0] pulse_meta;
    reg [11:0] pulse_sync;
    reg [6:0]  step_meta;
    reg [6:0]  step_sync;
    reg [7:0]  char_code;
    reg [3:0]  char_row;
    wire [15:0] char_bits;
    reg [3:0]  char_col;
    reg        char_sample;
    wire char_on=char_sample && char_bits[4'd15-char_col];
    reg [4:0]  text_col;
    reg [3:0]  text_row;
    reg [5:0]  local_x;
    reg [5:0]  local_y;

    wire [13:0] duty_milli = pulse_sync * 4'd5;
    wire [11:0] speed_offset_us = (pulse_sync >= CENTER_US) ?
                                  pulse_sync - CENTER_US : CENTER_US - pulse_sync;

    wire [3:0] p3 = pulse_sync / 1000;
    wire [3:0] p2 = (pulse_sync / 100) % 10;
    wire [3:0] p1 = (pulse_sync / 10) % 10;
    wire [3:0] p0 = pulse_sync % 10;
    wire [3:0] d4 = duty_milli / 10000;
    wire [3:0] d3 = (duty_milli / 1000) % 10;
    wire [3:0] d2 = (duty_milli / 100) % 10;
    wire [3:0] d1 = (duty_milli / 10) % 10;
    wire [3:0] d0 = duty_milli % 10;
    wire [3:0] s1 = (step_sync / 10) % 10;
    wire [3:0] s0 = step_sync % 10;
    wire [3:0] a3 = speed_offset_us / 1000;
    wire [3:0] a2 = (speed_offset_us / 100) % 10;
    wire [3:0] a1 = (speed_offset_us / 10) % 10;
    wire [3:0] a0 = speed_offset_us % 10;

    osd_char_lib u_servo_test_font (
        .I_char     ( char_code ),
        .I_row      ( char_row  ),
        .O_row_bits ( char_bits )
    );

    function [7:0] digit;
        input [3:0] value;
        begin digit = 8'h30 + value; end
    endfunction

    // Eight text rows. Unsupported positions return space (blank glyph).
    always @(*) begin
        char_code = 8'h20;
        case(text_row)
            4'd0: begin // SERVO CONT TEST
                case(text_col)
                    0:char_code="S"; 1:char_code="E"; 2:char_code="R";
                    3:char_code="V"; 4:char_code="O"; 6:char_code="C";
                    7:char_code="O"; 8:char_code="N"; 9:char_code="T";
                    11:char_code="T"; 12:char_code="E"; 13:char_code="S";
                    14:char_code="T";
                    default:char_code=8'h20;
                endcase
            end
            4'd1: begin // PULSE: 1500 US
                case(text_col)
                    0:char_code="P"; 1:char_code="U"; 2:char_code="L";
                    3:char_code="S"; 4:char_code="E"; 5:char_code=":";
                    7:char_code=digit(p3); 8:char_code=digit(p2);
                    9:char_code=digit(p1); 10:char_code=digit(p0);
                    12:char_code="U"; 13:char_code="S";
                    default:char_code=8'h20;
                endcase
            end
            4'd2: begin // OFFSET: 0000 US (direction visible in PULSE vs STOP)
                case(text_col)
                    0:char_code="O"; 1:char_code="F"; 2:char_code="F";
                    3:char_code="S"; 4:char_code="E"; 5:char_code="T";
                    6:char_code=":"; 8:char_code=digit(a3);
                    9:char_code=digit(a2); 10:char_code=digit(a1);
                    11:char_code=digit(a0); 13:char_code="U"; 14:char_code="S";
                    default:char_code=8'h20;
                endcase
            end
            4'd3: begin // DUTY: 07.500 %
                case(text_col)
                    0:char_code="D"; 1:char_code="U"; 2:char_code="T";
                    3:char_code="Y"; 4:char_code=":";
                    6:char_code=digit(d4); 7:char_code=digit(d3);
                    8:char_code=8'h2e; 9:char_code=digit(d2);
                    10:char_code=digit(d1); 11:char_code=digit(d0);
                    13:char_code=8'h25;
                    default:char_code=8'h20;
                endcase
            end
            4'd4: begin // STEP: 10 US
                case(text_col)
                    0:char_code="S"; 1:char_code="T"; 2:char_code="E";
                    3:char_code="P"; 4:char_code=":";
                    6:char_code=(step_sync >= 10) ? digit(s1) : 8'h20;
                    7:char_code=digit(s0); 9:char_code="U"; 10:char_code="S";
                    default:char_code=8'h20;
                endcase
            end
            4'd5: begin // K1 DEC   K2 INC
                case(text_col)
                    0:char_code="K"; 1:char_code="1"; 3:char_code="D";
                    4:char_code="E"; 5:char_code="C"; 9:char_code="K";
                    10:char_code="2"; 12:char_code="I"; 13:char_code="N";
                    14:char_code="C"; default:char_code=8'h20;
                endcase
            end
            4'd6: begin // K3 STEP  K4 STOP
                case(text_col)
                    0:char_code="K"; 1:char_code="3"; 3:char_code="S";
                    4:char_code="T"; 5:char_code="E"; 6:char_code="P";
                    9:char_code="K"; 10:char_code="4"; 12:char_code="S";
                    13:char_code="T"; 14:char_code="O"; 15:char_code="P";
                    default:char_code=8'h20;
                endcase
            end
            4'd7: begin // STOP: 1500 US (configured neutral pulse)
                case(text_col)
                    0:char_code="S"; 1:char_code="T"; 2:char_code="O";
                    3:char_code="P"; 4:char_code=":";
                    6:char_code=digit(CENTER_US / 1000);
                    7:char_code=digit((CENTER_US / 100) % 10);
                    8:char_code=digit((CENTER_US / 10) % 10);
                    9:char_code=digit(CENTER_US % 10);
                    11:char_code="U"; 12:char_code="S";
                    default:char_code=8'h20;
                endcase
            end
            default: char_code = 8'h20;
        endcase
    end

    // Geometry must not read the font bits; sample them downstream to
    // avoid a combinational feedback event loop when selecting characters.
    always @(*) begin
        text_col = 5'd31;
        text_row = 4'd15;
        local_x = 0;
        local_y = 0;
        char_col = 0;
        char_row = 0;
        char_sample = 1'b0;
        if(I_de && x>=TEXT_X && x<TEXT_X+12'd200 &&
           y>=TEXT_Y && y<TEXT_Y+12'd128) begin
            text_col = (x-TEXT_X) / CELL_W;
            text_row = (y-TEXT_Y) / CELL_H;
            local_x = (x-TEXT_X) % CELL_W;
            local_y = (y-TEXT_Y) % CELL_H;
            char_col = local_x[3:0];
            char_row = local_y[3:0];
            char_sample = 1'b1;
        end
    end

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            x          <= 12'd0;
            y          <= 12'd0;
            pulse_meta <= CENTER_US;
            pulse_sync <= CENTER_US;
            step_meta  <= 7'd10;
            step_sync  <= 7'd10;
            O_vsync    <= 1'b0;
            O_hsync    <= 1'b0;
            O_de       <= 1'b0;
            O_data     <= 24'd0;
        end else begin
            pulse_meta <= I_pulse_us;
            pulse_sync <= pulse_meta;
            step_meta  <= I_step_us;
            step_sync  <= step_meta;

            if(I_user) begin
                x <= 12'd0;
                y <= 12'd0;
            end else if(I_de) begin
                if(I_last) begin
                    x <= 12'd0;
                    y <= y + 1'b1;
                end else begin
                    x <= x + 1'b1;
                end
            end

            O_vsync <= I_vsync;
            O_hsync <= I_hsync;
            O_de    <= I_de;
            if(!I_de)
                O_data <= 24'h000000;
            else if(char_on)
                O_data <= 24'h00ff00;
            else if((x >= 12'd80) && (x < 12'd1200) &&
                    (y >= 12'd45) && (y < 12'd625))
                O_data <= 24'h102038;
            else
                O_data <= 24'h03070c;
        end
    end

endmodule
