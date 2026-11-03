// 8N1 receiver. Input synchronization, false-start rejection and break recovery.
module uart_rx #(
    parameter integer CLK_HZ = 50_000_000,
    parameter integer BAUD = 115200
)(
    input wire clk,
    input wire rst_n,
    input wire rx,
    output reg [7:0] data,
    output reg valid,
    output reg framing_error
);
    localparam integer BIT_CLKS = (CLK_HZ + BAUD / 2) / BAUD;
    localparam integer HALF_CLKS = BIT_CLKS / 2;
    localparam integer TIMER_WIDTH=(BIT_CLKS>1) ? $clog2(BIT_CLKS) : 1;
    localparam [2:0] IDLE=0, START=1, DATA=2, STOP=3, WAIT_HIGH=4;
    (* async_reg = "true" *) reg rx_meta;
    (* async_reg = "true" *) reg rx_sync;
    reg [2:0] state;
    reg [TIMER_WIDTH-1:0] timer;
    reg [2:0] bit_index;
    reg [7:0] shift_data;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_meta <= 1'b1;
            rx_sync <= 1'b1;
        end else begin
            rx_meta <= rx;
            rx_sync <= rx_meta;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
            timer <= 0;
            bit_index <= 0;
            shift_data <= 0;
            data <= 0;
            valid <= 0;
            framing_error <= 0;
        end else begin
            valid <= 0;
            framing_error <= 0;
            case (state)
                IDLE: if (!rx_sync) begin
                    timer <= HALF_CLKS - 1;
                    state <= START;
                end
                START: if (timer != 0) timer <= timer - 1'b1;
                       else if (rx_sync) state <= IDLE;
                       else begin
                           timer <= BIT_CLKS - 1;
                           bit_index <= 0;
                           state <= DATA;
                       end
                DATA: if (timer != 0) timer <= timer - 1'b1;
                      else begin
                          shift_data[bit_index] <= rx_sync;
                          timer <= BIT_CLKS - 1;
                          if (bit_index == 7) state <= STOP;
                          else bit_index <= bit_index + 1'b1;
                      end
                STOP: if (timer != 0) timer <= timer - 1'b1;
                      else if (rx_sync) begin
                          data <= shift_data;
                          valid <= 1;
                          state <= IDLE;
                      end else begin
                          framing_error <= 1;
                          state <= WAIT_HIGH;
                      end
                WAIT_HIGH: if (rx_sync) state <= IDLE;
                default: state <= IDLE;
            endcase
        end
    end
endmodule
