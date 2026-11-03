// 8N1 transmitter. A byte is accepted only when valid && ready.
module uart_tx #(
    parameter integer CLK_HZ = 50_000_000,
    parameter integer BAUD = 115200
)(
    input wire clk,
    input wire rst_n,
    input wire [7:0] data,
    input wire valid,
    output wire ready,
    output reg tx
);
    localparam integer BIT_CLKS = (CLK_HZ + BAUD / 2) / BAUD;
    localparam integer TIMER_WIDTH=(BIT_CLKS>1) ? $clog2(BIT_CLKS) : 1;
    reg busy;
    reg [TIMER_WIDTH-1:0] timer;
    reg [3:0] bit_index;
    reg [9:0] frame;
    assign ready = !busy;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            busy <= 0;
            timer <= 0;
            bit_index <= 0;
            frame <= 10'h3ff;
            tx <= 1;
        end else if (!busy) begin
            tx <= 1;
            if (valid) begin
                frame <= {1'b1, data, 1'b0};
                bit_index <= 0;
                timer <= BIT_CLKS - 1;
                tx <= 0;
                busy <= 1;
            end
        end else if (timer != 0) begin
            timer <= timer - 1'b1;
        end else if (bit_index == 9) begin
            tx <= 1;
            busy <= 0;
        end else begin
            bit_index <= bit_index + 1'b1;
            tx <= frame[bit_index + 1'b1];
            timer <= BIT_CLKS - 1;
        end
    end
endmodule
