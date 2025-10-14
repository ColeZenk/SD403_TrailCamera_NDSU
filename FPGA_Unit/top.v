module top (
    input wire clk,
    input wire rst_n,
    output reg [5:0] led
);

    reg [26:0] counter = 0;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            counter <= 0;
        else
            counter <= counter + 1;
    end

    always @(posedge clk) begin
        led <= counter[26:21];
    end

endmodule
