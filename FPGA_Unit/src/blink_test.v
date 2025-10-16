module blink_test (
    input wire clk,
    input wire rst_n,
    output reg [5:0] led
);
    reg [25:0] counter = 0;
    
    always @(posedge clk) begin
        if (!rst_n) begin
            counter <= 0;
            led <= 6'b0;
        end else begin
            counter <= counter + 1;
            led <= counter[25:20];  // Different blink rates on each LED
        end
    end
endmodule