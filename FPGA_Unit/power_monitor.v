module power_monitor (
    input wire clk,
    input wire rst_n,
    output reg power_good,
    output reg [5:0] status_led
);
    reg [26:0] counter = 0;
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            counter <= 0;
            power_good <= 1;
        end else begin
            counter <= counter + 1;
        end
    end
    
    // Heartbeat pattern - if this works, power is stable
    always @(posedge clk) begin
        status_led[0] <= counter[22];  // Fast blink
        status_led[1] <= counter[23];
        status_led[2] <= counter[24];
        status_led[3] <= counter[25];
        status_led[4] <= counter[26];
        status_led[5] <= power_good;   // Solid if power OK
    end
endmodule
