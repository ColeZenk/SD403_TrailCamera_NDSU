module uart_loopback (
    input wire clk,
    input wire rst_n,
    input wire uart_usb_rx,
    output wire uart_usb_tx,
    output reg [5:0] led
);
    // Simple loopback
    assign uart_usb_tx = uart_usb_rx;
    
    // Activity indicator
    reg [23:0] counter = 0;
    always @(posedge clk) begin
        if (!rst_n) begin
            counter <= 0;
            led <= 6'b0;
        end else begin
            counter <= counter + 1;
            led[0] <= counter[23];   // Heartbeat
            led[1] <= ~uart_usb_rx;  // RX activity
            led[5:2] <= 4'b0;
        end
    end
endmodule