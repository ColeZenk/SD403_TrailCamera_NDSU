module uart_passthrough (
    input wire clk,
    input wire rst_n,
    
    // USB UART (to/from PC)
    input wire uart_usb_rx,
    output wire uart_usb_tx,
    
    // ESP32 UART
    input wire esp32_tx,
    output reg esp32_rx,
    
    output reg [5:0] led
);

    // Direct passthrough connections
    assign uart_usb_tx = esp32_tx;  // ESP32 TX -> USB TX
    
    // Counter for heartbeat LED
    reg [24:0] counter = 0;
    
    always @(posedge clk) begin
        if (!rst_n) begin
            esp32_rx <= 1'b1;  // UART idle state
            led <= 6'b0;
            counter <= 0;
        end else begin
            esp32_rx <= uart_usb_rx;  // USB RX -> ESP32 RX
            counter <= counter + 1;
            
            // LED indicators
            led[0] <= counter[24];     // Heartbeat
            led[1] <= ~uart_usb_rx;    // RX activity (inverted)
            led[2] <= ~esp32_tx;       // TX activity (inverted)
            led[3] <= counter[23];     // Faster blink
            led[5:4] <= 2'b00;
        end
    end

endmodule