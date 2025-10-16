module top (
    input wire clk,
    input wire rst_n,
    input wire mode_select,
    output wire [5:0] led,
    
    // 8-bit bidirectional bus
    inout wire [7:0] data_bus,
    
    // Camera signals
    input wire cam_pclk,
    input wire cam_href,
    input wire cam_vsync,
    output wire cam_enable,
    
    // USB UART
    input wire uart_usb_rx,
    output wire uart_usb_tx,
    
    // Programming control
    output wire prog_enable
);

    // Wire definitions for bidirectional bus
    wire esp32_tx = data_bus[0];  // Input from ESP32
    wire esp32_rx;                 // Output to ESP32
    
    // Instantiate UART passthrough
    uart_passthrough uart_inst (
        .clk(clk),
        .rst_n(rst_n),
        .uart_usb_rx(uart_usb_rx),
        .uart_usb_tx(uart_usb_tx),
        .esp32_tx(esp32_tx),
        .esp32_rx(esp32_rx),
        .led(led)
    );
    
    // Control signals
    assign cam_enable = 1'b0;      // Keep camera disabled
    assign prog_enable = 1'b1;     // Keep programming enabled
    
    // Bidirectional bus control
    assign data_bus[1] = esp32_rx;  // Drive ESP32 RX pin
    assign data_bus[7:2] = 6'bz;    // High-Z for unused pins

endmodule