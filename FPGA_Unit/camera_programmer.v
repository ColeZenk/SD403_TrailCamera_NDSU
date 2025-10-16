module camera_programmer (
    input wire clk,
    input wire rst_n,
    input wire mode_select,
    inout wire [7:0] data_bus,
    input wire cam_pclk,
    input wire cam_href,
    input wire cam_vsync,
    output reg cam_enable,
    input  wire uart_usb_rx,
    output wire uart_usb_tx,
    output reg prog_enable,
    output reg [5:0] led
);

    // Simple LED blink
    reg [26:0] counter = 0;
    always @(posedge clk) begin
        counter <= counter + 1;
        led <= counter[26:21];
        cam_enable <= 0;
        prog_enable <= 1;
    end

    // UART passthrough between USB and ESP32
    // USB RX (from PC) -> data_bus[1] (to ESP32 RX on pin 85)
    // data_bus[0] (from ESP32 TX on pin 86) -> USB TX (to PC)
    
    assign uart_usb_tx = data_bus[0];  // ESP32 TX -> USB TX
    assign data_bus[1] = uart_usb_rx;   // USB RX -> ESP32 RX
    
    // Other data_bus pins unused
    assign data_bus[7:2] = 6'bz;

endmodule
