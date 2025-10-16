`timescale 1ns / 1ps

module testbench;
    reg clk = 0;
    reg rst_n = 0;
    reg uart_usb_rx = 1;
    reg esp32_tx = 1;
    
    wire uart_usb_tx;
    wire esp32_rx;
    wire [5:0] led;
    
    // 27MHz clock
    always #18.5 clk = ~clk;
    
    // DUT
    uart_passthrough dut (
        .clk(clk),
        .rst_n(rst_n),
        .uart_usb_rx(uart_usb_rx),
        .uart_usb_tx(uart_usb_tx),
        .esp32_tx(esp32_tx),
        .esp32_rx(esp32_rx),
        .led(led)
    );
    
    initial begin
        $dumpfile("uart_test.vcd");
        $dumpvars(0, testbench);
        
        // Reset
        rst_n = 0;
        #1000 rst_n = 1;
        
        // Test passthrough
        #1000 uart_usb_rx = 0;
        #100 uart_usb_rx = 1;
        #1000 esp32_tx = 0;
        #100 esp32_tx = 1;
        
        #10000 $finish;
    end
endmodule