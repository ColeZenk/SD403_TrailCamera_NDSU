`timescale 1ns / 1ps

module esp_interface_tb;

    //==========================================================================
    // Parameters
    //==========================================================================
    
    localparam CLK_PERIOD = 37;      // 27MHz = ~37ns period
    localparam SPI_PERIOD = 200;     // 5MHz SPI clock (slower than sys_clk)
    
    //==========================================================================
    // Signals
    //==========================================================================
    
    // System
    reg clk;
    reg rst_n;
    
    // SPI interface (directly from testbench to DUT)
    reg        tb_mosi;
    wire       tb_miso;
    reg        tb_sclk;
    reg        tb_cs_n;
    
    // Output from DUT
    wire [7:0] rx_data;
    wire       rx_valid;
    reg        rx_ready;
    
    // Test tracking
    integer bytes_received;
    integer errors;
    reg [7:0] expected_data;
    
    //==========================================================================
    // DUT instantiation
    //==========================================================================
    
    esp_interface dut (
        .clk       (clk),
        .rst_n     (rst_n),
        .esp_mosi  (tb_mosi),
        .esp_miso  (tb_miso),
        .esp_sclk  (tb_sclk),
        .esp_cs_n  (tb_cs_n),
        .rx_data   (rx_data),
        .rx_valid  (rx_valid),
        .rx_ready  (rx_ready)
    );
    
    //==========================================================================
    // Clock generation
    //==========================================================================
    
    initial begin
        clk = 0;
        forever #(CLK_PERIOD/2) clk = ~clk;
    end
    
    //==========================================================================
    // SPI Master tasks (simulates ESP32)
    //==========================================================================
    
    // Send a single byte over SPI (MSB first, Mode 0)
    task spi_send_byte;
        input [7:0] data;
        integer i;
        begin
            for (i = 7; i >= 0; i = i - 1) begin
                tb_mosi = data[i];
                #(SPI_PERIOD/2);
                tb_sclk = 1;            // Rising edge - DUT samples
                #(SPI_PERIOD/2);
                tb_sclk = 0;            // Falling edge
            end
        end
    endtask
    
    // Send multiple bytes (like a DMA transfer)
    task spi_send_burst;
        input [7:0] start_val;
        input integer num_bytes;
        integer i;
        begin
            tb_cs_n = 0;                // Assert CS
            #(SPI_PERIOD);
            
            for (i = 0; i < num_bytes; i = i + 1) begin
                spi_send_byte(start_val + i);
                #(SPI_PERIOD/2);        // Small gap between bytes
            end
            
            #(SPI_PERIOD);
            tb_cs_n = 1;                // Deassert CS
        end
    endtask
    
    //==========================================================================
    // Monitor received data
    //==========================================================================
    
    always @(posedge clk) begin
        if (rx_valid) begin
            $display("[%0t] RX byte: 0x%02X (expected: 0x%02X) %s", 
                     $time, rx_data, expected_data,
                     (rx_data == expected_data) ? "OK" : "MISMATCH!");
            
            if (rx_data !== expected_data) begin
                errors = errors + 1;
            end
            
            bytes_received = bytes_received + 1;
            expected_data = expected_data + 1;
        end
    end
    
    //==========================================================================
    // Main test sequence
    //==========================================================================
    
    initial begin
        // Setup waveform dump
        $dumpfile("esp_interface_tb.vcd");
        $dumpvars(0, esp_interface_tb);
        
        // Initialize
        rst_n = 0;
        tb_mosi = 0;
        tb_sclk = 0;
        tb_cs_n = 1;
        rx_ready = 1;
        bytes_received = 0;
        errors = 0;
        expected_data = 8'hAA;
        
        // Reset
        #(CLK_PERIOD * 10);
        rst_n = 1;
        #(CLK_PERIOD * 10);
        
        $display("===========================================");
        $display("ESP Interface Testbench Starting");
        $display("===========================================");
        
        //----------------------------------------------------------------------
        // Test 1: Single byte transfer
        //----------------------------------------------------------------------
        $display("\n[TEST 1] Single byte transfer");
        expected_data = 8'hAA;
        tb_cs_n = 0;
        #(SPI_PERIOD);
        spi_send_byte(8'hAA);
        #(SPI_PERIOD * 2);
        tb_cs_n = 1;
        #(CLK_PERIOD * 20);
        
        //----------------------------------------------------------------------
        // Test 2: Multiple bytes (simulating image data)
        //----------------------------------------------------------------------
        $display("\n[TEST 2] Burst transfer (16 bytes)");
        expected_data = 8'h00;
        spi_send_burst(8'h00, 16);
        #(CLK_PERIOD * 50);
        
        //----------------------------------------------------------------------
        // Test 3: Fast burst (stress test)
        //----------------------------------------------------------------------
        $display("\n[TEST 3] Fast burst (64 bytes)");
        expected_data = 8'h40;
        spi_send_burst(8'h40, 64);
        #(CLK_PERIOD * 100);
        
        //----------------------------------------------------------------------
        // Test 4: CS deassert mid-byte (error recovery)
        //----------------------------------------------------------------------
        $display("\n[TEST 4] CS deassert mid-byte (should recover)");
        tb_cs_n = 0;
        #(SPI_PERIOD);
        tb_mosi = 1;
        #(SPI_PERIOD/2); tb_sclk = 1; #(SPI_PERIOD/2); tb_sclk = 0;
        #(SPI_PERIOD/2); tb_sclk = 1; #(SPI_PERIOD/2); tb_sclk = 0;
        #(SPI_PERIOD/2); tb_sclk = 1; #(SPI_PERIOD/2); tb_sclk = 0;
        // Deassert CS after only 3 bits
        tb_cs_n = 1;
        #(CLK_PERIOD * 20);
        
        // Now send a clean byte - should work
        $display("  Sending clean byte after glitch...");
        expected_data = 8'h55;
        tb_cs_n = 0;
        #(SPI_PERIOD);
        spi_send_byte(8'h55);
        #(SPI_PERIOD * 2);
        tb_cs_n = 1;
        #(CLK_PERIOD * 20);
        
        //----------------------------------------------------------------------
        // Results
        //----------------------------------------------------------------------
        #(CLK_PERIOD * 50);
        
        $display("\n===========================================");
        $display("Test Complete");
        $display("  Bytes received: %0d", bytes_received);
        $display("  Errors: %0d", errors);
        $display("===========================================");
        
        if (errors == 0)
            $display("*** PASS ***");
        else
            $display("*** FAIL ***");
        
        $finish;
    end
    
    //==========================================================================
    // Timeout watchdog
    //==========================================================================
    
    initial begin
        #(1000000);  // 1ms timeout
        $display("ERROR: Testbench timeout!");
        $finish;
    end

endmodule
