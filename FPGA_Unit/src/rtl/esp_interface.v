//
// esp_interface.v
// 
// SPI slave module for receiving data from ESP32 DevKit
// Mode 0: CPOL=0, CPHA=0 (sample on rising edge, shift on falling edge)
//

module esp_interface (
    input  wire        clk,          // System clock (27 MHz)
    input  wire        rst_n,        // Active-low reset
    
    // SPI interface
    input  wire        esp_mosi,     // Master Out Slave In
    output reg         esp_miso,     // Master In Slave Out
    input  wire        esp_sclk,     // SPI clock from master
    input  wire        esp_cs_n,     // Chip select (active low)
    
    // Parallel data interface
    output reg  [7:0]  rx_data,      // Received byte
    output reg         rx_valid,     // Data valid strobe
    input  wire        rx_ready      // Ready for next byte
);

    //==========================================================================
    // Internal signals
    //==========================================================================
    
    // Synchronize SPI signals to system clock domain
    reg [2:0] sclk_sync;
    reg [2:0] cs_sync;
    reg [1:0] mosi_sync;
    
    wire sclk_rising;
    wire sclk_falling;
    wire cs_active;
    
    // Shift register
    reg [7:0] shift_reg;
    reg [2:0] bit_count;
    
    // State
    reg       byte_ready;
    
    //==========================================================================
    // Synchronize SPI signals (avoid metastability)
    //==========================================================================
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sclk_sync <= 3'b000;
            cs_sync   <= 3'b111;
            mosi_sync <= 2'b00;
        end
        else begin
            sclk_sync <= {sclk_sync[1:0], esp_sclk};
            cs_sync   <= {cs_sync[1:0], esp_cs_n};
            mosi_sync <= {mosi_sync[0], esp_mosi};
        end
    end
    
    // Edge detection
    assign sclk_rising  = (sclk_sync[2:1] == 2'b01);
    assign sclk_falling = (sclk_sync[2:1] == 2'b10);
    assign cs_active    = !cs_sync[2];  // Active low
    
    //==========================================================================
    // SPI Mode 0: Sample on rising edge, shift on falling edge
    //==========================================================================
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            shift_reg  <= 8'h00;
            bit_count  <= 3'd0;
            byte_ready <= 1'b0;
        end
        else begin
            byte_ready <= 1'b0;  // Default: clear strobe
           
            if (!cs_active) begin
                // CS deasserted - reset
                bit_count <= 3'd0;
                shift_reg <= 8'h00;
            end
            else if (sclk_rising) begin
                // Sample MOSI on rising edge
                shift_reg <= {shift_reg[6:0], mosi_sync[1]};
                bit_count <= bit_count + 1'b1;
                
                // After 8 bits, byte is complete
                if (bit_count == 3'd7) begin
                    byte_ready <= 1'b1;
                    bit_count  <= 3'd0;
                end
            end
        end
    end
    
    //==========================================================================
    // Output received byte with valid strobe
    //==========================================================================
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_data  <= 8'h00;
            rx_valid <= 1'b0;
        end
        else begin
            if (byte_ready) begin
                rx_data  <= {shift_reg[6:0], mosi_sync[1]};  // Include last bit
                rx_valid <= 1'b1;
            end
            else begin
                rx_valid <= 1'b0;
            end
        end
    end
    
    //==========================================================================
    // MISO output (for now, just send dummy data)
    // TODO: Implement proper response when needed
    //==========================================================================
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            esp_miso <= 1'b0;
        end
        else begin
            if (!cs_active) begin
                esp_miso  <= 1'b0;
            end
            else if (sclk_falling) begin
                // Shift out dummy data on falling edge
                esp_miso <= 1'b0;  // TODO: shift out actual response
            end
        end
    end

endmodule
