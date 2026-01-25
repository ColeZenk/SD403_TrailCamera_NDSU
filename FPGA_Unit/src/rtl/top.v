/*
* top.v
*
* Trail Camera IV - FPGA Top Module
* Connects: SPI Slave -> BSRAM -> LCD Controller
* 
* Uses explicit Gowin SDPB primitives for 32KB image buffer
*/

module top (
    input  wire sys_clk,        // Pin 52 (27MHz)
    
    // Button for LCD test pattern cycling
    input  wire btn,            // Pin 4 (S2 button)

    // SPI from ESP32 DevKit
    input  wire esp_mosi,       // Pin 49
    output wire esp_miso,       // Pin 77
    input  wire esp_sclk,       // Pin 76
    input  wire esp_cs_n,       // Pin 48

    // LCD RGB interface
    output wire lcd_clk,
    output wire lcd_de,
    output wire lcd_hsync,
    output wire lcd_vsync,
    output wire [4:0] lcd_r,
    output wire [5:0] lcd_g,
    output wire [4:0] lcd_b,

    // Debug LEDs
    output wire [5:0] led
);

  //==============================================================
  // Power-on reset
  //==============================================================
  reg [7:0] reset_counter = 8'd0;
  reg sys_rst_n = 1'b0;
  
  always @(posedge sys_clk) begin
      if (reset_counter < 8'd255) begin
          reset_counter <= reset_counter + 8'd1;
          sys_rst_n <= 1'b0;
      end else begin
          sys_rst_n <= 1'b1;
      end
  end
  
  //==============================================================
  // Parameters
  //==============================================================
  
  localparam BRAM_SIZE = 32768;       // 32KB using 16 BSRAM blocks
  localparam BRAM_ADDR_WIDTH = 15;    // 15 bits for 32K addresses
  
  //==============================================================
  // Internal signals
  //==============================================================
  
  // SPI slave outputs
  wire [7:0] esp_rx_data;
  wire       esp_rx_valid;
  wire       esp_rx_ready = 1'b1;  // Always ready
  
  // BRAM write signals
  reg  [BRAM_ADDR_WIDTH-1:0] bram_write_addr;
  reg  [7:0]                 bram_write_data;
  reg                        bram_write_en;
  
  // BRAM read signals (from LCD)
  wire [14:0] lcd_bram_addr;
  wire [7:0]  bram_read_data;
  
  // Frame control
  reg frame_ready;  // New frame is ready to display
  reg receiving;    // Currently receiving SPI data
  
  //=============================================================
  // ESP SPI Interface Module
  //=============================================================
  
  esp_interface esp_slave (
      .clk        (sys_clk),
      .rst_n      (sys_rst_n),
      .esp_mosi   (esp_mosi),
      .esp_miso   (esp_miso),
      .esp_sclk   (esp_sclk),
      .esp_cs_n   (esp_cs_n),
      .rx_data    (esp_rx_data),
      .rx_valid   (esp_rx_valid),
      .rx_ready   (esp_rx_ready)
  );

  //============================================================
  // BSRAM Image Buffer (32KB using 16 SDPB blocks)
  //============================================================
  
  bram_image_buffer image_buf (
      // Write port (SPI side)
      .clk_wr   (sys_clk),
      .we       (bram_write_en),
      .addr_wr  (bram_write_addr),
      .data_wr  (bram_write_data),
      
      // Read port (LCD side)
      .clk_rd   (sys_clk),
      .addr_rd  (lcd_bram_addr),
      .data_rd  (bram_read_data)
  );
  
  //============================================================
  // SPI Receive Logic - Simple streaming
  //============================================================
  
  // Track CS edges for frame sync
  reg esp_cs_n_prev;
  wire cs_falling = esp_cs_n_prev && !esp_cs_n;
  wire cs_rising = !esp_cs_n_prev && esp_cs_n;
  
  always @(posedge sys_clk or negedge sys_rst_n) begin
      if (!sys_rst_n) begin
          bram_write_addr <= 0;
          bram_write_data <= 8'd0;
          bram_write_en   <= 1'b0;
          receiving       <= 1'b0;
          frame_ready     <= 1'b0;
          esp_cs_n_prev   <= 1'b1;
      end
      else begin
          esp_cs_n_prev <= esp_cs_n;
          bram_write_en <= 1'b0;
          
          // CS falling edge - start new frame
          if (cs_falling) begin
              bram_write_addr <= 0;
              receiving <= 1'b1;
              frame_ready <= 1'b0;
          end
          
          // CS rising edge - frame complete
          if (cs_rising && receiving) begin
              receiving <= 1'b0;
              frame_ready <= 1'b1;
          end
          
          // Receive bytes while CS is low
          if (esp_rx_valid && !esp_cs_n) begin
              bram_write_data <= esp_rx_data;
              bram_write_en <= 1'b1;
              
              // Wrap address within BRAM size
              if (bram_write_addr < BRAM_SIZE - 1)
                  bram_write_addr <= bram_write_addr + 1'b1;
              else
                  bram_write_addr <= 0;
          end
      end
  end
  
  //============================================================
  // LCD Controller
  //============================================================
  
  lcd_controller lcd_ctrl (
      .clk        (sys_clk),
      .rst_n      (sys_rst_n),
      .btn        (btn),
      .bram_addr  (lcd_bram_addr),
      .bram_data  (bram_read_data),
      .lcd_clk    (lcd_clk),
      .lcd_hsync  (lcd_hsync),
      .lcd_vsync  (lcd_vsync),
      .lcd_de     (lcd_de),
      .lcd_r      (lcd_r),
      .lcd_g      (lcd_g),
      .lcd_b      (lcd_b)
  );
  
  //============================================================
  // Debug LEDs
  //============================================================
  
  // Show system state on LEDs
  assign led[0] = receiving;        // Blips when receiving SPI data
  assign led[1] = frame_ready;      // Stays on after frame received
  assign led[2] = !esp_cs_n;        // CS active
  assign led[3] = esp_rx_valid;     // Data being clocked in
  assign led[4] = |bram_write_addr[14:10];  // Progress indicator
  assign led[5] = bram_write_en;    // Write activity

endmodule
