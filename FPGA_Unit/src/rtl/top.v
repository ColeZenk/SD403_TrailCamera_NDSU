/*
* top.v
*
* This file is essential the main for the FPGA unit
* The scope will include one main state machine as well as tieing 
* all of the logic management mechanisms (ex. SPI slave, LCD control etc.)
*
*/

module top (
    input  wire sys_clk,        // Pin 52 (27MHz)
    // input  wire sys_rst_n,      // Button or pull-up

    // SPI from ESP32 DevKit
    input  wire esp_mosi,       // Assign based on your wiring
    output wire esp_miso,
    input  wire esp_sclk,
    input  wire esp_cs_n,

    // LCD RGB interface
    output wire lcd_clk,        // IOT37A (RGB_CK)
    output wire lcd_de,         // IOT24B (RGB_DE)
    output wire lcd_hsync,      // IOT17B (RGB_HS)
    output wire lcd_vsync,      // IOT23B (RGB_VSYNC)
    output wire [4:0] lcd_r,    // RGB_R[7:3]
    output wire [5:0] lcd_g,    // RGB_G[7:2]
    output wire [4:0] lcd_b,    // RGB_B[7:3]

    // Debug LEDs
    output wire [5:0] led
);

// Power-on reset
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
// Internal signals
//==============================================================

// SPI slave (to ESP) outputs
wire [7:0] esp_rx_data;
wire       esp_rx_valid;
wire       esp_rx_ready;

// Image buffer
localparam IMAGE_SIZE = 32768;
localparam ADDR_WIDTH = 15;

// BRAM signals
reg [ADDR_WIDTH-1:0] bram_write_addr;
reg [7:0]            bram_write_data;
reg                  bram_write_en;
reg [ADDR_WIDTH-1:0] bram_read_addr;
wire [7:0]           bram_read_data;

// Frame control
reg frame_complete;

// State machine
localparam IDLE         = 3'b000;
localparam RECEIVING    = 3'b001;
localparam PROCESSING   = 3'b010;
localparam TRANSMITTING = 3'b011;

reg [2:0] state;

//=============================================================
// ESP Interface Module
//=============================================================

esp_interface esp_slave(
    .clk        (sys_clk),
    .rst_n      (sys_rst_n),
    .spi_mosi   (esp_mosi),
    .spi_miso   (esp_miso),
    .spi_sclk   (esp_sclk),
    .spi_cs_n   (esp_cs_n),
    .rx_data    (esp_rx_data),
    .rx_valid   (esp_rx_valid),
    .rx_ready   (esp_rx_ready)
);

//============================================================
// Block RAM for Memory storage
//============================================================

reg [7:0] image_bram [0:IMAGE_SIZE-1];

always @(posedge sys_clk) begin
    if (bram_write_en) begin
        image_bram[bram_write_addr] <= bram_write_data;
    end
end

assign bram_read_data = image_bram[bram_read_addr];

// // Initialize BRAM with test pattern
// integer i;
//
// initial begin
//     for (i = 0; i < IMAGE_SIZE; i = i + 1) begin
//         // Checkerboard pattern
//         image_bram[i] = ((i[7:4] ^ i[3:0]) & 1) ? 8'hFF : 8'h00;
//     end
// end

//============================================================
// LCD Display Controller
//============================================================
//
// wire [14:0] lcd_bram_addr;
// wire [7:0]  lcd_bram_data;
//
// // Mux BRAM reads between write port and LCD read port
// assign lcd_bram_data = image_bram[lcd_bram_addr];
//
// rgb_lcd_controller lcd_ctrl (
//     .clk        (sys_clk),
//     .rst_n      (sys_rst_n),
//     .bram_addr  (lcd_bram_addr),
//     .bram_data  (lcd_bram_data),
//     .lcd_clk    (lcd_clk),
//     .lcd_hsync  (lcd_hsync),
//     .lcd_vsync  (lcd_vsync),
//     .lcd_de     (lcd_de),
//     .lcd_r      (lcd_r),
//     .lcd_g      (lcd_g),
//     .lcd_b      (lcd_b)
// );
//

//============================================================
// LCD Display Controller - TEST VERSION
//============================================================

// Create a small separate test pattern memory for LCD
reg [7:0] lcd_test_pattern;
wire [14:0] lcd_bram_addr;

// Simple test pattern generator
always @(posedge sys_clk) begin
    // Red in top half, blue in bottom half
    if (lcd_bram_addr[14])  // Top half of screen
        lcd_test_pattern <= 8'hE0;  // Red-ish
    else
        lcd_test_pattern <= 8'h1F;  // Blue-ish
end

rgb_lcd_controller lcd_ctrl (
    .clk        (sys_clk),
    .rst_n      (sys_rst_n),
    .bram_addr  (lcd_bram_addr),
    .bram_data  (lcd_test_pattern),  // Use test pattern, not BRAM
    .lcd_clk    (lcd_clk),
    .lcd_hsync  (lcd_hsync),
    .lcd_vsync  (lcd_vsync),
    .lcd_de     (lcd_de),
    .lcd_r      (lcd_r),
    .lcd_g      (lcd_g),
    .lcd_b      (lcd_b)
);

//============================================================
// Image Buffer Management
//============================================================

always @(posedge sys_clk or negedge sys_rst_n) begin
  if(!sys_rst_n) begin
    bram_write_addr <= 15'd0;
    bram_write_data <= 8'd0;
    bram_write_en   <= 1'b0 ;
    frame_complete  <= 1'b0 ;
  end
  else begin
    bram_write_en <= 1'b0;


    if(esp_rx_valid && state == RECEIVING) begin
       bram_write_data <= esp_rx_data;
       bram_write_addr <= bram_write_addr + 1'b1;
       bram_write_en   <= 1'b1;

      //check just to be safe
      if (bram_write_addr == IMAGE_SIZE - 1) begin
        frame_complete  <= 1'b1 ;
        bram_write_addr <= 15'd0;
      end
    end
    else if (state == PROCESSING) begin
      frame_complete <= 1'b0;
    end
  end
end

//============================================================
// Main state machine (kind of like a while(1) loop in C)
//============================================================

always @(posedge sys_clk or negedge sys_rst_n) begin
  if (!sys_rst_n) begin
    state <= IDLE;
  end
  else begin
    case (state)
       IDLE: begin
         if (!esp_cs_n) begin // Chip select asserted, now good to recieve 
           state <= RECEIVING;
         end
      end

      RECEIVING:begin
        if (frame_complete) begin
          state <= PROCESSING;
        end
      end

      PROCESSING: begin
        //TODO:implemment FFT (start with IP block)
        state <= TRANSMITTING;
      end

      TRANSMITTING: begin
        //TODO:add LoRa functionality (before FFT)
        state <= IDLE;
      end

      default: state <= IDLE;
    endcase
  end
end

//============================================================
// Debugging with LEDs for now
//============================================================
  // wire [5:0] led;
  // assign led_0 = led[0]; 
  // assign led_1 = led[1];
  // assign led_2 = led[2];
  // assign led_3 = led[3];
  // assign led_4 = led[4];
  // assign led_5 = led[5];

// reg [23:0] led_update_counter = 24'd0;
// reg [5:0]  led_display = 6'd0;
//
// always @(posedge sys_clk) begin
//   led_update_counter <= led_update_counter +24'd1;
//
//   if(led_update_counter == 24'd0) begin
//     led_display[0] = (state == IDLE        );
//     led_display[1] = (state == RECEIVING   );
//     led_display[2] = (state == PROCESSING  );
//     led_display[3] = (state == TRANSMITTING);
//     led_display[4] = frame_complete;
//     led_display[5] = !esp_cs_n;
//   end
// end
//
// assign led = led_display;
  //Placeholders
  
  //LoRa (*todo)
  // assign lora_tx  = 1'b1;
 
  endmodule
