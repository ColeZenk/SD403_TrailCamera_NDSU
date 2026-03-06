/*
 * top.v
 *
 * Trail Camera IV — FPGA Top Module
 * SPI Slave → BSRAM → LCD Controller
 * I2C GPIO Expander → stepper coils + joystick buttons
 * Branchless datapath: if only for async reset
 */

module top (
    input  wire sys_clk,        // Pin 52 (27MHz)
    input  wire btn,            // Pin 4 (S2 button)

    // SPI from ESP32 DevKit
    input  wire esp_mosi,       // Pin 77
    output wire esp_miso,       // Pin 76
    input  wire esp_sclk,       // Pin 48
    input  wire esp_cs_n,       // Pin 49

    // LCD RGB interface
    output wire lcd_clk,
    output wire lcd_de,
    output wire lcd_hsync,
    output wire lcd_vsync,
    output wire [4:0] lcd_r,
    output wire [5:0] lcd_g,
    output wire [4:0] lcd_b,

    // I2C GPIO Expander (open-drain, external 4.7k pull-ups to 3.3V)
    inout  wire i2c_sda,        // Pin 82 (IOT11A) LVCMOS18
    input  wire i2c_scl,        // Pin 81 (IOT11B) LVCMOS18

    // Physical GPIO — driven by i2c_gpio_expander registers
    // Stepper coils: output (DIR_REG bits [3:0] = 0)
    output wire step_1,         // Pin 25
    output wire step_2,         // Pin 26
    output wire step_3,         // Pin 27
    output wire step_4,         // Pin 28

    // Joystick buttons: input (DIR_REG bits [7:4] = 1 on reset)
    input  wire button_L,       // Pin 79
    input  wire button_R,       // Pin 80
    input  wire button_U,       // Pin 83
    input  wire button_D,       // Pin 84
    input  wire button_S,       // Pin 85

    // Debug LEDs
    output wire [5:0] led
);

    // ==========================================================
    // Power-on reset
    // ==========================================================
    reg [7:0] reset_counter = 8'd0;
    reg sys_rst_n = 1'b0;
    wire rst_done = (reset_counter == 8'd255);

    always @(posedge sys_clk) begin
        reset_counter <= reset_counter + {7'd0, ~rst_done};
        sys_rst_n     <= rst_done;
    end

    // ==========================================================
    // Parameters
    // ==========================================================
    localparam BRAM_SIZE       = 32768;
    localparam BRAM_ADDR_WIDTH = 15;

    // ==========================================================
    // Internal signals
    // ==========================================================
    wire [7:0] esp_rx_data;
    wire       esp_rx_valid;
    wire       esp_rx_ready = 1'b1;

    reg  [BRAM_ADDR_WIDTH-1:0] bram_write_addr;
    reg  [7:0]                 bram_write_data;
    reg                        bram_write_en;

    wire [14:0] lcd_bram_addr;
    wire [7:0]  bram_read_data;

    reg frame_ready;
    reg receiving;

    // ==========================================================
    // I2C SDA tristate
    // sda_oe=1 -> drive low (open-drain pull-down)
    // sda_oe=0 -> release (external pull-up handles high)
    // ==========================================================
    wire sda_oe;
    assign i2c_sda = sda_oe ? 1'b0 : 1'bz;

    // ==========================================================
    // GPIO expander register outputs
    // ==========================================================
    wire [7:0] i2c_gpio_out;
    wire [7:0] i2c_gpio_dir;
    wire [7:0] i2c_gpio_invert;

    // Button inputs registered for metastability before feeding into expander
    // Packed: [7:4] = buttons [4:0] mapped to [7:3], [3:0] = stepper feedback (unused)
    // Layout: gpio_in[4]=button_S, [3]=button_D, [2]=button_U, [1]=button_R, [0]=button_L
    reg [4:0] btn_sync_0, btn_sync_1;
    always @(posedge sys_clk or negedge sys_rst_n) begin
        if (!sys_rst_n) begin
            btn_sync_0 <= 5'b11111;
            btn_sync_1 <= 5'b11111;
        end else begin
            btn_sync_0 <= {button_S, button_D, button_U, button_R, button_L};
            btn_sync_1 <= btn_sync_0;
        end
    end

    wire [7:0] i2c_gpio_in = {3'b000, btn_sync_1};  // upper 3 bits unused

    // ==========================================================
    // I2C GPIO Expander
    //
    // Register convention (ESP32 sets on boot):
    //   DIR_REG    = 0xF8  (bits[7:3]=1 inputs, bits[2:0] unused outputs)
    //                       wait — steppers on [3:0], buttons on [7:3]:
    //   DIR_REG    = 0xF8  → bits[7:3] input (buttons on [4:0] = gpio_in[4:0])
    //                         bits[2:0] output... but steppers are 4 bits
    //   Actual:
    //   gpio_out[3:0] -> step_1..4  (DIR bits[3:0] = 0 = output)
    //   gpio_in[4:0]  -> buttons    (DIR bits[4:0] = 1 = input, but these are IN wires)
    //   DIR_REG = 0xE0  (bits[7:5]=1 unused inputs, bits[4:0] don't matter for in,
    //                    bits[3:0]=0 output for steppers)
    //   Simplest: ESP32 writes DIR=0xF0 on boot
    //     [7:4]=1 → inputs (buttons on gpio_in[4:0], mapped to [4:0])
    //     [3:0]=0 → outputs (steppers on gpio_out[3:0])
    // ==========================================================
    i2c_gpio_expander #(
        .DEVICE_ADDR(7'h27)
    ) gpio_expander (
        .clk        (sys_clk),
        .rst_n      (sys_rst_n),
        .scl_i      (i2c_scl),
        .sda_i      (i2c_sda),   // reads before tristate assignment
        .sda_oe     (sda_oe),
        .gpio_out   (i2c_gpio_out),
        .gpio_dir   (i2c_gpio_dir),
        .gpio_invert(i2c_gpio_invert),
        .gpio_in    (i2c_gpio_in)
    );

    // Stepper outputs gated by direction register — only drive when configured as output
    // gpio_dir[n]=0 means output; gpio_dir[n]=1 means input (high-Z effectively)
    assign step_1 = i2c_gpio_out[0] & ~i2c_gpio_dir[0];
    assign step_2 = i2c_gpio_out[1] & ~i2c_gpio_dir[1];
    assign step_3 = i2c_gpio_out[2] & ~i2c_gpio_dir[2];
    assign step_4 = i2c_gpio_out[3] & ~i2c_gpio_dir[3];

    // ==========================================================
    // ESP SPI Interface
    // ==========================================================
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

    // ==========================================================
    // BSRAM Image Buffer
    // ==========================================================
    bram_image_buffer image_buf (
        .clk_wr   (sys_clk),
        .we       (bram_write_en),
        .addr_wr  (bram_write_addr),
        .data_wr  (bram_write_data),
        .clk_rd   (sys_clk),
        .addr_rd  (lcd_bram_addr),
        .data_rd  (bram_read_data)
    );

    // ==========================================================
    // SPI Receive Logic — branchless
    // ==========================================================
    reg esp_cs_n_prev;

    wire cs_sync    = esp_cs_n;
    wire cs_falling = esp_cs_n_prev & ~cs_sync;
    wire cs_rising  = ~esp_cs_n_prev & cs_sync;
    wire cs_active  = ~cs_sync;

    wire rx_write     = esp_rx_valid & cs_active;
    wire addr_last    = (bram_write_addr == BRAM_SIZE - 1);
    wire [14:0] addr_inc = (bram_write_addr + 15'd1) & {15{~addr_last}};

    wire [14:0] addr_on_write = (addr_inc        & {15{rx_write & ~cs_falling}});
    wire [14:0] addr_on_hold  = (bram_write_addr & {15{~rx_write & ~cs_falling}});
    wire [14:0] addr_next     = addr_on_write | addr_on_hold;

    wire next_receiving   = receiving   ^ ((receiving   ^ 1'b1) & cs_falling)
                                        ^ ((receiving   ^ 1'b0) & (cs_rising & receiving));

    wire next_frame_ready = frame_ready ^ ((frame_ready ^ 1'b0) & cs_falling)
                                        ^ ((frame_ready ^ 1'b1) & (cs_rising & receiving));

    always @(posedge sys_clk or negedge sys_rst_n) begin
        if (!sys_rst_n) begin
            bram_write_addr <= 15'd0;
            bram_write_data <= 8'd0;
            bram_write_en   <= 1'b0;
            receiving       <= 1'b0;
            frame_ready     <= 1'b0;
            esp_cs_n_prev   <= 1'b1;
        end else begin
            esp_cs_n_prev   <= cs_sync;
            bram_write_addr <= addr_next;
            bram_write_data <= esp_rx_data;
            bram_write_en   <= rx_write;
            receiving       <= next_receiving;
            frame_ready     <= next_frame_ready;
        end
    end

    // ==========================================================
    // LCD Controller
    // ==========================================================
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

    // ==========================================================
    // Debug LEDs
    // ==========================================================
    assign led[0] = receiving;
    assign led[1] = frame_ready;
    assign led[2] = cs_active;
    assign led[3] = esp_rx_valid;
    assign led[4] = |i2c_gpio_out[3:0];   // any stepper active
    assign led[5] = |btn_sync_1;           // any button pressed

endmodule
