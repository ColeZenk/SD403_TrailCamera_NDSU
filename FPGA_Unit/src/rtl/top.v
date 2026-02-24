/*
 * top.v
 *
 * Trail Camera IV — FPGA Top Module
 * SPI Slave → BSRAM → LCD Controller
 * Branchless datapath: if only for async reset
 */

module top (
    input  wire sys_clk,        // Pin 52 (27MHz)

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

    // ==========================================================
    // Power-on reset — no async reset here, just a counter
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
    //
    // CS edge detect via XOR on prev vs current
    // Address wraps via AND-mask: (addr + 1) & {15{~addr_last}}
    // Write enable = rx_valid & cs_active (AND, no branch)
    // Frame control via XOR latch trick
    // ==========================================================
    reg esp_cs_n_prev;

    wire cs_sync    = esp_cs_n;
    wire cs_falling = esp_cs_n_prev & ~cs_sync;
    wire cs_rising  = ~esp_cs_n_prev & cs_sync;
    wire cs_active  = ~cs_sync;

    // write conditions
    wire rx_write     = esp_rx_valid & cs_active;
    wire addr_last    = (bram_write_addr == BRAM_SIZE - 1);
    wire [14:0] addr_inc = (bram_write_addr + 15'd1) & {15{~addr_last}};

    // next address: cs_falling zeros it, rx_write increments, otherwise hold
    // priority: cs_falling > rx_write > hold
    wire [14:0] addr_on_write = (addr_inc   & {15{rx_write & ~cs_falling}});
    wire [14:0] addr_on_hold  = (bram_write_addr & {15{~rx_write & ~cs_falling}});
    wire [14:0] addr_next     = addr_on_write | addr_on_hold;
    // cs_falling case: all terms zero -> addr_next = 0 (implicit)

    // frame_ready: set on cs_rising & receiving, clear on cs_falling
    // receiving:   set on cs_falling, clear on cs_rising
    // XOR latch: val ^ ((val ^ target) & trigger)
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
        end
        else begin
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
    assign led[4] = |bram_write_addr[14:10];
    assign led[5] = bram_write_en;

endmodule
