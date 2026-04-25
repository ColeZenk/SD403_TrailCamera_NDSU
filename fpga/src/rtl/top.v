/*
 * top.v
 *
 * Trail Camera IV — FPGA Top Module
 * SPI Slave → BSRAM → LCD Controller
 * I2C GPIO Expander → stepper coils + joystick buttons
 * Branchless datapath: if only for async reset
 */

module top
    (input wire sys_clk,
     // Pin 52 (27MHz)
     input wire btn,
     // Pin 4 (S2 button)

     // SPI from ESP32 DevKit
     input wire esp_mosi,
     // Pin 77
     output wire esp_miso,
     // Pin 76
     input wire esp_sclk,
     // Pin 48
     input wire esp_cs_n,
     // Pin 49

     // LCD RGB interface
     output wire lcd_clk,
     output wire lcd_de,
     output wire lcd_hsync,
     output wire lcd_vsync,
     output wire [4 : 0] lcd_r,
     output wire [5 : 0] lcd_g,
     output wire [4 : 0] lcd_b,

     // I2C GPIO Expander (open-drain, external 4.7k pull-ups to 3.3V)
     inout wire i2c_sda,
     // Pin 82 (IOT11A) LVCMOS18
     input wire i2c_scl,
     // Pin 81 (IOT11B) LVCMOS18

     // Physical GPIO — driven by i2c_gpio_expander registers
     // Stepper coils: output (DIR_REG bits [3:0] = 0)
     output wire step_1,
     // Pin 25
     output wire step_2,
     // Pin 26
     output wire step_3,
     // Pin 27
     output wire step_4,
     // Pin 28

     // Joystick buttons: input (DIR_REG bits [2:0] = 1 on reset)
     input wire button_L,
     // Pin 84
     input wire button_R,
     // Pin 83
     input wire button_S,
     // Pin 85

     // PSRAM (internal to GW1NR package -- no pin constraints needed)
     output wire [1:0] O_psram_ck,
     output wire [1:0] O_psram_ck_n,
     inout wire [1:0] IO_psram_rwds,
     inout wire [15:0] IO_psram_dq,
     output wire [1:0] O_psram_reset_n,
     output wire [1:0] O_psram_cs_n,

     // Debug LEDs
     output wire [5 : 0] led);

        // ==========================================================
        // Power-on reset
        // ==========================================================
        reg [7 : 0] reset_counter = 8'd0;
        reg sys_rst_n = 1'b0;
        wire rst_done = (reset_counter == 8'd255);

        always @(posedge sys_clk) begin
                reset_counter <= reset_counter + {7'd0, ~rst_done};
                sys_rst_n <= rst_done;
        end

        // ==========================================================
        // PLL: 27 MHz -> 81 MHz + 81 MHz phase-shifted
        // ==========================================================
        wire clk_81m;
        wire clk_81m_p;

        Gowin_rPLL pll(
            .clkout(clk_81m),
            .clkoutp(clk_81m_p),
            .clkin(sys_clk)
        );

        // ==========================================================
        // PSRAM Controller (4 MB HyperRAM, internal to GW1NR)
        // ==========================================================
        localparam PSRAM_FREQ = 81_000_000;
        localparam PSRAM_LATENCY = 3;

        // PSRAM signals — muxed between SPI write and scheduler read
        wire sched_psram_read;
        wire [21:0] sched_psram_addr;
        reg spi_psram_write;
        reg [21:0] spi_psram_addr;
        reg [15:0] spi_psram_din;

        wire psram_read_mux  = sched_psram_read & ~spi_psram_write;
        wire psram_write_mux = spi_psram_write;
        wire [21:0] psram_addr_mux = spi_psram_write ? spi_psram_addr
                                                      : sched_psram_addr;
        wire [15:0] psram_dout;
        wire psram_busy;

        assign O_psram_reset_n = {sys_rst_n, sys_rst_n};
        assign O_psram_ck_n = 2'b00;

        PsramController #(
            .FREQ(PSRAM_FREQ),
            .LATENCY(PSRAM_LATENCY)
        ) psram_ctrl(
            .clk(clk_81m),
            .clk_p(clk_81m_p),
            .resetn(sys_rst_n),
            .read(psram_read_mux),
            .write(psram_write_mux),
            .addr(psram_addr_mux),
            .din(spi_psram_din),
            .byte_write(1'b0),
            .dout(psram_dout),
            .busy(psram_busy),
            .O_psram_ck(O_psram_ck),
            .IO_psram_rwds(IO_psram_rwds),
            .IO_psram_dq(IO_psram_dq),
            .O_psram_cs_n(O_psram_cs_n)
        );

        // ==========================================================
        // Parameters
        // ==========================================================
        localparam BRAM_SIZE = 32768;
        localparam BRAM_ADDR_WIDTH = 15;

        // ==========================================================
        // Internal signals
        // ==========================================================
        wire [7 : 0] esp_rx_data;
        wire esp_rx_valid;
        wire esp_rx_ready = 1'b1;
        wire esp_cs_active; // 3-stage synchronized CS from esp_interface

        reg [BRAM_ADDR_WIDTH - 1 : 0] bram_write_addr;
        reg [7 : 0] bram_write_data;
        reg bram_write_en;

        wire [14 : 0] lcd_bram_addr;
        wire [7 : 0] bram_read_data;

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
        wire [7 : 0] i2c_gpio_out;
        wire [7 : 0] i2c_gpio_dir;
        wire [7 : 0] i2c_gpio_invert;

        // Button inputs registered for metastability before feeding into
        // expander Layout: gpio_in[2]=button_S, [1]=button_R, [0]=button_L
        reg [2 : 0] btn_sync_0, btn_sync_1;
        always @(posedge sys_clk or negedge sys_rst_n) begin
                if (!sys_rst_n) begin
                        btn_sync_0 <= 3'b111;
                        btn_sync_1 <= 3'b111;
                end else begin
                        btn_sync_0 <= {button_S, button_R, button_L};
                        btn_sync_1 <= btn_sync_0;
                end
        end

        wire [7 : 0] i2c_gpio_in = {5'b00000,
                                    btn_sync_1}; // upper 5 bits unused

        // ==========================================================
        // I2C GPIO Expander
        //
        // Register convention (ESP32 sets on boot):
        //   gpio_out[3:0] -> step_1..4   (DIR bits[3:0] = 0 = output)
        //   gpio_in[2:0]  -> L, R, S     (DIR bits[2:0] = 1 = input)
        //   gpio_in[7:3]  -> unused
        //   ESP32 writes DIR_REG = 0xF0 on boot:
        //     bits[7:4] = 1 → unused inputs
        //     bits[3:0] = 0 → stepper outputs (step gates: step_N = out[N] &
        //     ~dir[N])
        //   Buttons are on gpio_in port (separate wires), DIR does not gate
        //   them i.e. 0b11110000 = 0xF0
        // ==========================================================
        i2c_gpio_expander #(.DEVICE_ADDR(7'h27))
            gpio_expander(.clk(sys_clk),
                          .rst_n(sys_rst_n),
                          .scl_i(i2c_scl),
                          .sda_i(i2c_sda),
                          // reads before tristate assignment
                          .sda_oe(sda_oe),
                          .gpio_out(i2c_gpio_out),
                          .gpio_dir(i2c_gpio_dir),
                          .gpio_invert(i2c_gpio_invert),
                          .gpio_in(i2c_gpio_in));

        // Stepper outputs gated by direction register — only drive when
        // configured as output gpio_dir[n]=0 means output; gpio_dir[n]=1 means
        // input (high-Z effectively)
        assign step_1 = i2c_gpio_out[0] & ~i2c_gpio_dir[0];
        assign step_2 = i2c_gpio_out[1] & ~i2c_gpio_dir[1];
        assign step_3 = i2c_gpio_out[2] & ~i2c_gpio_dir[2];
        assign step_4 = i2c_gpio_out[3] & ~i2c_gpio_dir[3];

        // ==========================================================
        // TX output buffer (compressed data from scheduler)
        //   Scheduler writes -> buffer -> ESP reads on MISO
        //   Max compressed frame ~200 bytes, 512 depth is plenty
        // ==========================================================
        reg [7:0] tx_buf [0:511];
        reg [7:0] tx_rd_data;
        reg [8:0] tx_wr_ptr;
        reg [8:0] tx_rd_ptr;
        reg [8:0] tx_len;
        wire [7:0] tx_cur_byte = (tx_rd_ptr < tx_len) ? tx_rd_data : 8'd0;

        always @(posedge sys_clk) begin
                if (sched_tx_valid)
                        tx_buf[tx_wr_ptr] <= sched_tx_data;
                tx_rd_data <= tx_buf[tx_rd_ptr];
        end
        wire esp_tx_next;

        // ==========================================================
        // ESP SPI Interface
        // ==========================================================
        esp_interface esp_slave(.clk(sys_clk),
                                .rst_n(sys_rst_n),
                                .esp_mosi(esp_mosi),
                                .esp_miso(esp_miso),
                                .esp_sclk(esp_sclk),
                                .esp_cs_n(esp_cs_n),
                                .rx_data(esp_rx_data),
                                .rx_valid(esp_rx_valid),
                                .rx_ready(esp_rx_ready),
                                .tx_data(tx_cur_byte),
                                .tx_next(esp_tx_next),
                                .cs_active_sync(esp_cs_active));

        // ==========================================================
        // BSRAM Image Buffer
        // ==========================================================
        bram_image_buffer image_buf(.clk_wr(sys_clk),
                                    .we(bram_write_en),
                                    .addr_wr(bram_write_addr),
                                    .data_wr(bram_write_data),
                                    .clk_rd(sys_clk),
                                    .addr_rd(lcd_bram_addr),
                                    .data_rd(bram_read_data));

        // ==========================================================
        // SPI Receive Logic — branchless
        // ==========================================================
        reg esp_cs_active_prev;

        // cs_start: CS just asserted  (rising edge of cs_active = falling edge
        // of cs_n) cs_end:   CS just deasserted (falling edge of cs_active =
        // rising edge of cs_n)
        wire cs_start = ~esp_cs_active_prev & esp_cs_active;
        wire cs_end = esp_cs_active_prev & ~esp_cs_active;

        always @(posedge sys_clk or negedge sys_rst_n) begin
                if (!sys_rst_n)
                        esp_cs_active_prev <= 1'b0;
                else
                        esp_cs_active_prev <= esp_cs_active;
        end

        wire rx_write = esp_rx_valid & esp_cs_active;
        wire addr_last = (bram_write_addr == BRAM_SIZE - 1);
        wire [14 : 0] addr_inc = (bram_write_addr + 15'd1) & {15{~addr_last}};

        wire [14 : 0] addr_on_write = (addr_inc & {15{rx_write & ~cs_start}});
        wire [14 : 0] addr_on_hold =
                          (bram_write_addr & {15{~rx_write & ~cs_start}});
        wire [14 : 0] addr_next = addr_on_write | addr_on_hold;

        wire next_receiving = receiving ^ ((receiving ^ 1'b1) & cs_start) ^
                              ((receiving ^ 1'b0) & (cs_end & receiving));

        wire next_frame_ready = frame_ready ^
                                ((frame_ready ^ 1'b0) & cs_start) ^
                                ((frame_ready ^ 1'b1) & (cs_end & receiving));

        always @(posedge sys_clk or negedge sys_rst_n) begin
                if (!sys_rst_n) begin
                        bram_write_addr <= 15'd0;
                        bram_write_data <= 8'd0;
                        bram_write_en <= 1'b0;
                        receiving <= 1'b0;
                        frame_ready <= 1'b0;
                end else begin
                        bram_write_addr <= addr_next;
                        bram_write_data <= esp_rx_data;
                        bram_write_en <= rx_write;
                        receiving <= next_receiving;
                        frame_ready <= next_frame_ready;
                end
        end

        // ==========================================================
        // SPI -> PSRAM write path
        //   Buffer byte pairs, issue word writes.
        //   Runs during SPI receive (receiving == 1).
        // ==========================================================
        reg [7:0] byte_hold;
        reg byte_phase;

        always @(posedge sys_clk or negedge sys_rst_n) begin
                if (!sys_rst_n) begin
                        spi_psram_write <= 1'b0;
                        spi_psram_addr <= 22'd0;
                        spi_psram_din <= 16'd0;
                        byte_hold <= 8'd0;
                        byte_phase <= 1'b0;
                end else begin
                        spi_psram_write <= 1'b0;

                        if (cs_start) begin
                                spi_psram_addr <= cur_base;
                                byte_phase <= 1'b0;
                        end

                        if (esp_rx_valid & receiving & ~psram_busy) begin
                                if (~byte_phase) begin
                                        byte_hold <= esp_rx_data;
                                        byte_phase <= 1'b1;
                                end else begin
                                        spi_psram_din <= {esp_rx_data, byte_hold};
                                        spi_psram_write <= 1'b1;
                                        spi_psram_addr <= spi_psram_addr + 22'd2;
                                        byte_phase <= 1'b0;
                                end
                        end
                end
        end

        // ==========================================================
        // TX buffer write (scheduler -> buffer) and read (MISO out)
        // ==========================================================
        always @(posedge sys_clk or negedge sys_rst_n) begin
                if (!sys_rst_n) begin
                        tx_wr_ptr <= 9'd0;
                        tx_rd_ptr <= 9'd0;
                        tx_len <= 9'd0;
                end else begin
                        // Scheduler start: reset write pointer
                        if (cs_end & receiving) begin
                                tx_wr_ptr <= 9'd0;
                        end

                        // Scheduler writes compressed bytes
                        if (sched_tx_valid) begin
                                tx_wr_ptr <= tx_wr_ptr + 9'd1;
                        end

                        // Latch length when scheduler finishes
                        if (sched_frame_done) begin
                                tx_len <= tx_wr_ptr;
                        end

                        // Reset read pointer on new SPI transaction
                        if (cs_start)
                                tx_rd_ptr <= 9'd0;

                        // Advance read pointer on each MISO byte sent
                        if (esp_tx_next & esp_cs_active)
                                tx_rd_ptr <= tx_rd_ptr + 9'd1;
                end
        end

        // ==========================================================
        // PSRAM ping-pong frame regions
        //   Region A: 0x000000  (76800 bytes = 320*240)
        //   Region B: 0x012C00
        //   SPI writes to cur_base, scheduler diffs cur vs prev.
        //   Swap after each frame.
        // ==========================================================
        localparam [21:0] REGION_A = 22'h000000;
        localparam [21:0] REGION_B = 22'h012C00;

        reg frame_sel;
        wire [21:0] cur_base  = frame_sel ? REGION_B : REGION_A;
        wire [21:0] prev_base = frame_sel ? REGION_A : REGION_B;

        always @(posedge sys_clk or negedge sys_rst_n) begin
                if (!sys_rst_n)
                        frame_sel <= 1'b0;
                else if (sched_frame_done)
                        frame_sel <= ~frame_sel;
        end

        // ==========================================================
        // Block Scheduler (WHT compress pipeline)
        // ==========================================================
        wire sched_frame_done;
        wire sched_tx_valid;
        wire [7:0] sched_tx_data;

        block_sched #(.W(16), .TOP_K(5)) sched(
            .clk(sys_clk),
            .rst_n(sys_rst_n),
            .start(cs_end & receiving),
            .frame_done(sched_frame_done),
            .psram_read(sched_psram_read),
            .psram_addr(sched_psram_addr),
            .psram_rdata(psram_dout),
            .psram_busy(psram_busy),
            .tx_valid(sched_tx_valid),
            .tx_data(sched_tx_data),
            .tx_ready(1'b1),
            .cur_base(cur_base),
            .prev_base(prev_base)
        );

        // ==========================================================
        // LCD Controller
        // ==========================================================
        lcd_controller lcd_ctrl(.clk(sys_clk),
                                .rst_n(sys_rst_n),
                                .btn(btn),
                                .bram_addr(lcd_bram_addr),
                                .bram_data(bram_read_data),
                                .lcd_clk(lcd_clk),
                                .lcd_hsync(lcd_hsync),
                                .lcd_vsync(lcd_vsync),
                                .lcd_de(lcd_de),
                                .lcd_r(lcd_r),
                                .lcd_g(lcd_g),
                                .lcd_b(lcd_b));

        // ==========================================================
        // Debug LEDs
        // ==========================================================
        reg sched_ever_done;
        always @(posedge sys_clk or negedge sys_rst_n)
                if (!sys_rst_n) sched_ever_done <= 1'b0;
                else if (sched_frame_done) sched_ever_done <= 1'b1;

        assign led[0] = receiving;
        assign led[1] = frame_ready;
        assign led[2] = esp_cs_active;
        assign led[3] = esp_rx_valid;
        assign led[4] = sched_ever_done;
        assign led[5] = |tx_len;
endmodule
