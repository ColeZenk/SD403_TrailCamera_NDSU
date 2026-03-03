/*
 * i2c_gpio_expander.v
 *
 * Trail Camera IV — I2C Slave GPIO Expander
 * PCA9534-compatible register map
 *
 * Register Map:
 *   0x00  OUTPUT_PORT   Drive gpio_out[7:0] (output pins only, gated by DIR)
 *   0x01  INPUT_PORT    Read gpio_in[7:0] with invert mask applied (read-only)
 *   0x02  DIR_REG       1=input, 0=output  (default 0xFF — all inputs on reset)
 *   0x03  INVERT_REG    Input polarity invert mask (default 0x00)
 *
 * Usage for trail camera:
 *   gpio_in[4:0]  -> button_L, button_R, button_U, button_D, button_S
 *   gpio_out[3:0] -> step_1, step_2, step_3, step_4
 *   DIR_REG       -> ESP32 writes 0x1F on boot (lower 5 = inputs, upper 3 = outputs[2:0] unused, [3:0] outputs)
 *                    i.e. 0b00001111 = step pins out, button pins in → 0x0F... actually:
 *                    DIR[4:0]=1 (inputs=buttons), DIR[3:0]=0 (outputs=steppers) → 0b11100000 | wait
 *                    DIR=0x00 for outputs, 0xFF for inputs.
 *                    For step_1..4 on [3:0] output, buttons on [7:4] input: write 0xF0 to DIR_REG
 *
 * Electrical:
 *   SDA -> pin 82 (IOT11A) LVCMOS18, external 4.7k pull-up to 3.3V
 *   SCL -> pin 81 (IOT11B) LVCMOS18, external 4.7k pull-up to 3.3V
 *   Tristate buffer in top.v: assign i2c_sda = sda_oe ? 1'b0 : 1'bz
 *
 * I2C address: 7'h27 (configurable)
 */

module i2c_gpio_expander #(
    parameter DEVICE_ADDR = 7'h27
)(
    input  wire       clk,
    input  wire       rst_n,

    input  wire       scl_i,
    input  wire       sda_i,
    output reg        sda_oe,      // 1 = drive SDA low; 0 = release (pull-up handles high)

    output reg  [7:0] gpio_out,    // output register — wire to physical pins via dir gate
    output reg  [7:0] gpio_dir,    // direction: 1=input, 0=output
    output reg  [7:0] gpio_invert, // input invert mask
    input  wire [7:0] gpio_in      // physical input pin values (registered externally)
);

    // ================================================================
    // 3-stage synchronizers — metastability on async I2C lines
    // ================================================================
    reg [2:0] scl_sync;
    reg [2:0] sda_sync;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            scl_sync <= 3'b111;
            sda_sync <= 3'b111;
        end else begin
            scl_sync <= {scl_sync[1:0], scl_i};
            sda_sync <= {sda_sync[1:0], sda_i};
        end
    end

    wire scl_s     = scl_sync[1];
    wire sda_s     = sda_sync[1];
    wire scl_rise  = (scl_sync[2:1] == 2'b01);
    wire scl_fall  = (scl_sync[2:1] == 2'b10);
    wire start_det = scl_sync[1] & (sda_sync[2:1] == 2'b10); // SDA falls, SCL high
    wire stop_det  = scl_sync[1] & (sda_sync[2:1] == 2'b01); // SDA rises, SCL high

    // ================================================================
    // States
    // ================================================================
    localparam [3:0]
        S_IDLE      = 4'd0,
        S_ADDR      = 4'd1,   // receive 7-bit addr + R/W bit
        S_ADDR_ACK  = 4'd2,   // drive ACK/NACK
        S_REG       = 4'd3,   // receive register pointer
        S_REG_ACK   = 4'd4,
        S_WRITE     = 4'd5,   // receive data byte
        S_WRITE_ACK = 4'd6,
        S_READ      = 4'd7,   // transmit data byte
        S_READ_ACK  = 4'd8;   // wait for master ACK/NACK

    reg [3:0] state;
    reg [7:0] shift_reg;  // rx shift register (MSB first)
    reg [2:0] bit_cnt;    // counts 0..7
    reg       rw_bit;     // latched R/W from address phase
    reg [7:0] reg_addr;   // current register pointer (auto-increments on burst)
    reg [7:0] tx_byte;    // byte being clocked out during reads

    // ================================================================
    // Register read mux — combinational
    // ================================================================
    function [7:0] read_reg;
        input [7:0] addr;
        begin
            case (addr)
                8'h00:   read_reg = gpio_out;
                8'h01:   read_reg = gpio_in ^ gpio_invert; // invert applied, dir not masked (master knows dir)
                8'h02:   read_reg = gpio_dir;
                8'h03:   read_reg = gpio_invert;
                default: read_reg = 8'hFF;
            endcase
        end
    endfunction

    // ================================================================
    // State machine
    //
    // Timing contract:
    //   ACK/data bits driven on scl_fall (low phase), stable when SCL rises
    //   RX bits sampled on scl_rise
    //   START/STOP always override current state
    // ================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state       <= S_IDLE;
            sda_oe      <= 1'b0;
            gpio_out    <= 8'h00;
            gpio_dir    <= 8'hFF;   // all inputs on reset — safe default
            gpio_invert <= 8'h00;
            shift_reg   <= 8'h00;
            bit_cnt     <= 3'd0;
            rw_bit      <= 1'b0;
            reg_addr    <= 8'h00;
            tx_byte     <= 8'h00;
        end else begin

            // START/STOP override all states — always check first
            if (start_det) begin
                state   <= S_ADDR;
                bit_cnt <= 3'd0;
                sda_oe  <= 1'b0;
            end else if (stop_det) begin
                state  <= S_IDLE;
                sda_oe <= 1'b0;
            end else begin
                case (state)

                    // ------------------------------------------------
                    S_IDLE: begin
                        sda_oe <= 1'b0;
                    end

                    // ------------------------------------------------
                    // Shift in address byte (7 addr bits + R/W)
                    S_ADDR: begin
                        if (scl_rise) begin
                            shift_reg <= {shift_reg[6:0], sda_s};
                            if (bit_cnt == 3'd7) begin
                                rw_bit  <= sda_s;         // last bit = R/W
                                bit_cnt <= 3'd0;
                                state   <= S_ADDR_ACK;
                            end else
                                bit_cnt <= bit_cnt + 1'b1;
                        end
                    end

                    // ------------------------------------------------
                    // Drive ACK low during SCL low phase after address
                    // Release before SCL rises so master sees stable high after ACK
                    S_ADDR_ACK: begin
                        if (scl_fall) begin
                            if (shift_reg[7:1] == DEVICE_ADDR) begin
                                sda_oe <= 1'b1;  // ACK — pull low
                                // pre-load tx_byte now so it's ready when we enter S_READ
                                tx_byte <= read_reg(reg_addr);
                                // direction set after SCL goes high (S_ADDR_ACK_RELEASE)
                            end else begin
                                sda_oe <= 1'b0;  // NACK — not our address
                                state  <= S_IDLE;
                            end
                        end
                        if (scl_rise & (shift_reg[7:1] == DEVICE_ADDR)) begin
                            // release SDA now — ACK was held through SCL high
                            sda_oe <= 1'b0;
                            state  <= rw_bit ? S_READ : S_REG;
                        end
                    end

                    // ------------------------------------------------
                    S_REG: begin
                        if (scl_fall) sda_oe <= 1'b0;  // ensure released
                        if (scl_rise) begin
                            shift_reg <= {shift_reg[6:0], sda_s};
                            if (bit_cnt == 3'd7) begin
                                reg_addr <= {shift_reg[6:0], sda_s};
                                bit_cnt  <= 3'd0;
                                state    <= S_REG_ACK;
                            end else
                                bit_cnt <= bit_cnt + 1'b1;
                        end
                    end

                    // ------------------------------------------------
                    S_REG_ACK: begin
                        if (scl_fall) sda_oe <= 1'b1;  // ACK
                        if (scl_rise) begin
                            sda_oe <= 1'b0;  // release
                            state  <= S_WRITE;
                        end
                    end

                    // ------------------------------------------------
                    S_WRITE: begin
                        if (scl_fall) sda_oe <= 1'b0;
                        if (scl_rise) begin
                            shift_reg <= {shift_reg[6:0], sda_s};
                            if (bit_cnt == 3'd7) begin
                                bit_cnt <= 3'd0;
                                state   <= S_WRITE_ACK;
                            end else
                                bit_cnt <= bit_cnt + 1'b1;
                        end
                    end

                    // ------------------------------------------------
                    // Latch write on scl_fall after 8th bit
                    S_WRITE_ACK: begin
                        if (scl_fall) begin
                            // commit the byte to the addressed register
                            case (reg_addr)
                                8'h00: gpio_out    <= shift_reg;
                                8'h02: gpio_dir    <= shift_reg;
                                8'h03: gpio_invert <= shift_reg;
                                default: ; // 0x01 INPUT_PORT read-only
                            endcase
                            reg_addr <= reg_addr + 1'b1;  // auto-increment for burst writes
                            sda_oe   <= 1'b1;             // ACK
                        end
                        if (scl_rise) begin
                            sda_oe <= 1'b0;  // release
                            state  <= S_WRITE;
                        end
                    end

                    // ------------------------------------------------
                    // Clock out tx_byte MSB first
                    // Drive on scl_fall, stable during SCL high
                    S_READ: begin
                        if (scl_fall) begin
                            sda_oe  <= ~tx_byte[7];           // 0=drive low, 1=release
                            tx_byte <= {tx_byte[6:0], 1'b1};  // shift, pad with 1
                            if (bit_cnt == 3'd7) begin
                                bit_cnt <= 3'd0;
                                state   <= S_READ_ACK;
                            end else
                                bit_cnt <= bit_cnt + 1'b1;
                        end
                    end

                    // ------------------------------------------------
                    // Sample master ACK/NACK on scl_rise
                    // ACK (SDA low)  = master wants more bytes
                    // NACK (SDA high) = master done
                    S_READ_ACK: begin
                        if (scl_fall) sda_oe <= 1'b0;  // release SDA for master to drive
                        if (scl_rise) begin
                            if (!sda_s) begin
                                // Master ACK — load next register and continue
                                reg_addr <= reg_addr + 1'b1;
                                tx_byte  <= read_reg(reg_addr + 1'b1);
                                state    <= S_READ;
                            end else
                                state <= S_IDLE;  // Master NACK — done
                        end
                    end

                    default: state <= S_IDLE;
                endcase
            end
        end
    end

endmodule
