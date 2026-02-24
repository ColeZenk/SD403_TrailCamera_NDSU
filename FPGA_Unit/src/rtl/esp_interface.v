//
// esp_interface.v
//
// SPI slave — Mode 0 (CPOL=0 CPHA=0): sample rising, shift falling
// Branchless datapath: AND-mask / OR-combine, if only for async reset
//

module esp_interface (
    input  wire        clk,
    input  wire        rst_n,

    input  wire        esp_mosi,
    output reg         esp_miso,
    input  wire        esp_sclk,
    input  wire        esp_cs_n,

    output reg  [7:0]  rx_data,
    output reg         rx_valid,
    input  wire        rx_ready
);

    // ==========================================================
    // Synchronizers — 3-stage sclk/cs, 2-stage mosi
    // Pure shift, no branches
    // ==========================================================
    reg [2:0] sclk_sync, cs_sync;
    reg [1:0] mosi_sync;

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

    // edge / level detection — combinational
    wire sclk_rise = (sclk_sync[2:1] == 2'b01);
    wire sclk_fall = (sclk_sync[2:1] == 2'b10);
    wire cs_active = ~cs_sync[2];

    // ==========================================================
    // Shift register + bit counter
    //
    // On sclk_rise & cs_active: shift in mosi, increment count
    // On ~cs_active: hold zeros (AND-mask clears)
    // byte_complete when bit_count == 7 on a rising edge
    //
    // shift_next: shift in new bit, AND-masked by (sclk_rise & cs_active)
    //             held at 0 when ~cs_active
    // count_next: (count + 1) on shift, AND-masked to wrap at 8,
    //             zeroed when ~cs_active
    // ==========================================================
    reg [7:0] shift_reg;
    reg [2:0] bit_count;
    reg       byte_ready;

    wire       shifting    = sclk_rise & cs_active;
    wire       byte_done   = shifting & (bit_count == 3'd7);
    wire [7:0] shifted_in  = {shift_reg[6:0], mosi_sync[1]};

    // AND-mask: shifting selects new shifted value, ~shifting holds current
    // ~cs_active zeros everything (overrides shifting since shifting requires cs_active)
    wire [7:0] shift_next = (shifted_in & {8{shifting}}) | (shift_reg & {8{~shifting & cs_active}});
    wire [2:0] count_inc  = (bit_count + 3'd1) & {3{~byte_done}};  // wraps to 0 on byte_done
    wire [2:0] count_next = (count_inc & {3{shifting}}) | (bit_count & {3{~shifting}});
    wire [2:0] count_out  = count_next & {3{cs_active}};  // zero when CS deasserted

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            shift_reg  <= 8'h00;
            bit_count  <= 3'd0;
            byte_ready <= 1'b0;
        end
        else begin
            shift_reg  <= shift_next & {8{cs_active}};
            bit_count  <= count_out;
            byte_ready <= byte_done;
        end
    end

    // ==========================================================
    // Output latch — rx_data holds on byte_ready, rx_valid strobes
    //
    // data: AND-mask selects new capture vs hold
    // valid: just byte_ready delayed one cycle (already is — byte_ready
    //        is registered, so rx_valid = byte_ready is the strobe)
    // ==========================================================
    wire [7:0] captured   = {shift_reg[6:0], mosi_sync[1]};  // include last bit
    wire [7:0] data_next  = (captured & {8{byte_ready}}) | (rx_data & {8{~byte_ready}});

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_data  <= 8'h00;
            rx_valid <= 1'b0;
        end
        else begin
            rx_data  <= data_next;
            rx_valid <= byte_ready;
        end
    end

    // ==========================================================
    // MISO — dummy for now, zeroed when CS inactive
    // TODO: shift out actual response for bidirectional SPI
    // ==========================================================
    always @(posedge clk or negedge rst_n)
        if (!rst_n) esp_miso <= 1'b0;
        else        esp_miso <= 1'b0;  // placeholder

endmodule
