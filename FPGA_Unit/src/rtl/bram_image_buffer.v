/*
 * bram_image_buffer.v
 *
 * 32KB simple dual-port RAM for Tang Nano 9K BSRAM inference.
 * Write port: SPI receive side
 * Read port:  LCD controller side
 *
 * 32768 x 8-bit = 32KB
 * Uses full 15-bit address on both ports.
 */

module bram_image_buffer (
    // Write port (SPI side)
    input  wire        clk_wr,
    input  wire        we,
    input  wire [14:0] addr_wr,
    input  wire [7:0]  data_wr,

    // Read port (LCD side)
    input  wire        clk_rd,
    input  wire [14:0] addr_rd,
    output reg  [7:0]  data_rd
);

    // 32KB - Gowin tools will infer 16x BSRAM SDPB blocks from this
    reg [7:0] mem [0:32767];

    // Write port - synchronous
    always @(posedge clk_wr) begin
        if (we)
            mem[addr_wr] <= data_wr;
    end

    // Read port - synchronous (1-cycle read latency, matches BSRAM behavior)
    always @(posedge clk_rd) begin
        data_rd <= mem[addr_rd];
    end

endmodule
