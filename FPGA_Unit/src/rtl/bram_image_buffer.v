/*
 * bram_image_buffer.v
 * 
 * Single-clock dual-port RAM for reliable BSRAM inference
 */

module bram_image_buffer (
    // Write port (SPI side)
    input  wire        clk_wr,
    input  wire        we,
    input  wire [14:0] addr_wr,
    input  wire [7:0]  data_wr,
    
    // Read port (LCD side)  
    input  wire        clk_rd,      // Ignored - using clk_wr for both
    input  wire [14:0] addr_rd,
    output reg  [7:0]  data_rd
);

    // 2KB RAM 
    reg [11:0] mem [0:8191];
    
    // Single clock for both ports
    always @(posedge clk_wr) begin
        if (we) begin
            mem[addr_wr[10:0]] <= data_wr;
        end
        data_rd <= mem[addr_rd[10:0]];
    end

endmodule
