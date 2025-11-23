//
// lcd_controller.v
// Simplified - no multiplication, just linear scan
//

module lcd_controller (
    input  wire        clk,
    input  wire        rst_n,

    output reg  [14:0] bram_addr,
    input  wire [7:0]  bram_data,

    output wire        lcd_clk,
    output reg         lcd_hsync,
    output reg         lcd_vsync,
    output reg         lcd_de,
    output reg  [4:0]  lcd_r,
    output reg  [5:0]  lcd_g,
    output reg  [4:0]  lcd_b
);

    // Timing parameters
    localparam H_ACTIVE = 480;
    localparam H_TOTAL  = 525;
    localparam V_ACTIVE = 272;
    localparam V_TOTAL  = 286;
 
    // Clock divider
    reg pclk_div;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            pclk_div <= 1'b0;
        else
            pclk_div <= ~pclk_div;
    end
    assign lcd_clk = pclk_div;
    
    // Counters
    reg [9:0] h_count;
    reg [8:0] v_count;
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            h_count <= 10'd0;
        end
        else if (pclk_div) begin
            if (h_count >= H_TOTAL - 1)
                h_count <= 10'd0;
            else
                h_count <= h_count + 10'd1;
        end
    end
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            v_count <= 9'd0;
        end
        else if (pclk_div && (h_count >= H_TOTAL - 1)) begin
            if (v_count >= V_TOTAL - 1)
                v_count <= 9'd0;
            else
                v_count <= v_count + 9'd1;
        end
    end
    
    // Sync signals
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            lcd_hsync <= 1'b0;
            lcd_vsync <= 1'b0;
            lcd_de <= 1'b0;
        end
        else begin
            lcd_hsync <= (h_count >= 482 && h_count < 523);
            lcd_vsync <= (v_count >= 274 && v_count < 284);
            lcd_de <= (h_count < H_ACTIVE) && (v_count < V_ACTIVE);
        end
    end
    
    // Simple linear address - just scan through memory
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            bram_addr <= 15'd0;
        end
        else if (pclk_div && lcd_de) begin
            if (bram_addr < 15'd32767)
                bram_addr <= bram_addr + 15'd1;
            else
                bram_addr <= 15'd0;
        end
        else if (!lcd_de) begin
            bram_addr <= 15'd0;
        end
    end
    
    // RGB output
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            lcd_r <= 5'd0;
            lcd_g <= 6'd0;
            lcd_b <= 5'd0;
        end
        else if (lcd_de) begin
            lcd_r <= bram_data[7:3];
            lcd_g <= bram_data[7:2];
            lcd_b <= bram_data[7:3];
        end
        else begin
            lcd_r <= 5'd0;
            lcd_g <= 6'd0;
            lcd_b <= 5'd0;
        end
    end

endmodule
