//
// lcd_controller.v
// Simplified - no multiplication, just linear scan
//

module lcd_controller (
    input  wire        clk,  //27MHz
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


    // Timing parameters for 800x480 @ ~30Hz
    localparam H_ACTIVE = 800;
    localparam H_FRONT  = 40;
    localparam H_SYNC   = 48;
    localparam H_BACK   = 40;
    localparam H_TOTAL  = H_ACTIVE + H_FRONT + H_SYNC + H_BACK;
    
    localparam V_ACTIVE = 480;
    localparam V_FRONT  = 13;
    localparam V_SYNC   = 3;
    localparam V_BACK   = 29;
    localparam V_TOTAL  = V_ACTIVE + V_FRONT + V_SYNC + V_BACK;

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
    reg [9:0] v_count;
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) h_count <= 10'd0;
        else if (pclk_div) begin
            h_count <= (h_count >= H_TOTAL - 1) ?  10'd0 : h_count + 10'd1;
        end
    end
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) v_count <= 10'd0;
        else if (pclk_div && (h_count >= H_TOTAL - 1)) begin
           v_count <= (v_count >= V_TOTAL - 1) ? 10'd0 : v_count + 10'd1;
        end
    end
    
    // Sync signals
    // Sync signals (calculated from parameters)
    wire hsync_start = (h_count == H_ACTIVE + H_FRONT);
    wire hsync_end   = (h_count == H_ACTIVE + H_FRONT + H_SYNC);
    wire vsync_start = (v_count == V_ACTIVE + V_FRONT);
    wire vsync_end   = (v_count == V_ACTIVE + V_FRONT + V_SYNC);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            lcd_hsync <= 1'b0;
            lcd_vsync <= 1'b0;
        end
        else begin
            if (hsync_start) lcd_hsync <= 1'b1;
            if (hsync_end)   lcd_hsync <= 1'b0;
            if (vsync_start) lcd_vsync <= 1'b1;
            if (vsync_end)   lcd_vsync <= 1'b0;
        end
    end
    
    // Data enable (only during visible area)
    wire visible = (h_count < H_ACTIVE) && (v_count < V_ACTIVE);
    
    // Simple linear address - just scan through memory
    always @(posedge clk or negedge rst_n) begin
       lcd_de <= (!rst_n) ? 1'b0 : visible;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) bram_addr <= 15'd0;
        else if (pclk_div && visible) begin
            bram_addr <= (bram_addr < 15'd32767) ?  bram_addr + 15'd1 : 15'd0;
        end
        else if (!visible) bram_addr <= 15'd0;
    end

    // RGB output
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            lcd_r <= 5'd0;
            lcd_g <= 6'd0;
            lcd_b <= 5'd0;
        end
        else if (lcd_de) begin
            // Convert 8-bit grayscale to RGB
            lcd_r <= bram_data[7:3];  //top 5 bits
            lcd_g <= bram_data[7:2];  //top 6
            lcd_b <= bram_data[7:3];  //top 5
        end
        else begin
            lcd_r <= 5'd0;
            lcd_g <= 6'd0;
            lcd_b <= 5'd0;
        end
    end

endmodule
