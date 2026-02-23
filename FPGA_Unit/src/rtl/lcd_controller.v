//
// lcd_controller.v
// RGB LCD controller with button-cycled test patterns
//

module lcd_controller (
    input  wire        clk,       // 27MHz
    input  wire        rst_n,
    input  wire        btn,       // Button input (active low)

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

    // Timing parameters for 480x272 LCD (common 4.3" size)
    localparam H_ACTIVE = 480;
    localparam H_FRONT  = 2;
    localparam H_SYNC   = 41;
    localparam H_BACK   = 2;
    localparam H_TOTAL  = H_ACTIVE + H_FRONT + H_SYNC + H_BACK;  // 525

    localparam V_ACTIVE = 272;
    localparam V_FRONT  = 2;
    localparam V_SYNC   = 10;
    localparam V_BACK   = 2;
    localparam V_TOTAL  = V_ACTIVE + V_FRONT + V_SYNC + V_BACK;  // 286

    // Clock divider - 27MHz / 2 = 13.5MHz pixel clock
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
        if (!rst_n)
            h_count <= 10'd0;
        else if (pclk_div) begin
            if (h_count >= H_TOTAL - 1)
                h_count <= 10'd0;
            else
                h_count <= h_count + 10'd1;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            v_count <= 10'd0;
        else if (pclk_div && (h_count == H_TOTAL - 1)) begin
            if (v_count >= V_TOTAL - 1)
                v_count <= 10'd0;
            else
                v_count <= v_count + 10'd1;
        end
    end

    // Sync signals
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            lcd_hsync <= 1'b0;
            lcd_vsync <= 1'b0;
        end
        else begin
            lcd_hsync <= (h_count >= H_ACTIVE + H_FRONT) &&
                         (h_count < H_ACTIVE + H_FRONT + H_SYNC);

            lcd_vsync <= (v_count >= V_ACTIVE + V_FRONT) &&
                         (v_count < V_ACTIVE + V_FRONT + V_SYNC);
        end
    end

    // Data enable
    wire visible = (h_count < H_ACTIVE) && (v_count < V_ACTIVE);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            lcd_de <= 1'b0;
        else
            lcd_de <= visible;
    end

    // BRAM address - prefetch one pixel ahead to compensate for 1-cycle read latency.
    // BSRAM is synchronous: addr presented on cycle N returns data on cycle N+1.
    // Starting at 1 means the first pixel clock reads addr 0 data correctly.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            bram_addr <= 15'd1;
        else if (v_count == 0 && h_count == 0)
            bram_addr <= 15'd1;
        else if (pclk_div && visible)
            bram_addr <= bram_addr + 15'd1;
    end

    // =========================================================
    // Button debounce and edge detect
    // =========================================================
    reg [19:0] debounce_cnt;
    reg btn_sync1, btn_sync2, btn_stable, btn_prev;
    wire btn_pressed;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            btn_sync1 <= 1'b1;
            btn_sync2 <= 1'b1;
            btn_stable <= 1'b1;
            btn_prev <= 1'b1;
            debounce_cnt <= 20'd0;
        end
        else begin
            // Sync button input
            btn_sync1 <= btn;
            btn_sync2 <= btn_sync1;

            // Debounce (~20ms at 27MHz)
            if (btn_sync2 != btn_stable) begin
                debounce_cnt <= debounce_cnt + 1'b1;
                if (debounce_cnt >= 20'd540000) begin
                    btn_stable <= btn_sync2;
                    debounce_cnt <= 20'd0;
                end
            end
            else begin
                debounce_cnt <= 20'd0;
            end

            btn_prev <= btn_stable;
        end
    end

    // Detect falling edge (button press, active low)
    assign btn_pressed = btn_prev && !btn_stable;

    // =========================================================
    // Pattern selector (cycles 0-7 on button press)
    // =========================================================
    reg [2:0] pattern_sel;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            pattern_sel <= 3'd7;   // Boot directly into BRAM display mode
        else if (btn_pressed)
            pattern_sel <= pattern_sel + 1'b1;
    end

    // =========================================================
    // Test patterns
    // =========================================================
    wire [2:0] bar_num = h_count[6:4];  // Vertical bars

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            lcd_r <= 5'd0;
            lcd_g <= 6'd0;
            lcd_b <= 5'd0;
        end
        else if (visible) begin
            case (pattern_sel)
                3'd0: begin  // Solid RED
                    lcd_r <= 5'd31;
                    lcd_g <= 6'd0;
                    lcd_b <= 5'd0;
                end

                3'd1: begin  // Solid GREEN
                    lcd_r <= 5'd0;
                    lcd_g <= 6'd63;
                    lcd_b <= 5'd0;
                end

                3'd2: begin  // Solid BLUE
                    lcd_r <= 5'd0;
                    lcd_g <= 6'd0;
                    lcd_b <= 5'd31;
                end

                3'd3: begin  // Solid WHITE
                    lcd_r <= 5'd31;
                    lcd_g <= 6'd63;
                    lcd_b <= 5'd31;
                end

                3'd4: begin  // Vertical color bars
                    case (bar_num)
                        3'd0: begin lcd_r <= 5'd31; lcd_g <= 6'd0;  lcd_b <= 5'd0;  end
                        3'd1: begin lcd_r <= 5'd0;  lcd_g <= 6'd63; lcd_b <= 5'd0;  end
                        3'd2: begin lcd_r <= 5'd0;  lcd_g <= 6'd0;  lcd_b <= 5'd31; end
                        3'd3: begin lcd_r <= 5'd31; lcd_g <= 6'd63; lcd_b <= 5'd0;  end
                        3'd4: begin lcd_r <= 5'd31; lcd_g <= 6'd0;  lcd_b <= 5'd31; end
                        3'd5: begin lcd_r <= 5'd0;  lcd_g <= 6'd63; lcd_b <= 5'd31; end
                        3'd6: begin lcd_r <= 5'd31; lcd_g <= 6'd63; lcd_b <= 5'd31; end
                        3'd7: begin lcd_r <= 5'd0;  lcd_g <= 6'd0;  lcd_b <= 5'd0;  end
                    endcase
                end

                3'd5: begin  // Horizontal gradient (red)
                    lcd_r <= h_count[8:4];
                    lcd_g <= 6'd0;
                    lcd_b <= 5'd0;
                end

                3'd6: begin  // Checkerboard
                    if (h_count[5] ^ v_count[5]) begin
                        lcd_r <= 5'd31;
                        lcd_g <= 6'd63;
                        lcd_b <= 5'd31;
                    end
                    else begin
                        lcd_r <= 5'd0;
                        lcd_g <= 6'd0;
                        lcd_b <= 5'd0;
                    end
                end

                3'd7: begin  // Use BRAM data (for future SPI test)
                    lcd_r <= bram_data[7:3];
                    lcd_g <= bram_data[7:2];
                    lcd_b <= bram_data[7:3];
                end
            endcase
        end
        else begin
            lcd_r <= 5'd0;
            lcd_g <= 6'd0;
            lcd_b <= 5'd0;
        end
    end

endmodule
