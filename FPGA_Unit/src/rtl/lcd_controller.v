//
// lcd_controller.v
// RGB LCD controller — 480x272 @ 13.5MHz (27MHz / 2)
//
// Style: if(!rst_n) for async reset (yosys requires this pattern),
//        AND-mask / OR-combine / XOR everywhere else. No if/else in datapath.
//

module lcd_controller (
    input  wire        clk,       // 27MHz
    input  wire        rst_n,
    input  wire        btn,

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

    localparam H_ACTIVE = 480, H_FRONT = 2, H_SYNC = 41, H_TOTAL = 525;
    localparam V_ACTIVE = 272, V_FRONT = 2, V_SYNC = 10, V_TOTAL = 286;
    // ==========================================================
    // Pixel clock: toggle → 13.5 MHz
    // ==========================================================
    reg pclk;
    always @(posedge clk or negedge rst_n)
        if (!rst_n) pclk <= 1'b0;
        else        pclk <= ~pclk;

    assign lcd_clk = pclk;

    // ==========================================================
    // H/V counters
    //   (count + 1) & {N{~last}}  →  wraps to 0 with no branch
    // ==========================================================
    reg  [9:0] h_count, v_count;
    wire h_last = (h_count == H_TOTAL - 1);
    wire v_last = (v_count == V_TOTAL - 1);

    wire [9:0] h_next = (h_count + 10'd1) & {10{~h_last}};
    wire [9:0] v_next = (v_count + 10'd1) & {10{~v_last}};

    // pclk gates h; pclk & h_last gates v
    // hold = current & mask; load = next & ~mask; result = hold | load
    wire [9:0] h_load = (h_next & {10{pclk}}) | (h_count & {10{~pclk}});
    wire       v_en   = pclk & h_last;
    wire [9:0] v_load = (v_next & {10{v_en}}) | (v_count & {10{~v_en}});

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            h_count <= 10'd0;
            v_count <= 10'd0;
        end
        else begin
            h_count <= h_load;
            v_count <= v_load;
        end
    end

    // ==========================================================
    // Derived signals — pure combinational
    // ==========================================================
    wire visible     = (h_count < H_ACTIVE) & (v_count < V_ACTIVE);
    wire frame_start = ~|h_count & ~|v_count;
    wire h_sync      = (h_count >= H_ACTIVE + H_FRONT) &
                       (h_count <  H_ACTIVE + H_FRONT + H_SYNC);
    wire v_sync      = (v_count >= V_ACTIVE + V_FRONT) &
                       (v_count <  V_ACTIVE + V_FRONT + V_SYNC);

    // ==========================================================
    // Register sync/DE — AND-mask select: pclk→new, ~pclk→hold
    // ==========================================================
    wire next_hsync = (h_sync  & pclk) | (lcd_hsync & ~pclk);
    wire next_vsync = (v_sync  & pclk) | (lcd_vsync & ~pclk);
    wire next_de    = (visible & pclk) | (lcd_de    & ~pclk);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            lcd_hsync <= 1'b0;
            lcd_vsync <= 1'b0;
            lcd_de    <= 1'b0;
        end
        else begin
            lcd_hsync <= next_hsync;
            lcd_vsync <= next_vsync;
            lcd_de    <= next_de;
        end
    end

    // ==========================================================
    // BRAM address
    //   +visible increments only in active region
    //   & ~frame_start zeros at frame boundary
    //   & pclk / ~pclk selects new vs hold
    // ==========================================================
    wire [14:0] addr_inc  = (bram_addr + {14'd0, visible}) & {15{~frame_start}};
    wire [14:0] next_addr = (addr_inc  & {15{pclk}}) | (bram_addr & {15{~pclk}});

    always @(posedge clk or negedge rst_n)
        if (!rst_n) bram_addr <= 15'd0;
        else        bram_addr <= next_addr;

    // ==========================================================
    // Button debounce
    //   XOR detects change. AND-mask zeros counter on match.
    //   Stable latch via SDD501 XOR trick.
    // ==========================================================
    reg [19:0] db_cnt;
    reg btn_s1, btn_s2, btn_stable, btn_prev;

    wire btn_changed = btn_s2 ^ btn_stable;
    wire db_done     = (db_cnt >= 20'd540000);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            btn_s1     <= 1'b1;
            btn_s2     <= 1'b1;
            btn_stable <= 1'b1;
            btn_prev   <= 1'b1;
            db_cnt     <= 20'd0;
        end
        else begin
            btn_s1     <= btn;
            btn_s2     <= btn_s1;
            btn_prev   <= btn_stable;
            db_cnt     <= (db_cnt + {19'd0, btn_changed}) & {20{btn_changed}};
            btn_stable <= btn_stable ^ ((btn_stable ^ btn_s2) & db_done);
        end
    end

    wire btn_pressed = btn_prev & ~btn_stable;

    // ==========================================================
    // Pattern selector — increment masked by btn_pressed
    // ==========================================================
    reg [2:0] pat;
    always @(posedge clk or negedge rst_n)
        if (!rst_n) pat <= 3'd0;
        else        pat <= pat + {2'd0, btn_pressed};

    // ==========================================================
    // Pixel output — fully branchless
    //
    // All 8 patterns computed as parallel wires.
    // AND-mask with one-hot select, OR-combine.
    // AND-mask with visibility. AND-mask with pclk for hold.
    // ==========================================================

    // one-hot decode
    wire [7:0] sel;
    assign sel[0] = (pat == 3'd0);
    assign sel[1] = (pat == 3'd1);
    assign sel[2] = (pat == 3'd2);
    assign sel[3] = (pat == 3'd3);
    assign sel[4] = (pat == 3'd4);
    assign sel[5] = (pat == 3'd5);
    assign sel[6] = (pat == 3'd6);
    assign sel[7] = (pat == 3'd7);

    // visibility AND masks
    wire [4:0] vis5 = {5{visible}};
    wire [5:0] vis6 = {6{visible}};

    // --- p0: solid red ---
    wire [4:0] p0_r = {5{sel[0]}};
    wire [5:0] p0_g = 6'd0;
    wire [4:0] p0_b = 5'd0;

    // --- p1: solid green ---
    wire [4:0] p1_r = 5'd0;
    wire [5:0] p1_g = {6{sel[1]}};
    wire [4:0] p1_b = 5'd0;

    // --- p2: solid blue ---
    wire [4:0] p2_r = 5'd0;
    wire [5:0] p2_g = 6'd0;
    wire [4:0] p2_b = {5{sel[2]}};

    // --- p3: solid white ---
    wire [4:0] p3_r = {5{sel[3]}};
    wire [5:0] p3_g = {6{sel[3]}};
    wire [4:0] p3_b = {5{sel[3]}};

    // --- p4: vertical color bars ---
    wire [2:0] bar = h_count[6:4];
    wire bar_r = (bar == 3'd0) | (bar == 3'd3) | (bar == 3'd4) | (bar == 3'd6);
    wire bar_g = (bar == 3'd1) | (bar == 3'd3) | (bar == 3'd5) | (bar == 3'd6);
    wire bar_b = (bar == 3'd2) | (bar == 3'd4) | (bar == 3'd5) | (bar == 3'd6);

    wire [4:0] p4_r = {5{bar_r & sel[4]}};
    wire [5:0] p4_g = {6{bar_g & sel[4]}};
    wire [4:0] p4_b = {5{bar_b & sel[4]}};

    // --- p5: horizontal red gradient ---
    wire [4:0] p5_r = h_count[8:4] & {5{sel[5]}};
    wire [5:0] p5_g = 6'd0;
    wire [4:0] p5_b = 5'd0;

    // --- p6: checkerboard ---
    wire chk = h_count[5] ^ v_count[5];
    wire [4:0] p6_r = {5{chk & sel[6]}};
    wire [5:0] p6_g = {6{chk & sel[6]}};
    wire [4:0] p6_b = {5{chk & sel[6]}};

    // --- p7: BRAM grayscale ---
    wire [4:0] p7_r = bram_data[7:3] & {5{sel[7]}};
    wire [5:0] p7_g = bram_data[7:2] & {6{sel[7]}};
    wire [4:0] p7_b = bram_data[7:3] & {5{sel[7]}};

    // --- OR-combine all patterns, AND with visibility ---
    wire [4:0] raw_r = (p0_r | p1_r | p2_r | p3_r | p4_r | p5_r | p6_r | p7_r) & vis5;
    wire [5:0] raw_g = (p0_g | p1_g | p2_g | p3_g | p4_g | p5_g | p6_g | p7_g) & vis6;
    wire [4:0] raw_b = (p0_b | p1_b | p2_b | p3_b | p4_b | p5_b | p6_b | p7_b) & vis5;

    // --- pclk AND-mask: new on pclk, hold on ~pclk ---
    wire [4:0] next_r = (raw_r & {5{pclk}}) | (lcd_r & {5{~pclk}});
    wire [5:0] next_g = (raw_g & {6{pclk}}) | (lcd_g & {6{~pclk}});
    wire [4:0] next_b = (raw_b & {5{pclk}}) | (lcd_b & {5{~pclk}});

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            lcd_r <= 5'd0;
            lcd_g <= 6'd0;
            lcd_b <= 5'd0;
        end
        else begin
            lcd_r <= next_r;
            lcd_g <= next_g;
            lcd_b <= next_b;
        end
    end

endmodule
