/*
 * blk_schd.v -- Block scheduler FSM
 *
 * Iterates 40x30 blocks over a 320x240 frame pair in PSRAM.
 * Per block: LOAD current + previous pixels, diff, WHT, coeff, EMIT.
 * Two PSRAM regions: SPI writes to cur_base, scheduler reads both.
 * top.v swaps cur_base/prev_base each frame.
 *
 * Flat bus convention: element [r][c] = flat[(r*8+c)*W +: W]
 */

module block_sched
    #(parameter W = 16, TOP_K = 5)
    (input wire clk,
     input wire rst_n,
     input wire start,
     output reg frame_done,

     output reg psram_read,
     output reg [21:0] psram_addr,
     input wire [15:0] psram_rdata,
     input wire psram_busy,

     output reg tx_valid,
     output reg [7:0] tx_data,
     input wire tx_ready,

     input wire [21:0] cur_base,
     input wire [21:0] prev_base);

        // ==========================================================
        // FSM states
        // ==========================================================
        localparam [2:0] IDLE = 3'd0,
                         LOAD = 3'd1,
                         WHT  = 3'd2,
                         WAIT = 3'd3,
                         EMIT = 3'd4,
                         NEXT = 3'd5;
        reg [2:0] state;

        // ==========================================================
        // Block counters
        // ==========================================================
        reg [5:0] bx;
        reg [4:0] by;

        // ==========================================================
        // LOAD counters + diff logic
        // ==========================================================
        reg [2:0] load_row;
        reg [1:0] load_col;
        reg load_pass;
        reg psram_pending;
        reg [15:0] cur_word;

        // ==========================================================
        // 8x8 pixel register array -- flat packed bus
        // ==========================================================
        reg signed [W*64-1:0] pixels;

        // ==========================================================
        // Address calculation
        // ==========================================================
        wire [21:0] by_offset  = ({17'b0, by} << 11) + ({17'b0, by} << 9);
        wire [21:0] bx_offset  = {16'b0, bx} << 3;
        wire [21:0] row_offset = ({19'b0, load_row} << 8)
                               + ({19'b0, load_row} << 6);
        wire [21:0] col_offset = {20'b0, load_col, 1'b0};
        wire [21:0] block_offset = by_offset + bx_offset
                                 + row_offset + col_offset;
        wire [21:0] cur_pixel_addr  = cur_base  + block_offset;
        wire [21:0] prev_pixel_addr = prev_base + block_offset;

        wire signed [W-1:0] diff_lo = {{(W-8){1'b0}}, cur_word[7:0]}
                                    - {{(W-8){1'b0}}, psram_rdata[7:0]};
        wire signed [W-1:0] diff_hi = {{(W-8){1'b0}}, cur_word[15:8]}
                                    - {{(W-8){1'b0}}, psram_rdata[15:8]};

        wire [4:0] pix_pair = {load_row, load_col};

        // ==========================================================
        // WHT -- single wht8, 4-bit pass counter
        //   pass 0-7: rows 0-7     pass 8-15: cols 0-7
        // ==========================================================
        reg [3:0] wht_pass;
        reg signed [W*8-1:0] wht_in;
        wire signed [W*8-1:0] wht_out;

        integer m;
        always @(*) begin
                case (wht_pass)
                4'd0:  for(m=0;m<8;m=m+1) wht_in[m*W+:W] = pixels[(0*8+m)*W+:W];
                4'd1:  for(m=0;m<8;m=m+1) wht_in[m*W+:W] = pixels[(1*8+m)*W+:W];
                4'd2:  for(m=0;m<8;m=m+1) wht_in[m*W+:W] = pixels[(2*8+m)*W+:W];
                4'd3:  for(m=0;m<8;m=m+1) wht_in[m*W+:W] = pixels[(3*8+m)*W+:W];
                4'd4:  for(m=0;m<8;m=m+1) wht_in[m*W+:W] = pixels[(4*8+m)*W+:W];
                4'd5:  for(m=0;m<8;m=m+1) wht_in[m*W+:W] = pixels[(5*8+m)*W+:W];
                4'd6:  for(m=0;m<8;m=m+1) wht_in[m*W+:W] = pixels[(6*8+m)*W+:W];
                4'd7:  for(m=0;m<8;m=m+1) wht_in[m*W+:W] = pixels[(7*8+m)*W+:W];
                4'd8:  for(m=0;m<8;m=m+1) wht_in[m*W+:W] = pixels[(m*8+0)*W+:W];
                4'd9:  for(m=0;m<8;m=m+1) wht_in[m*W+:W] = pixels[(m*8+1)*W+:W];
                4'd10: for(m=0;m<8;m=m+1) wht_in[m*W+:W] = pixels[(m*8+2)*W+:W];
                4'd11: for(m=0;m<8;m=m+1) wht_in[m*W+:W] = pixels[(m*8+3)*W+:W];
                4'd12: for(m=0;m<8;m=m+1) wht_in[m*W+:W] = pixels[(m*8+4)*W+:W];
                4'd13: for(m=0;m<8;m=m+1) wht_in[m*W+:W] = pixels[(m*8+5)*W+:W];
                4'd14: for(m=0;m<8;m=m+1) wht_in[m*W+:W] = pixels[(m*8+6)*W+:W];
                4'd15: for(m=0;m<8;m=m+1) wht_in[m*W+:W] = pixels[(m*8+7)*W+:W];
                endcase
        end

        wht2d #(.W(W)) wht_inst(
            .x(wht_in),
            .y(wht_out)
        );

        // ==========================================================
        // Sequency mask (combinational, reads pixels after WHT done)
        // ==========================================================
        wire signed [W*64-1:0] masked;

        seq_msk #(.W(W)) msk_inst(
            .c_i(pixels),
            .c_o(masked)
        );

        // ==========================================================
        // Coeff extractor
        // ==========================================================
        reg coeff_bgn;
        wire coeff_done;
        wire [3*TOP_K-1:0] coeff_r;
        wire [3*TOP_K-1:0] coeff_c;
        wire signed [W*TOP_K-1:0] coeff_q;
        wire [2:0] coeff_n;

        coeff #(.W(W), .TOP_K(TOP_K)) coeff_inst(
            .clk(clk),
            .rst(~rst_n),
            .bgn(coeff_bgn),
            .c_i(masked),
            .r_o(coeff_r),
            .c_o(coeff_c),
            .q_o(coeff_q),
            .n_out(coeff_n),
            .done(coeff_done)
        );

        // ==========================================================
        // EMIT sub-counter
        // ==========================================================
        reg [2:0] emit_idx;
        reg [4:0] emit_byte;
        reg [1:0] emit_sub;
        reg emit_active;

        wire [7:0] header_hi = {2'b0, by, bx[5]};
        wire [7:0] header_lo = {bx[4:0], coeff_n};

        wire [2:0] ei = emit_idx;
        wire [7:0] rc_byte = {2'b0, coeff_r[ei*3 +: 3], coeff_c[ei*3 +: 3]};
        wire signed [W-1:0] emit_q = coeff_q[ei*W +: W];
        wire [7:0] q_hi = emit_q[W-1:8];
        wire [7:0] q_lo = emit_q[7:0];

        // ==========================================================
        // Main FSM
        // ==========================================================
        integer j;

        always @(posedge clk or negedge rst_n) begin
                if (!rst_n) begin
                        state <= IDLE;
                        bx <= 6'd0;
                        by <= 5'd0;
                        load_row <= 3'd0;
                        load_col <= 2'd0;
                        load_pass <= 1'b0;
                        psram_read <= 1'b0;
                        psram_addr <= 22'd0;
                        psram_pending <= 1'b0;
                        cur_word <= 16'd0;
                        coeff_bgn <= 1'b0;
                        wht_pass <= 4'd0;
                        tx_valid <= 1'b0;
                        tx_data <= 8'd0;
                        frame_done <= 1'b0;
                        emit_idx <= 3'd0;
                        emit_byte <= 4'd0;
                        emit_sub <= 2'd0;
                        emit_active <= 1'b0;
                        pixels <= {(W*64){1'b0}};
                end else begin

                psram_read <= 1'b0;
                coeff_bgn <= 1'b0;
                tx_valid <= 1'b0;
                frame_done <= 1'b0;

                case (state)
                // --------------------------------------------------
                IDLE: begin
                        bx <= 6'd0;
                        by <= 5'd0;
                        state <= start ? LOAD : IDLE;
                        load_pass <= 1'b0;
                end

                // --------------------------------------------------
                LOAD: begin
                        if (~psram_busy & ~psram_pending) begin
                                psram_addr <= load_pass ? prev_pixel_addr
                                                        : cur_pixel_addr;
                                psram_read <= 1'b1;
                                psram_pending <= 1'b1;
                        end

                        if (psram_pending & ~psram_busy) begin
                                psram_pending <= 1'b0;

                                if (~load_pass) begin
                                        cur_word <= psram_rdata;
                                        load_pass <= 1'b1;
                                end else begin
                                        case (pix_pair)
                                        5'd0:  begin pixels[ 0*W+:W]<=diff_lo; pixels[ 1*W+:W]<=diff_hi; end
                                        5'd1:  begin pixels[ 2*W+:W]<=diff_lo; pixels[ 3*W+:W]<=diff_hi; end
                                        5'd2:  begin pixels[ 4*W+:W]<=diff_lo; pixels[ 5*W+:W]<=diff_hi; end
                                        5'd3:  begin pixels[ 6*W+:W]<=diff_lo; pixels[ 7*W+:W]<=diff_hi; end
                                        5'd4:  begin pixels[ 8*W+:W]<=diff_lo; pixels[ 9*W+:W]<=diff_hi; end
                                        5'd5:  begin pixels[10*W+:W]<=diff_lo; pixels[11*W+:W]<=diff_hi; end
                                        5'd6:  begin pixels[12*W+:W]<=diff_lo; pixels[13*W+:W]<=diff_hi; end
                                        5'd7:  begin pixels[14*W+:W]<=diff_lo; pixels[15*W+:W]<=diff_hi; end
                                        5'd8:  begin pixels[16*W+:W]<=diff_lo; pixels[17*W+:W]<=diff_hi; end
                                        5'd9:  begin pixels[18*W+:W]<=diff_lo; pixels[19*W+:W]<=diff_hi; end
                                        5'd10: begin pixels[20*W+:W]<=diff_lo; pixels[21*W+:W]<=diff_hi; end
                                        5'd11: begin pixels[22*W+:W]<=diff_lo; pixels[23*W+:W]<=diff_hi; end
                                        5'd12: begin pixels[24*W+:W]<=diff_lo; pixels[25*W+:W]<=diff_hi; end
                                        5'd13: begin pixels[26*W+:W]<=diff_lo; pixels[27*W+:W]<=diff_hi; end
                                        5'd14: begin pixels[28*W+:W]<=diff_lo; pixels[29*W+:W]<=diff_hi; end
                                        5'd15: begin pixels[30*W+:W]<=diff_lo; pixels[31*W+:W]<=diff_hi; end
                                        5'd16: begin pixels[32*W+:W]<=diff_lo; pixels[33*W+:W]<=diff_hi; end
                                        5'd17: begin pixels[34*W+:W]<=diff_lo; pixels[35*W+:W]<=diff_hi; end
                                        5'd18: begin pixels[36*W+:W]<=diff_lo; pixels[37*W+:W]<=diff_hi; end
                                        5'd19: begin pixels[38*W+:W]<=diff_lo; pixels[39*W+:W]<=diff_hi; end
                                        5'd20: begin pixels[40*W+:W]<=diff_lo; pixels[41*W+:W]<=diff_hi; end
                                        5'd21: begin pixels[42*W+:W]<=diff_lo; pixels[43*W+:W]<=diff_hi; end
                                        5'd22: begin pixels[44*W+:W]<=diff_lo; pixels[45*W+:W]<=diff_hi; end
                                        5'd23: begin pixels[46*W+:W]<=diff_lo; pixels[47*W+:W]<=diff_hi; end
                                        5'd24: begin pixels[48*W+:W]<=diff_lo; pixels[49*W+:W]<=diff_hi; end
                                        5'd25: begin pixels[50*W+:W]<=diff_lo; pixels[51*W+:W]<=diff_hi; end
                                        5'd26: begin pixels[52*W+:W]<=diff_lo; pixels[53*W+:W]<=diff_hi; end
                                        5'd27: begin pixels[54*W+:W]<=diff_lo; pixels[55*W+:W]<=diff_hi; end
                                        5'd28: begin pixels[56*W+:W]<=diff_lo; pixels[57*W+:W]<=diff_hi; end
                                        5'd29: begin pixels[58*W+:W]<=diff_lo; pixels[59*W+:W]<=diff_hi; end
                                        5'd30: begin pixels[60*W+:W]<=diff_lo; pixels[61*W+:W]<=diff_hi; end
                                        5'd31: begin pixels[62*W+:W]<=diff_lo; pixels[63*W+:W]<=diff_hi; end
                                        endcase
                                        load_pass <= 1'b0;

                                        if (load_col == 2'd3) begin
                                                load_col <= 2'd0;
                                                if (load_row == 3'd7) begin
                                                        load_row <= 3'd0;
                                                        wht_pass <= 4'd0;
                                                        state <= WHT;
                                                end else begin
                                                        load_row <= load_row + 3'd1;
                                                end
                                        end else begin
                                                load_col <= load_col + 2'd1;
                                        end
                                end
                        end
                end

                // --------------------------------------------------
                WHT: begin
                        case (wht_pass)
                        4'd0:  for(j=0;j<8;j=j+1) pixels[(0*8+j)*W+:W] <= wht_out[j*W+:W];
                        4'd1:  for(j=0;j<8;j=j+1) pixels[(1*8+j)*W+:W] <= wht_out[j*W+:W];
                        4'd2:  for(j=0;j<8;j=j+1) pixels[(2*8+j)*W+:W] <= wht_out[j*W+:W];
                        4'd3:  for(j=0;j<8;j=j+1) pixels[(3*8+j)*W+:W] <= wht_out[j*W+:W];
                        4'd4:  for(j=0;j<8;j=j+1) pixels[(4*8+j)*W+:W] <= wht_out[j*W+:W];
                        4'd5:  for(j=0;j<8;j=j+1) pixels[(5*8+j)*W+:W] <= wht_out[j*W+:W];
                        4'd6:  for(j=0;j<8;j=j+1) pixels[(6*8+j)*W+:W] <= wht_out[j*W+:W];
                        4'd7:  for(j=0;j<8;j=j+1) pixels[(7*8+j)*W+:W] <= wht_out[j*W+:W];
                        4'd8:  for(j=0;j<8;j=j+1) pixels[(j*8+0)*W+:W] <= wht_out[j*W+:W];
                        4'd9:  for(j=0;j<8;j=j+1) pixels[(j*8+1)*W+:W] <= wht_out[j*W+:W];
                        4'd10: for(j=0;j<8;j=j+1) pixels[(j*8+2)*W+:W] <= wht_out[j*W+:W];
                        4'd11: for(j=0;j<8;j=j+1) pixels[(j*8+3)*W+:W] <= wht_out[j*W+:W];
                        4'd12: for(j=0;j<8;j=j+1) pixels[(j*8+4)*W+:W] <= wht_out[j*W+:W];
                        4'd13: for(j=0;j<8;j=j+1) pixels[(j*8+5)*W+:W] <= wht_out[j*W+:W];
                        4'd14: for(j=0;j<8;j=j+1) pixels[(j*8+6)*W+:W] <= wht_out[j*W+:W];
                        4'd15: for(j=0;j<8;j=j+1) pixels[(j*8+7)*W+:W] <= wht_out[j*W+:W];
                        endcase

                        if (wht_pass == 4'd15) begin
                                coeff_bgn <= 1'b1;
                                state <= WAIT;
                        end else begin
                                wht_pass <= wht_pass + 4'd1;
                        end
                end

                // --------------------------------------------------
                WAIT: begin
                        if (coeff_done) begin
                                if (coeff_n == 0) begin
                                        state <= NEXT;
                                end else begin
                                        emit_byte <= 4'd0;
                                        emit_idx <= 3'd0;
                                        emit_sub <= 2'd0;
                                        emit_active <= 1'b1;
                                        state <= EMIT;
                                end
                        end
                end

                // --------------------------------------------------
                EMIT: begin
                        if (tx_ready & emit_active) begin
                                tx_valid <= 1'b1;

                                case (emit_byte)
                                4'd0: tx_data <= header_hi;
                                4'd1: tx_data <= header_lo;
                                default: begin
                                        case (emit_sub)
                                        2'd0: tx_data <= rc_byte;
                                        2'd1: tx_data <= q_hi;
                                        2'd2: begin
                                                tx_data <= q_lo;
                                                emit_idx <= emit_idx + 3'd1;
                                        end
                                        default: tx_data <= 8'd0;
                                        endcase
                                        emit_sub <= (emit_sub == 2'd2) ? 2'd0
                                                                       : emit_sub + 2'd1;
                                end
                                endcase

                                emit_byte <= emit_byte + 4'd1;

                                if (emit_byte == 4'd1 + {1'b0, coeff_n} * 3) begin
                                        emit_active <= 1'b0;
                                        state <= NEXT;
                                end
                        end
                end

                // --------------------------------------------------
                NEXT: begin
                        if (bx == 6'd39) begin
                                bx <= 6'd0;
                                if (by == 5'd29) begin
                                        frame_done <= 1'b1;
                                        state <= IDLE;
                                end else begin
                                        by <= by + 5'd1;
                                        state <= LOAD;
                                end
                        end else begin
                                bx <= bx + 6'd1;
                                state <= LOAD;
                        end
                end

                default: state <= IDLE;
                endcase
                end
        end
endmodule
