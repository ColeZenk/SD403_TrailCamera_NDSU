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
     input wire [21:0] prev_base,
     input wire [W-1:0] thresh,
     input wire [3:0] q_shift,
     input wire [3:0] seq_thresh);

        // ==========================================================
        // FSM states
        // ==========================================================
        localparam [2:0] IDLE  = 3'd0,
                         LOAD  = 3'd1,
                         WAIT  = 3'd2,
                         RUN   = 3'd3,
                         EMIT  = 3'd4,
                         NEXT  = 3'd5;
        reg [2:0] state;

        // ==========================================================
        // Block counters
        // ==========================================================
        reg [5:0] bx;
        reg [4:0] by;

        // ==========================================================
        // LOAD counters + diff logic
        //   Two-pass per word position: read current, read previous, diff.
        //   load_pass: 0 = read current, 1 = read previous + compute diff
        // ==========================================================
        reg [2:0] load_row;
        reg [1:0] load_col;
        reg load_pass;
        reg psram_pending;
        reg [15:0] cur_word;

        // ==========================================================
        // 8x8 diff register array -- flat packed bus
        // ==========================================================
        reg signed [W*64-1:0] pixels;

        // ==========================================================
        // Address calculation
        //   by * 2560 = (by << 11) + (by << 9)
        //   bx * 8    = bx << 3
        //   load_row * 320 = (load_row << 8) + (load_row << 6)
        //   load_col * 2
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

        // Diff: current - previous (sign-extended to W bits)
        wire signed [W-1:0] diff_lo = {{(W-8){cur_word[7]}},  cur_word[7:0]}
                                    - {{(W-8){psram_rdata[7]}},  psram_rdata[7:0]};
        wire signed [W-1:0] diff_hi = {{(W-8){cur_word[15]}}, cur_word[15:8]}
                                    - {{(W-8){psram_rdata[15]}}, psram_rdata[15:8]};

        // ==========================================================
        // Combinational chain: wht2d -> seq_msk
        // ==========================================================
        wire signed [W*64-1:0] wht_out;
        wire signed [W*64-1:0] masked;

        wht2d #(.W(W)) wht_inst(
            .b_i(pixels),
            .b_o(wht_out)
        );

        seq_msk #(.W(W)) msk_inst(
            .T(seq_thresh),
            .c_i(wht_out),
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
            .thresh(thresh),
            .q_shift(q_shift),
            .seq_thresh(seq_thresh),
            .r_o(coeff_r),
            .c_o(coeff_c),
            .q_o(coeff_q),
            .n_out(coeff_n),
            .done(coeff_done)
        );

        // ==========================================================
        // EMIT sub-counter
        //   Packet: [header_hi][header_lo] then per coeff [rc][q_hi][q_lo]
        //   header_hi = {2'b0, by[4:0], bx[5]}
        //   header_lo = {bx[4:0], coeff_n[2:0]}
        //   rc        = {0, 0, r[2:0], c[2:0]}
        //   q_hi      = q[W-1:8]
        //   q_lo      = q[7:0]
        // ==========================================================
        reg [2:0] emit_idx;
        reg [3:0] emit_byte;
        reg emit_active;

        wire [7:0] header_hi = {2'b0, by, bx[5]};
        wire [7:0] header_lo = {bx[4:0], coeff_n};

        wire [2:0] ei = emit_idx;
        wire [7:0] rc_byte = {2'b0, coeff_r[ei*3 +: 3], coeff_c[ei*3 +: 3]};
        wire signed [W-1:0] emit_q = coeff_q[ei*W +: W];
        wire [7:0] q_hi = emit_q[W-1:8];
        wire [7:0] q_lo = emit_q[7:0];

        // pixel index for LOAD writes
        wire [5:0] pix_idx_lo = load_row * 8 + {load_col, 1'b0};
        wire [5:0] pix_idx_hi = load_row * 8 + {load_col, 1'b1};

        // ==========================================================
        // Main FSM
        // ==========================================================
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
                        tx_valid <= 1'b0;
                        tx_data <= 8'd0;
                        frame_done <= 1'b0;
                        emit_idx <= 3'd0;
                        emit_byte <= 4'd0;
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
                        state <= state | ({3{start}} & LOAD);
                        load_pass <= 1'b0;
                end

                // --------------------------------------------------
                // LOAD: two reads per word position
                //   pass 0: read current frame word, stash in cur_word
                //   pass 1: read previous frame word, compute diff, store
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
                                        // pass 0: stash current word
                                        cur_word <= psram_rdata;
                                        load_pass <= 1'b1;
                                end else begin
                                        // pass 1: diff and store
                                        pixels[pix_idx_lo*W +: W] <= diff_lo;
                                        pixels[pix_idx_hi*W +: W] <= diff_hi;
                                        load_pass <= 1'b0;

                                        // advance to next column pair
                                        if (load_col == 2'd3) begin
                                                load_col <= 2'd0;
                                                if (load_row == 3'd7) begin
                                                        load_row <= 3'd0;
                                                        state <= RUN;
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
                RUN: begin
                        coeff_bgn <= 1'b1;
                        state <= WAIT;
                end

                // --------------------------------------------------
                WAIT: begin
                        if (coeff_done) begin
                                emit_byte <= 4'd0;
                                emit_idx <= 3'd0;
                                emit_active <= 1'b1;
                                state <= EMIT;
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
                                        case ((emit_byte - 4'd2) % 3)
                                        0: tx_data <= rc_byte;
                                        1: tx_data <= q_hi;
                                        2: begin
                                                tx_data <= q_lo;
                                                emit_idx <= emit_idx + 3'd1;
                                        end
                                        default: tx_data <= 8'd0;
                                        endcase
                                end
                                endcase

                                emit_byte <= emit_byte + 4'd1;

                                if (emit_byte == 4'd1 + {1'b0, coeff_n} * 3) begin
                                        emit_active <= 1'b0;
                                        state <= NEXT;
                                end
                        end

                        if (emit_active & coeff_n == 0 & emit_byte == 4'd1) begin
                                emit_active <= 1'b0;
                                state <= NEXT;
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
