/*
 * coeff.v -- Top-K coefficient extraction FSM
 *
 * Ports flattened for yosys: element [r][c] = flat[(r*8+c)*W +: W]
 * Output arrays flattened: slot k = flat[k*N +: N]
 */

module coeff
    #(parameter W = 16, TOP_K = 5)
    (input wire clk,
     input wire rst,
     input wire bgn,

     input wire signed [W*64-1:0] c_i,

     output reg [3*TOP_K-1:0] r_o,
     output reg [3*TOP_K-1:0] c_o,
     output reg signed [W*TOP_K-1:0] q_o,
     output reg [2:0] n_out,
     output reg done);

        localparam [W-1:0] thresh = 16'd400;
        localparam [3:0] q_shift = 4'd3;
        localparam [3:0] seq_thresh = 4'd6;

        reg [1:0] state;
        localparam IDLE = 2'd0, SCAN = 2'd1, DONE = 2'd2;

        reg [2:0] r_cnt;
        reg [2:0] c_cnt;

        wire skp = (r_cnt + c_cnt > seq_thresh) | (~|{r_cnt, c_cnt});

        wire signed [W-1:0] coeff_val = c_i[(r_cnt*8+c_cnt)*W +: W];
        wire [W-1:0] mag = (coeff_val ^ {W{coeff_val[W-1]}}) + coeff_val[W-1];
        wire [W-1:0] sft = mag - thresh;
        wire [W-1:0] q = sft >>> q_shift;
        wire signed [W-1:0] q_sgn = (q ^ {W{coeff_val[W-1]}}) + coeff_val[W-1];

        wire vld = ~skp & (mag > thresh) & (q != 0);

        wire state_scan = (state == SCAN);
        wire state_idle = (state == IDLE);

        wire wrt_en = state_scan & vld;
        wire clr = state_idle & bgn;

        wire fill = wrt_en & (n_out < TOP_K);
        wire repl = wrt_en & (n_out == TOP_K) & (mag > slot_abs[min_idx]);

        reg [2:0] slot_r [0:TOP_K-1];
        reg [2:0] slot_c [0:TOP_K-1];
        reg signed [W-1:0] slot_q [0:TOP_K-1];
        reg [W-1:0] slot_abs [0:TOP_K-1];
        reg [2:0] min_idx;

        // min rescan: find slot with smallest abs_q
        wire [2:0] m01  = slot_abs[0] < slot_abs[1] ? 0 : 1;
        wire [2:0] m23  = slot_abs[2] < slot_abs[3] ? 2 : 3;
        wire [2:0] m013 = slot_abs[m01] < slot_abs[m23] ? m01 : m23;
        wire [2:0] nmin = slot_abs[m013] < slot_abs[4] ? m013 : 4;

        integer k;

        always @(posedge clk) begin
                done <= (state == DONE);

                if (rst) begin
                        state <= IDLE;
                        n_out <= 3'd0;
                        r_cnt <= 3'd0;
                        c_cnt <= 3'd0;
                end else if (clr) begin
                        state <= SCAN;
                        r_cnt <= 3'd0;
                        c_cnt <= 3'd0;
                        n_out <= 3'd0;
                end else begin
                        c_cnt <= c_cnt + state_scan;
                        r_cnt <= r_cnt + (state_scan & c_cnt == 7);

                        if (state_scan & r_cnt == 7 & c_cnt == 7)
                                state <= DONE;

                        if (state == DONE)
                                state <= IDLE;

                        slot_r[n_out]   <= fill ? r_cnt : slot_r[n_out];
                        slot_c[n_out]   <= fill ? c_cnt : slot_c[n_out];
                        slot_q[n_out]   <= fill ? q_sgn : slot_q[n_out];
                        slot_abs[n_out] <= fill ? mag : slot_abs[n_out];
                        n_out           <= fill ? n_out + 1 : n_out;

                        slot_r[min_idx]   <= repl ? r_cnt : slot_r[min_idx];
                        slot_c[min_idx]   <= repl ? c_cnt : slot_c[min_idx];
                        slot_q[min_idx]   <= repl ? q_sgn : slot_q[min_idx];
                        slot_abs[min_idx] <= repl ? mag : slot_abs[min_idx];

                        min_idx <= (fill | repl) ? nmin : min_idx;
                end

                if (state == DONE) begin
                        for (k = 0; k < TOP_K; k = k + 1) begin
                                r_o[k*3 +: 3] <= slot_r[k];
                                c_o[k*3 +: 3] <= slot_c[k];
                                q_o[k*W +: W] <= slot_q[k];
                        end
                end
        end
endmodule
