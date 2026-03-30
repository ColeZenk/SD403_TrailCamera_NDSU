/*
 * wht8.v — 8-point Walsh-Hadamard Transform (combinational)
 *
 * Three butterfly stages: stride=1, stride=2, stride=4.
 * Pure additions/subtractions — zero multipliers.
 *
 * Bit growth: 8-bit input → at most 11-bit output per 1D stage
 * (log2(8)=3 stages, each can double worst-case value).
 * For 2D WHT (6 stages total), 8-bit → 14-bit; W=16 is safe.
 *
 * Maps directly from the branchless FWHT class in wht_sim.py.
 * Stage ordering matches stride=1,2,4 loop in fwht_reference().
 */

module wht8 #(
    parameter W = 16
) (
    input  wire signed [W-1:0] x0, x1, x2, x3,
    input  wire signed [W-1:0] x4, x5, x6, x7,
    output wire signed [W-1:0] y0, y1, y2, y3,
    output wire signed [W-1:0] y4, y5, y6, y7
);

    // ----------------------------------------------------------
    // Stage 1 — stride=1: butterfly adjacent pairs
    // ----------------------------------------------------------
    wire signed [W-1:0] s1_0 = x0 + x1;
    wire signed [W-1:0] s1_1 = x0 - x1;
    wire signed [W-1:0] s1_2 = x2 + x3;
    wire signed [W-1:0] s1_3 = x2 - x3;
    wire signed [W-1:0] s1_4 = x4 + x5;
    wire signed [W-1:0] s1_5 = x4 - x5;
    wire signed [W-1:0] s1_6 = x6 + x7;
    wire signed [W-1:0] s1_7 = x6 - x7;

    // ----------------------------------------------------------
    // Stage 2 — stride=2: butterfly pairs at distance 2
    // ----------------------------------------------------------
    wire signed [W-1:0] s2_0 = s1_0 + s1_2;
    wire signed [W-1:0] s2_2 = s1_0 - s1_2;
    wire signed [W-1:0] s2_1 = s1_1 + s1_3;
    wire signed [W-1:0] s2_3 = s1_1 - s1_3;
    wire signed [W-1:0] s2_4 = s1_4 + s1_6;
    wire signed [W-1:0] s2_6 = s1_4 - s1_6;
    wire signed [W-1:0] s2_5 = s1_5 + s1_7;
    wire signed [W-1:0] s2_7 = s1_5 - s1_7;

    // ----------------------------------------------------------
    // Stage 3 — stride=4: butterfly pairs at distance 4
    // ----------------------------------------------------------
    assign y0 = s2_0 + s2_4;
    assign y4 = s2_0 - s2_4;
    assign y1 = s2_1 + s2_5;
    assign y5 = s2_1 - s2_5;
    assign y2 = s2_2 + s2_6;
    assign y6 = s2_2 - s2_6;
    assign y3 = s2_3 + s2_7;
    assign y7 = s2_3 - s2_7;

endmodule
