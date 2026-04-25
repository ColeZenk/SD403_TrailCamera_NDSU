/*
 * wht2d.v -- Single wht8 wrapper
 *
 * Caller feeds one row or column (8 elements) per cycle,
 * latches results back. 16 passes = full 2D WHT on 8x8 block.
 */

module wht2d
    #(parameter W = 16)
    (input wire signed [W*8-1:0] x,
     output wire signed [W*8-1:0] y);

        wht8 #(.W(W)) u(
            .x0(x[0*W +: W]),
            .x1(x[1*W +: W]),
            .x2(x[2*W +: W]),
            .x3(x[3*W +: W]),
            .x4(x[4*W +: W]),
            .x5(x[5*W +: W]),
            .x6(x[6*W +: W]),
            .x7(x[7*W +: W]),
            .y0(y[0*W +: W]),
            .y1(y[1*W +: W]),
            .y2(y[2*W +: W]),
            .y3(y[3*W +: W]),
            .y4(y[4*W +: W]),
            .y5(y[5*W +: W]),
            .y6(y[6*W +: W]),
            .y7(y[7*W +: W])
        );
endmodule
