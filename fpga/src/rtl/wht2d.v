/*
 * wht2d.v -- Takes wht8.v as foundation and
 *            and applies it to 2 dimentions.
 *
 * Ports flattened for yosys: element [r][c] = flat[(r*8+c)*W +: W]
 *
 * Author: Cole Zenk
 */

module wht2d
    #(parameter W = 16)
    (input  wire signed [W*64-1:0] b_i,
     output wire signed [W*64-1:0] b_o);

        wire signed [W*64-1:0] r_o;

        genvar i;
        generate
                for (i = 0; i < 8; i = i + 1) begin : transform_loop
                        // Row pass
                        wht8 #(.W(W)) rows(
                            .x0(b_i[(i*8+0)*W +: W]),
                            .x1(b_i[(i*8+1)*W +: W]),
                            .x2(b_i[(i*8+2)*W +: W]),
                            .x3(b_i[(i*8+3)*W +: W]),
                            .x4(b_i[(i*8+4)*W +: W]),
                            .x5(b_i[(i*8+5)*W +: W]),
                            .x6(b_i[(i*8+6)*W +: W]),
                            .x7(b_i[(i*8+7)*W +: W]),
                            .y0(r_o[(i*8+0)*W +: W]),
                            .y1(r_o[(i*8+1)*W +: W]),
                            .y2(r_o[(i*8+2)*W +: W]),
                            .y3(r_o[(i*8+3)*W +: W]),
                            .y4(r_o[(i*8+4)*W +: W]),
                            .y5(r_o[(i*8+5)*W +: W]),
                            .y6(r_o[(i*8+6)*W +: W]),
                            .y7(r_o[(i*8+7)*W +: W])
                        );

                        // Column pass -- note transpose: [j][i] indexing
                        wht8 #(.W(W)) cols(
                            .x0(r_o[(0*8+i)*W +: W]),
                            .x1(r_o[(1*8+i)*W +: W]),
                            .x2(r_o[(2*8+i)*W +: W]),
                            .x3(r_o[(3*8+i)*W +: W]),
                            .x4(r_o[(4*8+i)*W +: W]),
                            .x5(r_o[(5*8+i)*W +: W]),
                            .x6(r_o[(6*8+i)*W +: W]),
                            .x7(r_o[(7*8+i)*W +: W]),
                            .y0(b_o[(0*8+i)*W +: W]),
                            .y1(b_o[(1*8+i)*W +: W]),
                            .y2(b_o[(2*8+i)*W +: W]),
                            .y3(b_o[(3*8+i)*W +: W]),
                            .y4(b_o[(4*8+i)*W +: W]),
                            .y5(b_o[(5*8+i)*W +: W]),
                            .y6(b_o[(6*8+i)*W +: W]),
                            .y7(b_o[(7*8+i)*W +: W])
                        );
                end
        endgenerate
endmodule
