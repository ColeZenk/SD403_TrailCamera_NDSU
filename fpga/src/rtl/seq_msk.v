/*
 * seq_msk.v -- zero coeffs for r+c>T
 *
 * T is a parameter; yosys evaluates r+c<=T at elaboration time.
 * No comparators in hardware.
 *
 * Ports flattened for yosys: element [r][c] = flat[(r*8+c)*W +: W]
 */

module seq_msk
    #(parameter W = 16, T = 6)
    (input  wire signed [W*64-1:0] c_i,
     output wire signed [W*64-1:0] c_o);

        genvar r, c;
        generate
                for (r = 0; r < 8; r = r + 1) begin : row
                        for (c = 0; c < 8; c = c + 1) begin : col
                                assign c_o[(r*8+c)*W +: W] =
                                    (r + c <= T) ? c_i[(r*8+c)*W +: W]
                                                 : {W{1'b0}};
                        end
                end
        endgenerate
endmodule
