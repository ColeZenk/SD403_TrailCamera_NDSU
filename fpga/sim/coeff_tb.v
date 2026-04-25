`timescale 1ns / 1ps

module coeff_tb;

        localparam W = 16;
        localparam TOP_K = 5;
        localparam CLK = 10;

        reg clk, rst;
        reg bgn;
        reg signed [W*64-1:0] c_i;
        wire [3*TOP_K-1:0] r_o, c_o;
        wire signed [W*TOP_K-1:0] q_o;
        wire [2:0] n_out;
        wire done;

        coeff #(.W(W), .TOP_K(TOP_K)) dut(
            .clk(clk), .rst(rst), .bgn(bgn),
            .c_i(c_i),
            .r_o(r_o), .c_o(c_o), .q_o(q_o),
            .n_out(n_out), .done(done)
        );

        initial begin clk = 0; forever #(CLK/2) clk = ~clk; end

        integer errors;
        integer i;

        task pulse_bgn;
                begin
                        @(posedge clk);
                        bgn <= 1;
                        @(posedge clk);
                        bgn <= 0;
                end
        endtask

        task wait_done;
                begin
                        @(posedge done);
                        @(posedge clk);
                end
        endtask

        initial begin
                $dumpfile("coeff_tb.vcd");
                $dumpvars(0, coeff_tb);
                errors = 0;
                bgn = 0;
                c_i = {(W*64){1'b0}};

                rst = 1;
                #(CLK*5);
                rst = 0;
                #(CLK*2);

                // ------------------------------------------------
                // Test 1: all zeros -- n_out should be 0
                // ------------------------------------------------
                $display("[TEST 1] All zeros");
                pulse_bgn;
                wait_done;
                if (n_out !== 3'd0) begin
                        $display("  FAIL: n_out=%0d, expected 0", n_out);
                        errors = errors + 1;
                end else
                        $display("  PASS: n_out=0");

                // ------------------------------------------------
                // Test 2: single large coefficient at [1][1]
                // (r+c=2 <= 6, so not masked by seq_msk -- but
                //  coeff.v doesn't apply seq_msk, that's external)
                // Set coeff [1][1] = 1000 (above thresh=400)
                // ------------------------------------------------
                $display("[TEST 2] Single coeff at [1][1]=1000");
                c_i = {(W*64){1'b0}};
                c_i[(1*8+1)*W +: W] = 16'sd1000;
                pulse_bgn;
                wait_done;
                $display("  n_out=%0d", n_out);
                if (n_out < 1) begin
                        $display("  FAIL: expected n_out >= 1");
                        errors = errors + 1;
                end else begin
                        $display("  slot0: r=%0d c=%0d q=%0d",
                                 r_o[0*3 +: 3], c_o[0*3 +: 3],
                                 $signed(q_o[0*W +: W]));
                        if (r_o[0*3 +: 3] !== 3'd1 || c_o[0*3 +: 3] !== 3'd1) begin
                                $display("  FAIL: wrong r/c");
                                errors = errors + 1;
                        end else
                                $display("  PASS");
                end

                // ------------------------------------------------
                // Test 3: n_out clears between blocks (Bug 2 regression)
                // Run all-zeros after Test 2 -- n_out must reset to 0
                // ------------------------------------------------
                $display("[TEST 3] n_out clears on new block");
                c_i = {(W*64){1'b0}};
                pulse_bgn;
                wait_done;
                if (n_out !== 3'd0) begin
                        $display("  FAIL: n_out=%0d after zero block (not cleared!)",
                                 n_out);
                        errors = errors + 1;
                end else
                        $display("  PASS: n_out=0 after zero block");

                // ------------------------------------------------
                // Test 4: negative coefficient at [2][3] = -800
                // ------------------------------------------------
                $display("[TEST 4] Negative coeff [2][3]=-800");
                c_i = {(W*64){1'b0}};
                c_i[(2*8+3)*W +: W] = -16'sd800;
                pulse_bgn;
                wait_done;
                if (n_out < 1) begin
                        $display("  FAIL: n_out=%0d", n_out);
                        errors = errors + 1;
                end else begin
                        $display("  slot0: r=%0d c=%0d q=%0d",
                                 r_o[0*3 +: 3], c_o[0*3 +: 3],
                                 $signed(q_o[0*W +: W]));
                        if ($signed(q_o[0*W +: W]) >= 0) begin
                                $display("  FAIL: quantized coeff should be negative");
                                errors = errors + 1;
                        end else
                                $display("  PASS: negative q");
                end

                // ------------------------------------------------
                // Test 5: more than TOP_K coefficients (replacement)
                // Place 7 large coefficients, only 5 should survive
                // ------------------------------------------------
                $display("[TEST 5] TOP_K overflow (7 coeffs, keep best 5)");
                c_i = {(W*64){1'b0}};
                c_i[(0*8+1)*W +: W] = 16'sd500;
                c_i[(0*8+2)*W +: W] = 16'sd600;
                c_i[(0*8+3)*W +: W] = 16'sd700;
                c_i[(1*8+0)*W +: W] = 16'sd800;
                c_i[(1*8+1)*W +: W] = 16'sd900;
                c_i[(1*8+2)*W +: W] = 16'sd1000;
                c_i[(2*8+0)*W +: W] = 16'sd1100;
                pulse_bgn;
                wait_done;
                $display("  n_out=%0d", n_out);
                if (n_out !== 3'd5) begin
                        $display("  FAIL: expected n_out=5");
                        errors = errors + 1;
                end else
                        $display("  PASS: capped at TOP_K");

                for (i = 0; i < TOP_K; i = i + 1)
                        $display("  slot[%0d]: r=%0d c=%0d q=%0d",
                                 i, r_o[i*3+:3], c_o[i*3+:3],
                                 $signed(q_o[i*W+:W]));

                // ------------------------------------------------
                // Test 6: coefficients at high sequency (r+c > 6)
                // coeff.v itself does NOT mask these -- it should find them
                // (seq_msk is applied externally before coeff input)
                // ------------------------------------------------
                $display("[TEST 6] High-sequency coeff at [7][7]=2000");
                c_i = {(W*64){1'b0}};
                c_i[(7*8+7)*W +: W] = 16'sd2000;
                pulse_bgn;
                wait_done;
                // r+c=14 > 6, but skp = (r_cnt+c_cnt > seq_thresh=6) | DC
                // So coeff.v DOES skip this internally!
                $display("  n_out=%0d (should be 0, skipped by internal sequency)",
                         n_out);
                if (n_out !== 3'd0) begin
                        $display("  FAIL");
                        errors = errors + 1;
                end else
                        $display("  PASS");

                #(CLK*10);
                $display("\n===========================================");
                if (errors == 0) $display("*** PASS ***");
                else $display("*** FAIL *** (%0d errors)", errors);
                $display("===========================================");
                $finish;
        end

        initial begin
                #100000;
                $display("ERROR: timeout");
                $finish;
        end
endmodule
