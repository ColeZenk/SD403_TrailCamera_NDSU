`timescale 1ns / 1ps

// Block scheduler testbench
// Mocks PSRAM with a simple memory array.
// Tests: single block flow, frame iteration, emit packet format.

module blk_schd_tb;

        localparam W = 16;
        localparam TOP_K = 5;
        localparam CLK = 10;

        reg clk, rst_n, start;
        wire frame_done;
        wire psram_read;
        wire [21:0] psram_addr;
        reg [15:0] psram_rdata;
        reg psram_busy;
        wire tx_valid;
        wire [7:0] tx_data;
        reg tx_ready;
        reg [21:0] cur_base, prev_base;

        block_sched #(.W(W), .TOP_K(TOP_K)) dut(
            .clk(clk), .rst_n(rst_n), .start(start),
            .frame_done(frame_done),
            .psram_read(psram_read), .psram_addr(psram_addr),
            .psram_rdata(psram_rdata), .psram_busy(psram_busy),
            .tx_valid(tx_valid), .tx_data(tx_data), .tx_ready(tx_ready),
            .cur_base(cur_base), .prev_base(prev_base)
        );

        initial begin clk = 0; forever #(CLK/2) clk = ~clk; end

        // Mock PSRAM: 1-cycle latency
        reg [7:0] psram [0:153599]; // 320*240*2 = 153600 bytes
        integer pi;

        initial begin
                for (pi = 0; pi < 153600; pi = pi + 1)
                        psram[pi] = 8'd0;
        end

        // PSRAM read model: assert busy for 1 cycle, then deliver
        reg psram_read_d;
        reg [21:0] psram_addr_d;

        always @(posedge clk) begin
                psram_read_d <= psram_read;
                psram_addr_d <= psram_addr;
                if (psram_read)
                        psram_busy <= 1;
                else if (psram_busy && psram_read_d) begin
                        psram_rdata <= {psram[psram_addr_d + 1],
                                        psram[psram_addr_d]};
                        psram_busy <= 0;
                end
        end

        // Capture TX output
        reg [7:0] tx_log [0:511];
        integer tx_cnt;

        always @(posedge clk) begin
                if (tx_valid) begin
                        tx_log[tx_cnt] <= tx_data;
                        tx_cnt <= tx_cnt + 1;
                end
        end

        integer errors;

        initial begin
                $dumpfile("blk_schd_tb.vcd");
                $dumpvars(0, blk_schd_tb);
                errors = 0;
                rst_n = 0;
                start = 0;
                psram_busy = 0;
                psram_rdata = 0;
                tx_ready = 1;
                tx_cnt = 0;
                cur_base = 22'd0;
                prev_base = 22'h012C00;

                #(CLK*10);
                rst_n = 1;
                #(CLK*5);

                // ------------------------------------------------
                // Test 1: All-zero frame -- no coefficients, no TX
                // Frame should complete with 0 TX bytes
                // ------------------------------------------------
                $display("===========================================");
                $display("[TEST 1] All-zero frame (cur==prev)");
                $display("===========================================");

                // Both cur and prev are zero, so diffs are all zero
                start = 1;
                @(posedge clk);
                start = 0;

                @(posedge frame_done);
                @(posedge clk);
                $display("  frame_done asserted, tx_cnt=%0d", tx_cnt);
                if (tx_cnt !== 0) begin
                        $display("  FAIL: expected 0 TX bytes for zero diff");
                        errors = errors + 1;
                end else
                        $display("  PASS");

                #(CLK*20);

                // ------------------------------------------------
                // Test 2: Single block with known diff
                // Set block [0][0] pixels in cur to non-zero,
                // prev stays zero. Should produce coefficients.
                // ------------------------------------------------
                $display("\n===========================================");
                $display("[TEST 2] Single-block diff (block 0,0)");
                $display("===========================================");

                // Fill first 8x8 block with a gradient so WHT produces
                // AC coefficients above threshold (uniform=DC only, skipped)
                for (pi = 0; pi < 8; pi = pi + 1) begin : fill_rows
                        integer ci;
                        for (ci = 0; ci < 8; ci = ci + 1) begin
                                psram[pi * 320 + ci] = (pi * 32 + ci * 4) & 8'hFF;
                        end
                end

                tx_cnt = 0;
                start = 1;
                @(posedge clk);
                start = 0;

                @(posedge frame_done);
                @(posedge clk);
                $display("  frame_done asserted, tx_cnt=%0d", tx_cnt);
                if (tx_cnt > 0) begin
                        $display("  PASS: got TX data from non-zero block");
                        $display("  header: 0x%02X 0x%02X", tx_log[0], tx_log[1]);
                        // header_hi = {2'b0, by[4:0], bx[5]} = {2'b0, 0, 0} = 0x00
                        // header_lo = {bx[4:0], coeff_n} = {0, n}
                        $display("  coeff_n from header = %0d", tx_log[1][2:0]);
                end else begin
                        $display("  FAIL: no TX data for 128-vs-0 diff");
                        errors = errors + 1;
                end

                #(CLK*50);

                $display("\n===========================================");
                if (errors == 0) $display("*** PASS ***");
                else $display("*** FAIL *** (%0d errors)", errors);
                $display("===========================================");
                $finish;
        end

        initial begin
                #50000000;
                $display("ERROR: timeout");
                $finish;
        end
endmodule
