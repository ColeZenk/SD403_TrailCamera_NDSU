`timescale 1ns / 1ps

// esp_interface TX path testbench -- accurate top.v tx_buf model.
//
// Models the synchronous BRAM read lag in top.v:
//   tx_rd_ptr advances on tx_next (registered).
//   tx_rd_data = tx_buf[tx_rd_ptr] is a registered read -- reads the OLD ptr.
//   tx_sr is loaded from tx_rd_data on byte_done_d (one cycle after tx_next).
//   Because both happen at the same posedge, tx_sr sees the pre-advance byte.
//
// Bug 1 (MISO shift): byte[N>=1] arrives as correct_byte << 1.
// Bug 3 (BRAM lag):   byte[N>=1] arrives as tx_mem[N-1] (one behind).
// Both must be fixed for all tests to pass.

module esp_interface_tx_tb;

        localparam CLK_PERIOD = 37;
        localparam SPI_HALF = CLK_PERIOD * 6;

        reg clk, rst_n;
        reg tb_mosi, tb_sclk, tb_cs_n;
        wire tb_miso;

        wire [7:0] rx_data;
        wire rx_valid;
        reg rx_ready;

        wire [7:0] tx_data;
        wire tx_next;
        wire cs_active_sync;

        esp_interface dut(
            .clk(clk), .rst_n(rst_n),
            .esp_mosi(tb_mosi), .esp_miso(tb_miso),
            .esp_sclk(tb_sclk), .esp_cs_n(tb_cs_n),
            .rx_data(rx_data), .rx_valid(rx_valid), .rx_ready(rx_ready),
            .tx_data(tx_data), .tx_next(tx_next),
            .cs_active_sync(cs_active_sync)
        );

        initial begin clk = 0; forever #(CLK_PERIOD/2) clk = ~clk; end

        integer errors;

        reg [7:0] tx_mem [0:15];

        initial begin
                tx_mem[0]  = 8'hDE;
                tx_mem[1]  = 8'hAD;
                tx_mem[2]  = 8'hBE;
                tx_mem[3]  = 8'hEF;
                tx_mem[4]  = 8'h01;
                tx_mem[5]  = 8'h23;
                tx_mem[6]  = 8'h45;
                tx_mem[7]  = 8'h67;
                tx_mem[8]  = 8'h89;
                tx_mem[9]  = 8'hAB;
                tx_mem[10] = 8'hCD;
                tx_mem[11] = 8'hEF;
                tx_mem[12] = 8'hFE;
                tx_mem[13] = 8'hDC;
                tx_mem[14] = 8'hBA;
                tx_mem[15] = 8'h98;
        end

        // Accurate top.v tx_buf model.
        // tx_rd_ptr advances on tx_next; resets when CS asserts.
        // tx_rd_data is a registered read of tx_mem[tx_rd_ptr] -- reads the OLD ptr
        // because both the ptr advance and the BRAM address are non-blocking.
        // This means tx_sr is loaded with the PREVIOUS byte on each byte boundary.
        reg [7:0] tx_rd_ptr;
        reg [7:0] tx_rd_data;
        reg       tb_cs_n_d;

        wire cs_assert = tb_cs_n_d & ~tb_cs_n;

        always @(posedge clk or negedge rst_n) begin
                if (!rst_n) begin
                        tx_rd_ptr  <= 8'd0;
                        tx_rd_data <= tx_mem[0];
                        tb_cs_n_d  <= 1'b1;
                end else begin
                        tb_cs_n_d  <= tb_cs_n;
                        if (cs_assert)
                                tx_rd_ptr <= 8'd0;
                        else if (tx_next)
                                tx_rd_ptr <= tx_rd_ptr + 8'd1;
                        tx_rd_data <= tx_mem[tx_rd_ptr]; // reads OLD ptr -- BRAM lag
                end
        end

        assign tx_data = tx_rd_data;

        task spi_xfer_byte;
                output [7:0] captured;
                integer i;
                begin
                        captured = 8'd0;
                        for (i = 7; i >= 0; i = i - 1) begin
                                tb_mosi = 0;
                                #(SPI_HALF);
                                tb_sclk = 1;
                                captured[i] = tb_miso;
                                #(SPI_HALF);
                                tb_sclk = 0;
                        end
                end
        endtask

        reg [7:0] got;
        integer byte_num;

        initial begin
                $dumpfile("esp_interface_tx_tb.vcd");
                $dumpvars(0, esp_interface_tx_tb);
                errors = 0;

                rst_n = 0;
                tb_mosi = 0;
                tb_sclk = 0;
                tb_cs_n = 1;
                rx_ready = 1;

                #(CLK_PERIOD*10);
                rst_n = 1;
                #(CLK_PERIOD*10);

                $display("===========================================");
                $display("ESP Interface TX Path Testbench (accurate)");
                $display("===========================================");

                // ----------------------------------------------------------
                // Test 1: 8-byte sequential readback
                // ----------------------------------------------------------
                $display("\n[TEST 1] 8-byte TX readback");
                tb_cs_n = 0;
                #(CLK_PERIOD * 8);

                for (byte_num = 0; byte_num < 8; byte_num = byte_num + 1) begin
                        spi_xfer_byte(got);
                        $display("  byte[%0d]: got=0x%02X exp=0x%02X %s",
                                 byte_num, got, tx_mem[byte_num],
                                 (got == tx_mem[byte_num]) ? "OK" : "MISMATCH");
                        if (got !== tx_mem[byte_num])
                                errors = errors + 1;
                end

                #(SPI_HALF * 2);
                tb_cs_n = 1;
                #(CLK_PERIOD*20);

                // ----------------------------------------------------------
                // Test 2: CS deassert/reassert -- ptr resets, byte[0] again
                // ----------------------------------------------------------
                $display("\n[TEST 2] CS toggle re-preload");
                tb_cs_n = 0;
                #(CLK_PERIOD * 8);
                spi_xfer_byte(got);
                $display("  first byte after re-CS: got=0x%02X exp=0x%02X %s",
                         got, tx_mem[0],
                         (got == tx_mem[0]) ? "OK" : "MISMATCH");
                if (got !== tx_mem[0])
                        errors = errors + 1;
                #(SPI_HALF * 2);
                tb_cs_n = 1;
                #(CLK_PERIOD*20);

                // ----------------------------------------------------------
                // Test 3: 16 back-to-back bytes, no inter-byte gap
                // ----------------------------------------------------------
                $display("\n[TEST 3] Back-to-back bytes (16 bytes, no gap)");
                rst_n = 0;
                #(CLK_PERIOD*5);
                rst_n = 1;
                #(CLK_PERIOD*10);

                tb_cs_n = 0;
                #(CLK_PERIOD * 8);

                for (byte_num = 0; byte_num < 16; byte_num = byte_num + 1) begin
                        spi_xfer_byte(got);
                        $display("  byte[%0d]: got=0x%02X exp=0x%02X %s",
                                 byte_num, got, tx_mem[byte_num],
                                 (got == tx_mem[byte_num]) ? "OK" : "MISMATCH");
                        if (got !== tx_mem[byte_num])
                                errors = errors + 1;
                end

                #(SPI_HALF * 2);
                tb_cs_n = 1;
                #(CLK_PERIOD*50);

                $display("\n===========================================");
                $display("  Errors: %0d", errors);
                if (errors == 0) $display("*** PASS ***");
                else $display("*** FAIL ***");
                $display("===========================================");
                $finish;
        end

        initial begin
                #5000000;
                $display("ERROR: timeout");
                $finish;
        end
endmodule
