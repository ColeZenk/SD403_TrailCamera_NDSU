/**
 * lora_bench.c — LoRa link quality test bench (TX/master on DevKitV1)
 *
 * Sends packets, waits for echoes from the S3, reports:
 *   - per-packet RTT, RSSI, SNR
 *   - summary: delivery rate, RTT min/avg/max, mean RSSI/SNR
 *   - burst test: max sustained packet rate
 *
 * SETUP:
 *   1. Define TEST_MODE_LORA_BENCH in config.h on BOTH DevKitV1 and S3.
 *   2. Set BENCH_SF and BENCH_BW identically on both sides.
 *   3. Flash DevKitV1 first, then S3.
 *   4. Open serial monitor on DevKitV1 — results print there.
 *
 * RYLR998 AT+PARAMETER: AT+PARAMETER=<SF>,<BW>,<CR>,<Preamble>
 *   BW encoding (SX1262): 7=125kHz  8=250kHz  9=500kHz
 *   CR: 1=4/5  (best for data, slightly less overhead than 4/8)
 *
 * Typical airtime for 115-byte payload:
 *   SF9 / 125kHz: ~630 ms  →  1.6 FPS max
 *   SF7 / 250kHz: ~100 ms  → 10   FPS max
 *   SF7 / 500kHz: ~50  ms  → 20   FPS max  ← 6FPS target needs this
 */

#include "config.h"
#include "lora_bench.h"

#ifdef TEST_MODE_LORA_BENCH

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_system.h"

static const char *TAG = "BENCH";

/* -----------------------------------------------------------------------
 * Internal UART helpers
 * --------------------------------------------------------------------- */

static void uart_init_bench(void)
{
    uart_config_t cfg = {
        .baud_rate  = LORA_BAUD_RATE,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_driver_install(LORA_UART_NUM, 2048, 2048, 0, NULL, 0);
    uart_param_config(LORA_UART_NUM, &cfg);
    uart_set_pin(LORA_UART_NUM, LORA_PIN_TX, LORA_PIN_RX, -1, -1);
    vTaskDelay(pdMS_TO_TICKS(300));
}

/**
 * Read one \n-terminated line from UART into buf (null-terminated).
 * Returns true if a line was read before timeout_ms elapsed.
 */
static bool readline(char *buf, int maxlen, int timeout_ms)
{
    int pos = 0;
    int64_t t_start = esp_timer_get_time() / 1000;

    while ((esp_timer_get_time() / 1000 - t_start) < timeout_ms) {
        uint8_t b;
        /* Short per-byte wait so we don't burn CPU, but use wall time for timeout */
        int n = uart_read_bytes(LORA_UART_NUM, &b, 1, pdMS_TO_TICKS(10));
        if (n <= 0) continue;

        if (b == '\n') {
            if (pos > 0 && buf[pos - 1] == '\r') pos--;
            buf[pos] = '\0';
            return pos > 0;
        }
        if (pos < maxlen - 1) buf[pos++] = (char)b;
    }
    buf[pos] = '\0';
    return false;
}

/**
 * Send an AT command string and wait for a line containing expect_str.
 * Returns true if expected response seen within timeout_ms.
 */
static bool at_cmd(const char *cmd, const char *expect_str, int timeout_ms)
{
    uart_flush_input(LORA_UART_NUM);  /* clear RX (uart_flush only flushes TX) */
    uart_write_bytes(LORA_UART_NUM, cmd, strlen(cmd));

    char line[128];
    int elapsed = 0;
    while (elapsed < timeout_ms) {
        int t0 = 50;
        if (readline(line, sizeof(line), t0)) {
            ESP_LOGI(TAG, "AT resp: %s", line);
            if (expect_str && strstr(line, expect_str)) return true;
        }
        elapsed += t0;
    }
    return false;
}

/* -----------------------------------------------------------------------
 * Module init + parameter set
 * --------------------------------------------------------------------- */

static void bench_module_init(void)
{
    /* Step 1: basic liveness check */
    ESP_LOGI(TAG, "--- AT liveness ---");
    bool alive = at_cmd("AT\r\n", "+OK", 1000);
    if (!alive) ESP_LOGW(TAG, "No +OK to bare AT — check wiring/baud");

    /* Step 2: reset module — required on each boot before AT+SEND works reliably.
     * Without this, module may be in a stuck state that returns ERR=15. */
    ESP_LOGI(TAG, "--- AT+RESET ---");
    at_cmd("AT+RESET\r\n", "+RESET", 1000);
    vTaskDelay(pdMS_TO_TICKS(1500));  /* wait for module to reboot */
    at_cmd("AT\r\n", "+OK", 1000);   /* confirm alive after reset */

    /* Step 3: query current state */
    ESP_LOGI(TAG, "--- Current module config ---");
    at_cmd("AT+ADDRESS?\r\n",   NULL, 600);
    at_cmd("AT+NETWORKID?\r\n", NULL, 600);
    at_cmd("AT+PARAMETER?\r\n", NULL, 600);
    at_cmd("AT+BAND?\r\n",      NULL, 600);

    /* Step 4: minimal ASCII send test */
    ESP_LOGI(TAG, "--- ASCII send test ---");
    bool send_ok = at_cmd("AT+SEND=0,4,TEST\r\n", "+OK", 3000);
    if (send_ok)
        ESP_LOGI(TAG, "AT+SEND accepted — module CAN transmit");
    else
        ESP_LOGE(TAG, "AT+SEND rejected — check module state (ERR=15?)");

    /* Step 5: set address, network, RF parameters */
    ESP_LOGI(TAG, "--- Configuring ---");
    char cmd[64];
    snprintf(cmd, sizeof(cmd), "AT+ADDRESS=%d\r\n", LORA_ADDRESS_RECEIVER);
    if (!at_cmd(cmd, "+OK", 600))
        ESP_LOGW(TAG, "ADDRESS set failed");

    snprintf(cmd, sizeof(cmd), "AT+NETWORKID=%d\r\n", LORA_NETWORK_ID);
    if (!at_cmd(cmd, "+OK", 600))
        ESP_LOGW(TAG, "NETWORKID set failed");

    snprintf(cmd, sizeof(cmd), "AT+PARAMETER=%d,%d,%d,%d\r\n",
             BENCH_SF, BENCH_BW, BENCH_CR, BENCH_PREAMBLE);
    if (!at_cmd(cmd, "+OK", 600))
        ESP_LOGW(TAG, "PARAMETER set failed — benching at module defaults");

    /* Confirm final state */
    ESP_LOGI(TAG, "--- Final config ---");
    at_cmd("AT+ADDRESS?\r\n",   NULL, 600);
    at_cmd("AT+NETWORKID?\r\n", NULL, 600);
    at_cmd("AT+PARAMETER?\r\n", NULL, 600);
}

/* -----------------------------------------------------------------------
 * Single packet send + echo receive
 * Returns RTT in ms, or -1 on timeout.
 * -----------------------------------------------------------------------
 * Bench payload layout (binary, size = BENCH_PKT_SIZE bytes):
 *   [0-3]  seq  (uint32 BE)
 *   [4-7]  ms   (uint32 BE, tx timestamp from esp_timer)
 *   [8-N]  0xAA filler
 *
 * RYLR998 note: AT+SEND=<addr>,<len>,<data>\r\n sends raw bytes.
 * +RCV= response: +RCV=<addr>,<len>,<data>,<rssi>,<snr>\r\n
 * 0xAA (170) contains no commas (0x2C=44) so sscanf field parsing is safe.
 * --------------------------------------------------------------------- */

static int64_t send_and_echo(uint32_t seq, int size, int timeout_ms,
                              int *rssi_out, int *snr_out)
{
    /* All-0xAA payload: no null bytes (avoids sscanf truncation on S3),
     * no commas (0x2C != 0xAA), safe for binary AT interface.
     * RTT is measured by wall clock — no need to embed timestamp. */
    uint8_t payload[240];
    (void)seq;
    memset(payload, 0xAA, size);

    /* AT+SEND=<dest_addr>,<len>,<payload>\r\n */
    char hdr[32];
    snprintf(hdr, sizeof(hdr), "AT+SEND=%d,%d,", BENCH_DEST_ADDR, size);
    uart_flush_input(LORA_UART_NUM);  /* discard any stale RX bytes */
    uart_write_bytes(LORA_UART_NUM, hdr, strlen(hdr));
    uart_write_bytes(LORA_UART_NUM, payload, size);
    uart_write_bytes(LORA_UART_NUM, "\r\n", 2);

    int64_t t0 = esp_timer_get_time() / 1000;

    /* Read lines until +RCV= arrives (echo) or timeout */
    char line[512];
    int remaining = timeout_ms;
    while (remaining > 0) {
        int step = (remaining < 50) ? remaining : 50;
        if (!readline(line, sizeof(line), step)) {
            remaining -= step;
            continue;
        }
        remaining -= step;

        if (strncmp(line, "+RCV=", 5) == 0) {
            int64_t rtt = esp_timer_get_time() / 1000 - t0;

            /* Parse RSSI and SNR from end of +RCV line.
             * Format: +RCV=<addr>,<len>,<data...>,<rssi>,<snr>
             * Since data is all 0xAA (no commas), sscanf correctly
             * identifies the two trailing comma-separated fields. */
            int addr, plen, rssi = 0, snr = 0;
            char data_buf[242];
            sscanf(line + 5, "%d,%d,%241[^,],%d,%d",
                   &addr, &plen, data_buf, &rssi, &snr);
            *rssi_out = rssi;
            *snr_out  = snr;
            return rtt;
        }
        /* Log +OK/+ERR/+SENDING so we can see if AT+SEND is being accepted */
        ESP_LOGI("BENCH_TX", "  module: %s", line);
    }
    return -1; /* timeout */
}

/* -----------------------------------------------------------------------
 * One test run: N packets at current SF/BW, print stats
 * --------------------------------------------------------------------- */

static void run_trial(int n, int pkt_size, int timeout_ms)
{
    int   rcvd = 0;
    int64_t rtt_min = INT64_MAX, rtt_max = 0, rtt_sum = 0;
    int   rssi_sum = 0, snr_sum = 0;

    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "--- Trial: SF=%d BW_code=%d pkt=%dB n=%d timeout=%dms ---",
             BENCH_SF, BENCH_BW, pkt_size, n, timeout_ms);
    ESP_LOGI(TAG, " seq   RTT(ms)  RSSI   SNR");

    for (int i = 0; i < n; i++) {
        int rssi = 0, snr = 0;
        int64_t rtt = send_and_echo(i, pkt_size, timeout_ms, &rssi, &snr);

        if (rtt < 0) {
            ESP_LOGI(TAG, " %-4d  TIMEOUT", i);
        } else {
            ESP_LOGI(TAG, " %-4d  %-7"PRId64"  %-5d  %d", i, rtt, rssi, snr);
            rcvd++;
            rtt_sum  += rtt;
            rssi_sum += rssi;
            snr_sum  += snr;
            if (rtt < rtt_min) rtt_min = rtt;
            if (rtt > rtt_max) rtt_max = rtt;
        }

        /* Small gap between packets — gives module time to process */
        vTaskDelay(pdMS_TO_TICKS(BENCH_INTER_PACKET_MS));
    }

    ESP_LOGI(TAG, "");
    if (rcvd == 0) {
        ESP_LOGW(TAG, "SUMMARY: 0/%d received — link failure", n);
        return;
    }
    float delivery = 100.0f * rcvd / n;
    ESP_LOGI(TAG, "SUMMARY: %d/%d (%.1f%% delivery)  "
                  "RTT min/avg/max=%"PRId64"/%"PRId64"/%"PRId64" ms  "
                  "RSSI avg=%d  SNR avg=%d",
             rcvd, n, delivery,
             (int64_t)rtt_min, (int64_t)(rtt_sum / rcvd), (int64_t)rtt_max,
             rssi_sum / rcvd, snr_sum / rcvd);

    /* Effective throughput */
    float fps = 1000.0f / (float)(rtt_sum / rcvd);
    float kbps = fps * pkt_size * 8 / 1000.0f;
    ESP_LOGI(TAG, "         effective ~%.1f pkt/s  %.1f kbps", fps, kbps);
}

/* -----------------------------------------------------------------------
 * Burst test: send as fast as possible for BENCH_BURST_SECS seconds,
 * count how many echoes arrive. Measures true max sustained rate.
 * --------------------------------------------------------------------- */

static void run_burst(int pkt_size)
{
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "--- Burst test: SF=%d BW_code=%d pkt=%dB for %ds ---",
             BENCH_SF, BENCH_BW, pkt_size, BENCH_BURST_SECS);

    int sent = 0, rcvd = 0;
    int64_t t_start = esp_timer_get_time() / 1000;
    int64_t t_end   = t_start + BENCH_BURST_SECS * 1000;

    for (;;) {
        int64_t now = esp_timer_get_time() / 1000;
        if (now >= t_end) break;

        int rssi, snr;
        int64_t rtt = send_and_echo(sent, pkt_size,
                                    BENCH_BURST_TIMEOUT_MS, &rssi, &snr);
        sent++;
        if (rtt >= 0) rcvd++;
    }

    int64_t elapsed = esp_timer_get_time() / 1000 - t_start;
    float pps = 1000.0f * rcvd / elapsed;
    float kbps = pps * pkt_size * 8 / 1000.0f;
    ESP_LOGI(TAG, "BURST: sent=%d rcvd=%d  %.1f pkt/s  %.1f kbps  loss=%.1f%%",
             sent, rcvd, pps, kbps, 100.0f * (sent - rcvd) / sent);
}

/* -----------------------------------------------------------------------
 * Main bench task
 * --------------------------------------------------------------------- */

void lora_bench_task(void *arg)
{
    ESP_LOGI(TAG, "=== LoRa Bench starting ===");
    ESP_LOGI(TAG, "DevKitV1 addr=%d  S3 addr=%d  net=%d",
             LORA_ADDRESS_RECEIVER, BENCH_DEST_ADDR, LORA_NETWORK_ID);

    uart_init_bench();
    vTaskDelay(pdMS_TO_TICKS(500));
    bench_module_init();
    vTaskDelay(pdMS_TO_TICKS(500));

    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  SF=%d  BW_code=%d  CR=%d  Preamble=%d",
             BENCH_SF, BENCH_BW, BENCH_CR, BENCH_PREAMBLE);
    ESP_LOGI(TAG, "  BW: 7=125kHz 8=250kHz 9=500kHz");
    ESP_LOGI(TAG, "========================================");

    /* Sanity check: 3 attempts to confirm link is up */
    ESP_LOGI(TAG, "Sanity ping (dest addr=%d, 3 attempts)...", BENCH_DEST_ADDR);
    {
        int rssi = 0, snr = 0;
        int64_t rtt = -1;
        for (int attempt = 0; attempt < 3 && rtt < 0; attempt++) {
            rtt = send_and_echo(0xDEAD, 32, 5000, &rssi, &snr);
            if (rtt < 0)
                ESP_LOGW(TAG, "  attempt %d timed out", attempt + 1);
        }
        if (rtt < 0) {
            ESP_LOGE(TAG, "SANITY PING FAILED after 3 attempts");
            ESP_LOGE(TAG, "  Check: correct dest addr? S3 booted in bench mode?");
            ESP_LOGE(TAG, "  Module default addr is usually 0 — try BENCH_DEST_ADDR=0");
            vTaskDelete(NULL);  /* errors already printed above */
        }
        ESP_LOGI(TAG, "Link UP  RTT=%"PRId64"ms  RSSI=%d  SNR=%d", rtt, rssi, snr);
    }

    /* Trial: target packet size at configured SF/BW */
    run_trial(BENCH_N, BENCH_PKT_SIZE, BENCH_TIMEOUT_MS);

    /* Trial: larger packet (worst-case compressed frame) */
    if (BENCH_PKT_SIZE < 200)
        run_trial(BENCH_N / 2, 200, BENCH_TIMEOUT_MS);

    /* Burst: measure max sustainable rate */
    run_burst(BENCH_PKT_SIZE);

    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "=== Bench complete ===");
    ESP_LOGI(TAG, "Rule of thumb for 6 FPS: need avg RTT < 167ms (1000/6)");
    ESP_LOGI(TAG, "Accounting for one-way: TX airtime < 83ms per 115B packet");

    vTaskDelete(NULL);
}

#else /* TEST_MODE_LORA_BENCH not defined — provide empty stub so linker is happy */

void lora_bench_task(void *arg) { (void)arg; }

#endif /* TEST_MODE_LORA_BENCH */
