/**
 * lora_bench.c — LoRa link quality test bench (RX/echo side on S3)
 *
 * Waits for packets from DevKitV1, echoes them back verbatim.
 * DevKitV1 is the TX master and measures RTT, RSSI, SNR, delivery rate.
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
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "esp_freertos_hooks.h"

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
    at_cmd("AT+ADDRESS?\r\n",   "+ADDRESS=",   600);
    at_cmd("AT+NETWORKID?\r\n", "+NETWORKID=", 600);
    at_cmd("AT+PARAMETER?\r\n", "+PARAMETER=", 600);
    at_cmd("AT+BAND?\r\n",      "+BAND=",      600);

    /* Step 4: set address, network, RF parameters */
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

    /* Step 5: confirm final state */
    ESP_LOGI(TAG, "--- Final config ---");
    at_cmd("AT+ADDRESS?\r\n",   "+ADDRESS=",   600);
    at_cmd("AT+NETWORKID?\r\n", "+NETWORKID=", 600);
    at_cmd("AT+PARAMETER?\r\n", "+PARAMETER=", 600);
}

/* -----------------------------------------------------------------------
 * Echo helpers
 * --------------------------------------------------------------------- */

/**
 * Parse a +RCV=<src>,<len>,<data>,<rssi>,<snr> line in-place.
 * Writes NUL terminators into `line` at the comma positions.
 * Sets *data_out to point into `line` at the start of the data field.
 * Returns false if the line is malformed or pkt_len is out of range.
 */
static bool parse_rcv(char *line, int *src_addr, int *pkt_len,
                      char **data_out, int *rssi, int *snr)
{
    char *p = line + 5;  /* skip "+RCV=" */

    char *c1 = strchr(p, ',');
    if (!c1) return false;
    *c1 = '\0';
    *src_addr = atoi(p);

    char *p2 = c1 + 1;
    char *c2 = strchr(p2, ',');
    if (!c2) return false;
    *c2 = '\0';
    *pkt_len = atoi(p2);
    if (*pkt_len <= 0 || *pkt_len > 240) return false;

    *data_out = c2 + 1;

    /* rssi/snr sit after the data field — use length to find them */
    char *after = *data_out + *pkt_len;
    *rssi = 0; *snr = 0;
    if (*after == ',') sscanf(after + 1, "%d,%d", rssi, snr);

    return true;
}

/**
 * Echo data back to src_addr.
 * Writes header and payload as separate uart_write_bytes calls to avoid
 * copying binary data into a string buffer.
 */
static void send_echo(int src_addr, const char *data, int pkt_len)
{
    char hdr[32];
    int hdr_len = snprintf(hdr, sizeof(hdr), "AT+SEND=%d,%d,", src_addr, pkt_len);
    uart_write_bytes(LORA_UART_NUM, hdr, hdr_len);
    uart_write_bytes(LORA_UART_NUM, data, pkt_len);
    uart_write_bytes(LORA_UART_NUM, "\r\n", 2);
}

/* -----------------------------------------------------------------------
 * Main bench task
 * --------------------------------------------------------------------- */

/* Returning false keeps can_go_idle=false, preventing WAITI.
 * Required on S3 with octal PSRAM: WAITI stalls CPU while a pending cache
 * miss needs MSPI — deadlock → IWDT reset. */
static bool no_waiti_idle_hook(void) { return false; }

void lora_bench_task(void *arg)
{
    esp_register_freertos_idle_hook(no_waiti_idle_hook);

    ESP_LOGI(TAG, "=== S3 LoRa echo bench starting ===");
    ESP_LOGI(TAG, "addr=%d  net=%d  SF=%d  BW_code=%d  CR=%d  Preamble=%d",
             LORA_ADDRESS_RECEIVER, LORA_NETWORK_ID,
             BENCH_SF, BENCH_BW, BENCH_CR, BENCH_PREAMBLE);

    uart_init_bench();
    vTaskDelay(pdMS_TO_TICKS(500));
    bench_module_init();
    vTaskDelay(pdMS_TO_TICKS(500));

    ESP_LOGI(TAG, "Echo loop ready — waiting for packets from DevKitV1");

    char     line[300];
    uint32_t rx_count = 0;

    /* Flood state — flood packets (first byte=0xFF) are counted, not echoed */
    bool     flood_active   = false;
    uint32_t flood_count    = 0;
    int      flood_pkt_size = 0;
    int64_t  flood_start_ms = 0;

    for (;;) {
        /* Shorter timeout while flood is running so we detect end-of-burst quickly */
        int rl_timeout = flood_active ? 2000 : 5000;
        if (!readline(line, sizeof(line), rl_timeout)) {
            if (flood_active) {
                int64_t elapsed = esp_timer_get_time() / 1000 - flood_start_ms;
                float   pps     = 1000.0f * flood_count / (float)elapsed;
                float   kbps    = pps * flood_pkt_size * 8.0f / 1000.0f;
                ESP_LOGI(TAG, "FLOOD RX done: %"PRIu32" pkts  %"PRId64"ms  "
                              "%.1f pkt/s  %.1f kbps",
                         flood_count, elapsed, pps, kbps);
                flood_active = false;
                flood_count  = 0;
            }
            continue;
        }

        if (strncmp(line, "+RCV=", 5) != 0) continue;

        int src_addr, pkt_len, rssi, snr;
        char *data;
        if (!parse_rcv(line, &src_addr, &pkt_len, &data, &rssi, &snr)) continue;

        /* Flood packet: first byte = 0xFF — count only, no echo */
        if (pkt_len > 0 && (uint8_t)data[0] == 0xFF) {
            if (!flood_active) {
                flood_active   = true;
                flood_count    = 0;
                flood_pkt_size = pkt_len;
                flood_start_ms = esp_timer_get_time() / 1000;
                ESP_LOGI(TAG, "Flood RX started (pkt=%dB)...", pkt_len);
            }
            flood_count++;
            if (flood_count % 20 == 0)
                ESP_LOGI(TAG, "  flood: %"PRIu32"  RSSI=%d SNR=%d",
                         flood_count, rssi, snr);
            continue;
        }

        /* Echo packet */
        vTaskDelay(pdMS_TO_TICKS(15));
        send_echo(src_addr, data, pkt_len);
        readline(line, sizeof(line), 1000);  /* consume +OK */

        rx_count++;
        if (rx_count % 10 == 0)
            ESP_LOGI(TAG, "echoed %"PRIu32" pkts  last: src=%d len=%d RSSI=%d SNR=%d",
                     rx_count, src_addr, pkt_len, rssi, snr);
    }
}

#else /* TEST_MODE_LORA_BENCH not defined — provide empty stub so linker is happy */

void lora_bench_task(void *arg) { (void)arg; }

#endif /* TEST_MODE_LORA_BENCH */
