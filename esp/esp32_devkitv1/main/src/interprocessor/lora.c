/**
 * lora.c -- DevKitV1 LoRa transmitter
 *
 * Sends compressed diff packets to S3 receiver via RYLR998.
 */
#include "config.h"

#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "lora.h"

static const char *TAG = "LORA";

static const uint16_t LORA_BUF_SIZE = (1 << 9);

/* ------------------------------------------------------------------ */
static void uart_init_lora(void)
{
        uart_config_t cfg = {
            .baud_rate = LORA_BAUD_RATE,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .source_clk = UART_SCLK_DEFAULT,
        };
        uart_driver_install(LORA_UART_NUM, LORA_BUF_SIZE, LORA_BUF_SIZE, 0,
                            NULL, 0);
        uart_param_config(LORA_UART_NUM, &cfg);
        uart_set_pin(LORA_UART_NUM, LORA_PIN_TX, LORA_PIN_RX, -1, -1);
        vTaskDelay(pdMS_TO_TICKS(LORA_INIT_DELAY_MS));
}

static bool readline(char *buf, int lenmax, int timeout_ms)
{
        int pos = 0;
        int64_t deadline = esp_timer_get_time() + (int64_t)timeout_ms * 1000;
        while (esp_timer_get_time() < deadline) {
                uint8_t b;
                if (uart_read_bytes(LORA_UART_NUM, &b, 1, pdMS_TO_TICKS(10)) <=
                    0)
                        continue;
                if (b == '\n') {
                        if (pos > 0 && buf[pos - 1] == '\r') pos--;
                        buf[pos] = '\0';
                        return pos > 0;
                }
                if (pos < lenmax - 1) buf[pos++] = (char)b;
        }
        buf[pos] = '\0';
        return false;
}

static bool at_cmd(const char *cmd, const char *expect, int timeout_ms)
{
        static const int POLL_MS = 50;
        uart_flush_input(LORA_UART_NUM);
        uart_write_bytes(LORA_UART_NUM, cmd, strlen(cmd));
        char line[INT8_MAX + 1];
        int elapsed = 0;
        while (elapsed < timeout_ms) {
                if (readline(line, sizeof(line), POLL_MS)) {
                        ESP_LOGI(TAG, " %s", line);
                        if (expect && strstr(line, expect)) return true;
                }
                elapsed += POLL_MS;
        }
        return false;
}

/* ------------------------------------------------------------------ */
static void module_init(void)
{
        at_cmd("AT+RESET\r\n", "+RESET", 1000);
        vTaskDelay(pdMS_TO_TICKS(1500));

        char cmd[64];
        snprintf(cmd, sizeof(cmd), "AT+ADDRESS=%d\r\n", LORA_ADDRESS_SENDER);
        at_cmd(cmd, "+OK", LORA_AT_TIMEOUT_MS);

        snprintf(cmd, sizeof(cmd), "AT+NETWORKID=%d\r\n", LORA_NETWORK_ID);
        at_cmd(cmd, "+OK", LORA_AT_TIMEOUT_MS);

        snprintf(cmd, sizeof(cmd), "AT+PARAMETER=%d,%d,%d,%d\r\n", LORA_SF,
                 LORA_BW, LORA_CR, LORA_PREAMBLE);
        at_cmd(cmd, "+OK", LORA_AT_TIMEOUT_MS);

        at_cmd("AT+CRFOP=7\r\n", "+OK", LORA_AT_TIMEOUT_MS);

        at_cmd("AT+ADDRESS?\r\n", "+ADDRESS=", LORA_AT_TIMEOUT_MS);
        at_cmd("AT+NETWORKID?\r\n", "+NETWORKID=", LORA_AT_TIMEOUT_MS);
        at_cmd("AT+PARAMETER?\r\n", "+PARAMETER=", LORA_AT_TIMEOUT_MS);
        at_cmd("AT+BAND?\r\n", "+BAND=", LORA_AT_TIMEOUT_MS);
        at_cmd("AT+CRFOP?\r\n", "+CRFOP=", LORA_AT_TIMEOUT_MS);
}

/* ------------------------------------------------------------------ */
__attribute__((unused))
static bool parse_rcv(char *line, int *src, char **data, int *len)
{
        if (strncmp(line, "+RCV=", 5) != 0) return false;

        char *p = line + 5;

        char *c1 = strchr(p, ',');
        if (!c1) return false;
        *c1 = '\0';
        *src = atoi(p);

        char *p_len = c1 + 1;
        char *c2 = strchr(p_len, ',');
        if (!c2) return false;
        *c2 = '\0';

        *len = atoi(p_len);
        if (*len <= 0 || *len > LORA_MAX_PAYLOAD) return false;

        *data = c2 + 1;

        char *end = *data + *len;
        if (*end == ',') {
                *end = '\0';
        } else if (*end == '\0') {
                /* firmware omitted rssi/snr */
        } else {
                return false;
        }

        return true;
}

/* ------------------------------------------------------------------ */
esp_err_t lora_init(void)
{
        uart_init_lora();
        vTaskDelay(pdMS_TO_TICKS(500));
        module_init();
        vTaskDelay(pdMS_TO_TICKS(500));

        ESP_LOGI(TAG, "LoRa transmitter ready addr=%d dest=%d net=%d SF=%d BW=%d",
                 LORA_ADDRESS_SENDER, LORA_DEST_ADDR, LORA_NETWORK_ID,
                 LORA_SF, LORA_BW);
        return ESP_OK;
}

esp_err_t lora_send_packet(const uint8_t *data, size_t len)
{
        if (!data || len == 0 || len > LORA_MAX_PAYLOAD)
                return ESP_ERR_INVALID_ARG;

        char hdr[32];
        int hdr_len = snprintf(hdr, sizeof(hdr), "AT+SEND=%d,%d,",
                               LORA_DEST_ADDR, (int)len);

        uart_write_bytes(LORA_UART_NUM, hdr, hdr_len);
        uart_write_bytes(LORA_UART_NUM, (const char *)data, len);
        uart_write_bytes(LORA_UART_NUM, "\r\n", 2);

        return ESP_OK;
}

/* ------------------------------------------------------------------ */
#ifdef TEST_MODE_LORA_BENCH

static void send_msg(uint32_t seq, int size)
{
        static const int IDEAL_PAYLOAD = 0xF0;
        static const int MAX_HDR = 0x20;
        static const int num_hold = 9;
        uint8_t payload[IDEAL_PAYLOAD + 1];
        int mask = -(size > IDEAL_PAYLOAD);
        size = size ^ (mask & (IDEAL_PAYLOAD ^ size));
        memset(payload, 'A', size);
        char seq_str[num_hold];
        snprintf(seq_str, num_hold, "%08lx", (unsigned long)seq);
        memcpy(payload, seq_str, num_hold - 1);
        char hdr[MAX_HDR];
        int hdr_len = snprintf(hdr, sizeof(hdr), "AT+SEND=%d,%d,",
                               BENCH_DEST_ADDR, size);
        uart_write_bytes(LORA_UART_NUM, hdr, hdr_len);
        uart_write_bytes(LORA_UART_NUM, (char *)payload, size);
        uart_write_bytes(LORA_UART_NUM, "\r\n", 2);
}

static int64_t wait_echo(int timeout_ms, int *rssi_out, int *snr_out)
{
        int64_t t0 = esp_timer_get_time() >> 10;
        char line[300];
        bool ok_received = false;

        while ((esp_timer_get_time() >> 10) - t0 < timeout_ms) {
                if (!readline(line, sizeof(line), 100))
                        continue;

                if (strstr(line, "+OK")) {
                        ESP_LOGI(TAG, "AT+SEND ack: %s", line);
                        ok_received = true;
                } else if (strncmp(line, "+RCV=", 5) == 0) {
                        ESP_LOGI(TAG, "rx: %s", line);
                        if (!ok_received)
                                ESP_LOGW(TAG, "echo before +OK");
                        int64_t rtt = (esp_timer_get_time() >> 10) - t0;
                        *rssi_out = 0;
                        *snr_out = 0;
                        char *p = line + 5;
                        char *c1 = strchr(p, ',');
                        if (c1) {
                                char *c2 = strchr(c1 + 1, ',');
                                if (c2) {
                                        char *after = c2 + 1 + BENCH_PKT_SIZE;
                                        if (*after == ',')
                                                sscanf(after + 1, "%d,%d",
                                                       rssi_out, snr_out);
                                }
                        }
                        return rtt;
                }
        }

        ESP_LOGW(TAG, "wait_echo timed out");
        return -1;
}

static bool wait_for_ready(char *line, int line_size)
{
        int src, len;
        char *data;

        bool valid_line = readline(line, line_size, 500);
        bool valid_rcvp = valid_line && parse_rcv(line, &src, &data, &len);
        bool valid_ascii = valid_rcvp && (len == 5) && !memcmp(data, "READY", 5);

        if (valid_line)
                ESP_LOGI(TAG, "handshake rx: %s (match=%d)", line, valid_ascii);

        return !valid_ascii;
}

void lora_bench_task(void *arg)
{
        (void)arg;

        ESP_LOGI(TAG, "=== DevKit TX bench ===");
        ESP_LOGI(TAG, "addr=%d dest=%d net=%d SF=%d BW=%d",
                 LORA_ADDRESS_SENDER, BENCH_DEST_ADDR, LORA_NETWORK_ID,
                 BENCH_SF, BENCH_BW);

        uart_init_lora();
        vTaskDelay(pdMS_TO_TICKS(500));
        module_init();
        vTaskDelay(pdMS_TO_TICKS(500));

        ESP_LOGI(TAG, "waiting for handshake...");

        char line[300];
        while (wait_for_ready(line, sizeof(line)));
        const char *cmd = "AT+SEND=1,5,START\r\n";
        uart_write_bytes(LORA_UART_NUM, cmd, strlen(cmd));
        readline(line, sizeof(line), 500);
        ESP_LOGI(TAG, "START sent");

        vTaskDelay(pdMS_TO_TICKS(100));

        int delivered = 0;
        int64_t rtt_sum = 0;

        ESP_LOGI(TAG, "echo trial: %d packets, %d bytes", BENCH_N,
                 BENCH_PKT_SIZE);

        for (int i = 0; i < BENCH_N; i++) {
                int rssi, snr;
                send_msg(i, BENCH_PKT_SIZE);
                int64_t rtt = wait_echo(BENCH_TIMEOUT_MS, &rssi, &snr);

                if (rtt >= 0) {
                        delivered++;
                        rtt_sum += rtt;
                        if (delivered % 10 == 0)
                                ESP_LOGI(TAG,
                                         "  [%d] RTT=%lldms RSSI=%d SNR=%d", i,
                                         rtt, rssi, snr);
                } else {
                        ESP_LOGW(TAG, "  [%d] timeout", i);
                }
                vTaskDelay(pdMS_TO_TICKS(BENCH_INTER_PACKET_MS));
        }

        ESP_LOGI(TAG, "");
        ESP_LOGI(TAG, "=== results ===");
        ESP_LOGI(TAG, "delivered: %d/%d (%.1f%%)", delivered, BENCH_N,
                 100.0f * delivered / BENCH_N);
        if (delivered > 0)
                ESP_LOGI(TAG, "avg RTT: %lld ms", rtt_sum / delivered);

        vTaskDelete(NULL);
}
#endif
