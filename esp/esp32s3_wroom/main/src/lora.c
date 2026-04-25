/**
 * lora.c -- ESP32-S3 LoRa receiver
 *
 * Listens for compressed diff packets from DevKitV1,
 * reconstructs frames, broadcasts via WebSocket.
 */
#include "config.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/uart.h"

#include "esp_log.h"
#include "esp_timer.h"

#include "image_reconstruct.h"
#include "lora.h"
#include "ws_server.h"

static const char *TAG = "LORA";

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

        snprintf(cmd, sizeof(cmd), "AT+ADDRESS=%d\r\n", LORA_ADDRESS_RECEIVER);
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

        ESP_LOGI(TAG, "LoRa receiver ready addr=%d net=%d SF=%d BW=%d",
                 LORA_ADDRESS_RECEIVER, LORA_NETWORK_ID, LORA_SF, LORA_BW);
        return ESP_OK;
}

void lora_receive_task(void *arg)
{
        (void)arg;

        char line[300];
        uint32_t seq = 0;

        ESP_LOGI(TAG, "receive task started");

        for (;;) {
                if (!readline(line, sizeof(line), 10000))
                        continue;

                if (strncmp(line, "+RCV=", 5) != 0) {
                        ESP_LOGD(TAG, "ignoring: %s", line);
                        continue;
                }

                int src, len;
                char *data;
                if (!parse_rcv(line, &src, &data, &len)) {
                        ESP_LOGW(TAG, "parse failed");
                        continue;
                }

                ESP_LOGI(TAG, "rx %d bytes from %d", len, src);

                uint8_t pos;
                uint8_t *frame = reconstruct((const uint8_t *)data, len, &pos);
                if (!frame) {
                        ESP_LOGW(TAG, "reconstruct failed");
                        continue;
                }

                ws_broadcast(pos, ++seq, frame, FRAME_BYTES);
        }
}

/* ------------------------------------------------------------------ */
#ifdef TEST_MODE_LORA_BENCH

static void send_echo(int dest, const char *data, int len)
{
        char hdr[32];
        int hdr_len = snprintf(hdr, sizeof(hdr), "AT+SEND=%d,%d,", dest, len);
        uart_write_bytes(LORA_UART_NUM, hdr, hdr_len);
        uart_write_bytes(LORA_UART_NUM, data, len);
        uart_write_bytes(LORA_UART_NUM, "\r\n", 2);
}

static bool wait_for_start(char *line, int line_size)
{
        int src, len;
        char *data;

        static int beacon_count = 0;
        beacon_count++;
        ESP_LOGW(TAG, "beacon #%d sending READY", beacon_count);

        const char *cmd = "AT+SEND=2,5,READY\r\n";
        uart_write_bytes(LORA_UART_NUM, cmd, strlen(cmd));

        int64_t deadline = esp_timer_get_time() + 1000000LL;
        while (esp_timer_get_time() < deadline) {
                if (!readline(line, line_size, 200)) continue;
                ESP_LOGW(TAG, "handshake rx: %s", line);
                if (parse_rcv(line, &src, &data, &len) && len == 5 &&
                    !memcmp(data, "START", 5))
                        return false;
        }
        return true;
}

void lora_bench_task(void *arg)
{
        (void)arg;

        ESP_LOGI(TAG, "=== S3 echo slave ===");

        uart_init_lora();
        vTaskDelay(pdMS_TO_TICKS(500));
        module_init();
        vTaskDelay(pdMS_TO_TICKS(500));

        uart_flush_input(LORA_UART_NUM);
        vTaskDelay(pdMS_TO_TICKS(200));

        char line[300];
        while (wait_for_start(line, sizeof(line)));
        ESP_LOGI(TAG, "handshake complete");

        for (;;) {
                if (!readline(line, sizeof(line), 10000))
                        continue;
                if (strncmp(line, "+RCV=", 5) != 0)
                        continue;

                ESP_LOGI(TAG, "rx: %s", line);

                int src, len;
                char *data;
                if (!parse_rcv(line, &src, &data, &len))
                        continue;

                vTaskDelay(pdMS_TO_TICKS(15));
                send_echo(src, data, len);
        }
}
#endif
