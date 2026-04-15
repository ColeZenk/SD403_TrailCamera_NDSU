/**
 * lora_bench.c - ESP32-S3 (echo slave)
 *
 * Listen for packets from DevKit, echo back verbatim.
 * DevKit must be running TX master with matching RF config.
 */
#include "config.h"
#ifdef TEST_MODE_LORA_BENCH

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/uart.h"

#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "ECHO";

/* ------------------------------------------------------------------ */
static void uart_init_bench(void) {
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

static bool readline(char *buf, int lenmax, int timeout_ms) {
        int pos = 0;
        int64_t deadline = esp_timer_get_time() + (int64_t)timeout_ms * 1000;

        while (esp_timer_get_time() < deadline) {
                uint8_t b;
                // IMPORTANT: small per-byte wait so we assemble lines quickly
                if (uart_read_bytes(LORA_UART_NUM, &b, 1, pdMS_TO_TICKS(10)) <=
                    0)
                        continue;

                if (b == '\n') {
                        if (pos > 0 && buf[pos - 1] == '\r') pos--;
                        buf[pos] = '\0';
                        return pos > 0;
                }

                if (pos < lenmax - 1) buf[pos++] = (char)b;
                // else: overlong line; keep consuming until '\n'
        }

        buf[pos] = '\0';
        return false;
}

static bool at_cmd(const char *cmd, const char *expect, int timeout_ms) {
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
static void module_init(void) {
        at_cmd("AT+RESET\r\n", "+RESET", 1000);
        vTaskDelay(pdMS_TO_TICKS(1500));

        char cmd[64];

        snprintf(cmd, sizeof(cmd), "AT+ADDRESS=%d\r\n", LORA_ADDRESS_RECEIVER);
        at_cmd(cmd, "+OK", LORA_AT_TIMEOUT_MS);

        snprintf(cmd, sizeof(cmd), "AT+NETWORKID=%d\r\n", LORA_NETWORK_ID);
        at_cmd(cmd, "+OK", LORA_AT_TIMEOUT_MS);

        snprintf(cmd, sizeof(cmd), "AT+PARAMETER=%d,%d,%d,%d\r\n", BENCH_SF,
                 BENCH_BW, BENCH_CR, BENCH_PREAMBLE);
        at_cmd(cmd, "+OK", LORA_AT_TIMEOUT_MS);

        at_cmd("AT+CRFOP=7\r\n", "+OK", LORA_AT_TIMEOUT_MS);

        at_cmd("AT+ADDRESS?\r\n", "+ADDRESS=", LORA_AT_TIMEOUT_MS);
        at_cmd("AT+NETWORKID?\r\n", "+NETWORKID=", LORA_AT_TIMEOUT_MS);
        at_cmd("AT+PARAMETER?\r\n", "+PARAMETER=", LORA_AT_TIMEOUT_MS);
        at_cmd("AT+BAND?\r\n", "+BAND=", LORA_AT_TIMEOUT_MS);
        at_cmd("AT+CRFOP?\r\n", "+CRFOP=", LORA_AT_TIMEOUT_MS);
}

/* ------------------------------------------------------------------ */
/**
 * Parse +RCV=<src>,<len>,<data>,<rssi>,<snr> (some firmwares omit rssi/snr).
 * Returns src addr, data pointer into line, and len.
 *
 * This function also isolates payload by inserting a '\0' at data[len]
 * when the next char is a comma.
 */
static bool parse_rcv(char *line, int *src, char **data, int *len) {
        if (strncmp(line, "+RCV=", 5) != 0) return false;

        char *p = line + 5;

        // src
        char *c1 = strchr(p, ',');
        if (!c1) return false;
        *c1 = '\0';
        *src = atoi(p);

        // len
        char *p_len = c1 + 1;
        char *c2 = strchr(p_len, ',');
        if (!c2) return false;
        *c2 = '\0';

        *len = atoi(p_len);
        if (*len <= 0 || *len > LORA_MAX_PAYLOAD) return false;

        // payload start
        *data = c2 + 1;

        // validate payload is fully present in this line buffer
        char *end = *data + *len;
        if (*end == ',') {
                *end = '\0'; // isolate payload; safe for debug logging
        } else if (*end == '\0') {
                // ok: firmware omitted rssi/snr
        } else {
                // partial line or malformed
                return false;
        }

        return true;
}

static void send_echo(int dest, const char *data, int len) {
        static const int MAX_HDR = 0x20;
        char hdr[MAX_HDR];
        int hdr_len = snprintf(hdr, sizeof(hdr), "AT+SEND=%d,%d,", dest, len);

        uart_write_bytes(LORA_UART_NUM, hdr, hdr_len);
        uart_write_bytes(LORA_UART_NUM, data, len);
        uart_write_bytes(LORA_UART_NUM, "\r\n", 2);
}

static bool wait_for_start(char *line, int line_size)
{
	static const int listen_ms = 1000;

	static const bool ready = 0x0;
	static const bool wait  = 0x1;
	bool status = wait;

	int src, len;
	char *data;

	static int beacon_count = 0;
	beacon_count++;
	ESP_LOGW(TAG, "beacon #%d sending READY", beacon_count);

	const char *cmd = "AT+SEND=2,5,READY\r\n";
	uart_write_bytes(LORA_UART_NUM, cmd, strlen(cmd));

	int64_t deadline = esp_timer_get_time() + (int64_t)listen_ms * 1000;
	while (esp_timer_get_time() < deadline) {
		if (!readline(line, line_size, 200))
			continue;

		ESP_LOGW(TAG, "handshake rx: %s", line);

		bool valid_rcvp = parse_rcv(line, &src, &data, &len);
		bool valid_size = valid_rcvp && (len == 5);
		bool valid_ascii = valid_size && !(bool)memcmp(data, "START", 5);

		if (valid_ascii) {
			status = ready;
			break;
		}
	}

	return status;

}

/* ------------------------------------------------------------------ */

void lora_bench_task(void *arg)
{
        (void)arg;

        ESP_LOGI(TAG, "=== S3 echo slave ===");
        ESP_LOGI(TAG, "addr=%d net=%d SF=%d BW=%d", LORA_ADDRESS_RECEIVER,
                 LORA_NETWORK_ID, BENCH_SF, BENCH_BW);

        // --- Initial Module Setup ---
        uart_init_bench();
        vTaskDelay(pdMS_TO_TICKS(500));
        module_init();
        vTaskDelay(pdMS_TO_TICKS(500));

	ESP_LOGI(TAG, "waiting for handshake...");

	uart_flush_input(LORA_UART_NUM);
	vTaskDelay(pdMS_TO_TICKS(200));

        char line[300];
	while (wait_for_start(line, sizeof(line)));

	ESP_LOGI(TAG, "handshake complete — listening...");


        for (;;) {
                // --- STATE: LISTENING ---
                // At the start of every loop, ensure the UART driver is clean.
                // This is a more robust way to "flush".

                // Wait for a line to arrive. This is our blocking call.
                if (!readline(line, sizeof(line),
                              10000)) { // 10-second timeout
                        continue;       // Nothing heard, loop and wait again.
                }

                // We only care about "+RCV" packets. If it's anything else,
                // ignore it.
                if (strncmp(line, "+RCV=", 5) != 0) {
                        ESP_LOGD(TAG, "Ignoring non-RCV line: %s", line);
                        continue;
                }

                ESP_LOGI(TAG, "rx: %s", line);

                int src, len;
                char *data;
                if (!parse_rcv(line, &src, &data, &len)) {
                        ESP_LOGW(TAG, "Failed to parse valid RCV line.");
                        continue;
                }

                // --- STATE: ECHOING ---
                // We have a valid packet. Now we will echo it.
                // To prevent the UART driver from getting into a bad state,
                // we will completely reinstall it. This is the software
                // equivalent of "resetting the slave" for the communication
                // part.

		vTaskDelay(pdMS_TO_TICKS(15));
                // Now, with a pristine driver state, send the echo.
                send_echo(src, data, len);

        }
}

#else
void lora_bench_task(void *arg) { (void)arg; }
#endif
