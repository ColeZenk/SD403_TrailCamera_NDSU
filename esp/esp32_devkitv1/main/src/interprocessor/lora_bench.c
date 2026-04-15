/**
 * lora_bench.c - DevKit V1 (TX master)
 *
 * Send N packets -> wait for echo -> measure RTT/loss.
 * S3 must be running echo slave with matching RF config.
 */
#include "config.h"
#ifdef TEST_MODE_LORA_BENCH
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
static const char *TAG = "BENCH";

static const uint16_t LORA_BUF_SIZE = (1 << 9); // 10 bits (512)

/* ----------------------------------------------------- */
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
        snprintf(cmd, sizeof(cmd), "AT+ADDRESS=%d\r\n", LORA_ADDRESS_SENDER);
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

static bool parse_rcv(char *line, int *src, char **data, int *len) {
	static const uint LORA_MAX_PAYLOAD = 240;
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

/**
 * Send one packet.
 */
static void send_msg(uint32_t seq, int size) 
{
        static const int IDEAL_PAYLOAD = 0xF0;
        static const int MAX_HDR = 0x20;
	static const int num_hold = 9;
        uint8_t payload[IDEAL_PAYLOAD + 1];
        /* size = (size > IDEAL_PAYLOAD) ? IDEAL_PAYLOAD : size; */
        int mask = -(size > IDEAL_PAYLOAD);
        size = size ^ (mask & (IDEAL_PAYLOAD ^ size));
        memset(payload, 'A', size);
        char seq_str[num_hold];
	 snprintf(seq_str, num_hold, "%08lx", (unsigned long)seq);
	memcpy(payload, seq_str, num_hold-1);
        /* AT+SEND=<dest>,<len>,<data>\r\n */
        char hdr[MAX_HDR];
        int hdr_len = snprintf(hdr, sizeof(hdr), "AT+SEND=%d,%d,",
                               BENCH_DEST_ADDR, size);
        // uart_flush_input(LORA_UART_NUM); // <--- COMMENT OUT OR DELETE THIS
        // LINE
        uart_write_bytes(LORA_UART_NUM, hdr, hdr_len);
        uart_write_bytes(LORA_UART_NUM, (char *)payload, size);
        uart_write_bytes(LORA_UART_NUM, "\r\n", 2);
}

static int64_t wait_echo(int timeout_ms, int *rssi_out, int *snr_out) {
        int64_t t0 = esp_timer_get_time() >> 10;
        char line[300];
        bool ok_received = false;

        // Loop for the total duration of the timeout
        while ((esp_timer_get_time() >> 10) - t0 < timeout_ms) {
                // Use a short polling timeout for readline
                if (!readline(line, sizeof(line), 100)) {
                        continue; // Nothing received in this poll, try again
                }

                // Check if the line is the +OK from our own module
                if (strstr(line, "+OK")) {
                        ESP_LOGI(TAG, "AT+SEND ack: %s", line);
                        ok_received = true;
                        // Don't return yet; we still need the echo
                }

                // Check if the line is the +RCV echo from the slave
                else if (strncmp(line, "+RCV=", 5) == 0) {
                        ESP_LOGI(TAG, "rx: %s", line);

                        if (!ok_received) {
                                ESP_LOGW(TAG, "Echo (+RCV) arrived before "
                                              "local +OK ack!");
                        }
                        int64_t rtt = (esp_timer_get_time() >> 10) - t0;

                        // --- Your existing RSSI/SNR parsing logic ---
                        *rssi_out = 0;
                        *snr_out = 0;
                        char *p = line + 5;
                        char *c1 = strchr(p, ',');
                        if (c1) {
                                char *c2 = strchr(c1 + 1, ',');
                                if (c2) {
                                        // Important: The data payload can
                                        // contain anything. To find the RSSI,
                                        // we need to know the expected payload
                                        // length.
                                        char *after_payload =
                                            c2 + 1 + BENCH_PKT_SIZE;
                                        if (*after_payload == ',') {
                                                sscanf(after_payload + 1,
                                                       "%d,%d", rssi_out,
                                                       snr_out);
                                        }
                                }
                        }
                        // --- End of parsing logic ---

                        return rtt; // Success! We got the echo. Return the
                                    // RTT.
                }
        }

        // If we exit the loop, it means we timed out without receiving the
        // echo
        ESP_LOGW(TAG, "wait_echo timed out");
        if (!ok_received) {
                ESP_LOGW(TAG, "Never received +OK confirmation either.");
        }
        return -1; // Timeout
}

static bool wait_for_ready(char *line, int line_size)
{
	static const int handshake_listen_ms = 500;

	static const bool ready = 0x0;
	static const bool wait  = 0x1;
	bool status = wait;


	int src, len;
	char *data;

	
	bool valid_line = readline(line, line_size, handshake_listen_ms);
	bool valid_rcvp = valid_line && parse_rcv(line,&src,&data,&len);
	bool valid_size = valid_rcvp && (len == 5);
	bool valid_ascii = valid_size && !(bool)memcmp(data, "READY", 5);

	if (valid_line)
		ESP_LOGI(TAG, "handshake rx: %s (match=%d)", line, valid_ascii);

	if(valid_ascii) status = ready;

	return status;

}


/* ------------------------------------------------------------------ */
void lora_bench_task(void *arg) {
        ESP_LOGI(TAG, "=== DevKit TX bench ===");
        ESP_LOGI(TAG, "addr=%d dest=%d net=%d SF=%d BW=%d",
                 LORA_ADDRESS_SENDER, BENCH_DEST_ADDR, LORA_NETWORK_ID,
                 BENCH_SF, BENCH_BW);

        uart_init_bench();
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

	vTaskDelay(pdMS_TO_TICKS(100));  // settle


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

#else
void lora_bench_task(void *arg) { (void)arg; }
#endif
