/**
 * lora_bench.c - ESP32-S3 (echo slave)
 *
 * Listen for packets from DevKit, echo back verbatim.
 * DevKit must be running TX master with matching RF config.
 */
#include "config.h"
#ifdef TEST_MODE_LORA_BENCH
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
static const char *TAG = "ECHO";

/* ------------------------------------------------------------------ */
static void uart_init_bench(void)
{
	uart_config_t cfg = {.baud_rate = LORA_BAUD_RATE,
	                     .data_bits = UART_DATA_8_BITS,
	                     .parity    = UART_PARITY_DISABLE,
	                     .stop_bits = UART_STOP_BITS_1,
	                     .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
	                     .source_clk= UART_SCLK_DEFAULT,
	                    };

	uart_driver_install(LORA_UART_NUM, LORA_BUF_SIZE,
	                    LORA_BUF_SIZE, 0, NULL, 0);
	uart_param_config(LORA_UART_NUM, &cfg);
	uart_set_pin(LORA_UART_NUM, LORA_PIN_TX, LORA_PIN_RX,
	             -1, -1);
	vTaskDelay(pdMS_TO_TICKS(LORA_INIT_DELAY_MS));
}

static bool readline(char *buf, int lenmax, int timeout_ms)
{
	int pos = 0;
	int64_t deadline = esp_timer_get_time() + (int64_t)timeout_ms * 1000;
	while (esp_timer_get_time() < deadline) {
		uint8_t b;
		if (uart_read_bytes(LORA_UART_NUM, &b, 1, pdMS_TO_TICKS(100)) <= 0)
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

static bool at_cmd(const char *cmd, const char *expect,
                   int timeout_ms)
{
	static const int POLL_MS = 50;
	uart_flush_input(LORA_UART_NUM);
	uart_write_bytes(LORA_UART_NUM, cmd, strlen(cmd));
	
	char line[INT8_MAX+1];
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
	
	snprintf(cmd, sizeof(cmd), "AT+ADDRESS=%d\r\n", \
	         LORA_ADDRESS_RECEIVER);
	at_cmd(cmd, "+OK", LORA_AT_TIMEOUT_MS);
	
	snprintf(cmd, sizeof(cmd), "AT+NETWORKID=%d\r\n", \
	         LORA_NETWORK_ID);
	at_cmd(cmd, "+OK", LORA_AT_TIMEOUT_MS);
	
	snprintf(cmd, sizeof(cmd), "AT+PARAMETER=%d,%d,%d,%d\r\n",
	         BENCH_SF, BENCH_BW, BENCH_CR, BENCH_PREAMBLE);
	at_cmd(cmd, "+OK", LORA_AT_TIMEOUT_MS);
	
	at_cmd("AT+ADDRESS?\r\n",   "+ADDRESS=",    LORA_AT_TIMEOUT_MS);
	at_cmd("AT+NETWORKID?\r\n", "+NETWORKID=",  LORA_AT_TIMEOUT_MS);
	at_cmd("AT+PARAMETER?\r\n", "+PARAMETER=",  LORA_AT_TIMEOUT_MS);
	at_cmd("AT+BAND?\r\n",      "+BAND=",       LORA_AT_TIMEOUT_MS);
}

/* ------------------------------------------------------------------ */

/**
 * Parse +RCV=<src>,<len>,<data>,<rssi>,<snr>
 * Returns src addr, data pointer into line, and length.
 */
static bool parse_rcv(char *line, int *src, char **data, int *len)
{
	char *p = line + 5;
	char *c1 = strchr(p, ',');
	if (!c1) return false;
	*c1 = '\0';
	*src = atoi(p);
	
	char *c2 = strchr(c1+1, ',');
	if (!c2) return false;
	*c2 = '\0';
	
	*len = atoi(c1+1);
	if (*len <= 0 \
	 || *len > LORA_MAX_PAYLOAD) return false;
	
	*data = c2 + 1;
	return true;
}

static void send_echo(int dest, const char *data, int len)
{
	static const int MAX_HDR = 0x20;
	char hdr[MAX_HDR];
	int hdr_len = snprintf(hdr, sizeof(hdr), "AT+SEND=%d,%d,", \
	                       dest, len);
	uart_write_bytes(LORA_UART_NUM, hdr, hdr_len);
	uart_write_bytes(LORA_UART_NUM, data, len);
	uart_write_bytes(LORA_UART_NUM, "\r\n", 2);
}

/* ------------------------------------------------------------------ */

void lora_bench_task(void *arg)
{
	ESP_LOGI(TAG, "=== S3 echo slave ===");
	ESP_LOGI(TAG, "addr=%d net=%d SF=%d BW=%d",
	         LORA_ADDRESS_RECEIVER, LORA_NETWORK_ID,
	         BENCH_SF, BENCH_BW);
	
	uart_init_bench();
	vTaskDelay(pdMS_TO_TICKS(500));
	module_init();
	vTaskDelay(pdMS_TO_TICKS(500));
	
	ESP_LOGI(TAG, "listening...");
	
	char line[300];
	uint32_t count = 0;
	
	for (;;) {
		if (!readline(line, sizeof(line), 5000)) continue;
		ESP_LOGI(TAG, "rx: %s", line);
		if (strncmp(line, "+RCV=", 5) != 0) continue;
		
		int src, len;
		char *data;
		if (!parse_rcv(line, &src, &data, &len)) continue;
		
		int actual = strlen(data);
		ESP_LOGI(TAG, "parsed len=%d actual=%d", len, actual);
		send_echo(src, data, actual);
		/* consume +OK */
		if (readline(line, sizeof(line), LORA_AT_TIMEOUT_MS))
			ESP_LOGI(TAG, "echo ack: %s", line);
		else
			ESP_LOGW(TAG, "echo ack: no response");
		
		if (++count == 10){
		    ESP_LOGI(TAG, "echoed %lu packets", count);
			count = 0;
		}
	}
}

#else
void lora_bench_task(void *arg) { (void)arg; }
#endif
