/*******************************************************************************
 * lora.c
 *
 * A file containing all lora related functions/init
 ******************************************************************************/

#include <stdio.h>
#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_freertos_hooks.h"
#include "driver/gpio.h"
#include "driver/uart.h"

#include "config.h"   /* defines TEST_MODE_LORA_BENCH before lora.h needs it */
#include "lora.h"

static const char *TAG = "LORA";

/*******************************************************************************
 * LoRa Init
 *
 * In bench mode: sets AT+PARAMETER to match DevKitV1 bench config.
 * In normal mode: uses LORA_ADDRESS / LORA_NETWORK_ID from config.h.
 ******************************************************************************/

/* Read one \r\n-terminated line from the LoRa UART, returns true if got one.
 *
 * Uses xTaskGetTickCount() rather than esp_timer_get_time().  On ESP32-S3
 * the SYSTIMER latch used by esp_timer_get_time() can stall indefinitely when
 * the UART RX ISR fires continuously, making the timeout never expire.
 * The FreeRTOS tick count is incremented by the tick ISR and is always
 * reliable.  taskYIELD() after each byte ensures IDLE runs even when the ring
 * buffer is never empty, preventing the 300 ms interrupt-WDT threshold from
 * being exceeded. */
static bool lora_readline(char *buf, int maxlen, int timeout_ms)
{
    int pos = 0;
    TickType_t t0 = xTaskGetTickCount();
    while (xTaskGetTickCount() - t0 < pdMS_TO_TICKS(timeout_ms)) {
        uint8_t b;
        int n = uart_read_bytes(LORA_UART_NUM, &b, 1, pdMS_TO_TICKS(10));
        taskYIELD(); /* let IDLE run even during continuous data flow */
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

/* Send AT command, log response line(s) for timeout_ms */
static void lora_at(const char *cmd, int timeout_ms)
{
    uart_flush_input(LORA_UART_NUM);
    uart_write_bytes(LORA_UART_NUM, cmd, strlen(cmd));
    char line[128];
    TickType_t t0 = xTaskGetTickCount();
    while ((TickType_t)(xTaskGetTickCount() - t0) < pdMS_TO_TICKS(timeout_ms)) {
        if (lora_readline(line, sizeof(line), 100))
            ESP_LOGI(TAG, "  %s -> %s", cmd[2] == '+' ? cmd + 3 : cmd, line);
    }
}

static void lora_init(void)
{
    uart_config_t cfg = {
        .baud_rate  = LORA_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_driver_install(LORA_UART_NUM, LORA_BUF_SIZE, LORA_BUF_SIZE, 0, NULL, ESP_INTR_FLAG_IRAM);
    uart_param_config(LORA_UART_NUM, &cfg);
    uart_set_pin(LORA_UART_NUM, LORA_TX_PIN, LORA_RX_PIN, -1, -1);
    vTaskDelay(pdMS_TO_TICKS(500));

    /* Query pre-existing state */
    lora_at("AT+ADDRESS?\r\n",   400);
    lora_at("AT+NETWORKID?\r\n", 400);
    lora_at("AT+PARAMETER?\r\n", 400);

    /* Reset to clean state */
    lora_at("AT+RESET\r\n", 400);
    vTaskDelay(pdMS_TO_TICKS(1500));
    lora_at("AT\r\n", 400);

    /* Configure */
    char cmd[64];
    snprintf(cmd, sizeof(cmd), "AT+ADDRESS=%d\r\n", LORA_ADDRESS);
    lora_at(cmd, 400);

    snprintf(cmd, sizeof(cmd), "AT+NETWORKID=%d\r\n", LORA_NETWORK_ID);
    lora_at(cmd, 400);

#ifdef TEST_MODE_LORA_BENCH
    snprintf(cmd, sizeof(cmd), "AT+PARAMETER=%d,%d,%d,%d\r\n",
             BENCH_SF, BENCH_BW, BENCH_CR, BENCH_PREAMBLE);
    lora_at(cmd, 400);
#endif

    /* Confirm what actually stuck */
    lora_at("AT+ADDRESS?\r\n",   400);
    lora_at("AT+NETWORKID?\r\n", 400);
    lora_at("AT+PARAMETER?\r\n", 400);

    ESP_LOGI(TAG, "LoRa init done");
}

/*******************************************************************************
 * LoRa Receive Task (Core 0)
 *
 * The LoRa module emits "+RCV=<sender>,<len>,<data>,<rssi>,<snr>\r\n"
 * We extract the raw <data> bytes, reconstruct, broadcast over WebSocket.
 ******************************************************************************/

static void lora_task(void *arg)
{
    static uint8_t line[64];      /* just for the header: "+RCV=addr,len," */
    static uint8_t payload[512];
    int line_len = 0;
    bool in_header = true;

    ESP_LOGI(TAG, "LoRa task started on core %d", xPortGetCoreID());
    uart_flush_input(LORA_UART_NUM);

    for (;;) {
        uint8_t byte;
        int n = uart_read_bytes(LORA_UART_NUM, &byte, 1, pdMS_TO_TICKS(100));
        taskYIELD();
        if (n <= 0) continue;

        if (in_header) {
            if (byte == '\n') {
                /* Short line with no binary data — reset */
                line[line_len] = '\0';
                line_len = 0;
                continue;
            }
            if (byte == '\r') continue;

            if (line_len < (int)sizeof(line) - 1)
                line[line_len++] = byte;

            /* Wait until we have "+RCV=<addr>,<len>," — detected by 3rd comma */
            int commas = 0;
            for (int i = 0; i < line_len; i++)
                if (line[i] == ',') commas++;

            if (commas < 2) continue;   /* not enough header yet */

            /* We have the full header prefix — parse it */
            line[line_len] = '\0';
            char *ptr    = (char *)line + 5;   /* skip "+RCV=" */
            char *comma1 = strchr(ptr, ',');
            char *comma2 = comma1 ? strchr(comma1 + 1, ',') : NULL;
            if (!comma1 || !comma2) { line_len = 0; continue; }

            int sender  = atoi(ptr);
            int pkt_len = atoi(comma1 + 1);

            if (pkt_len <= 0 || pkt_len > (int)sizeof(payload)) {
                ESP_LOGW(TAG, "bad pkt_len=%d", pkt_len);
                line_len = 0;
                continue;
            }

            /* --- Read exactly pkt_len raw bytes --- */
            size_t got = 0;
            while ((int)got < pkt_len) {
                int r = uart_read_bytes(LORA_UART_NUM, payload + got,
                                        pkt_len - got, pdMS_TO_TICKS(500));
                if (r <= 0) break;
                got += r;
            }

            if ((int)got != pkt_len) {
                ESP_LOGW(TAG, "payload read short: got=%d want=%d", (int)got, pkt_len);
                line_len = 0;
                continue;
            }

            /* --- Read trailing ",<rssi>,<snr>\r\n" --- */
            char trailer[32];
            int tlen = 0;
            while (tlen < (int)sizeof(trailer) - 1) {
                uint8_t tb;
                int r = uart_read_bytes(LORA_UART_NUM, &tb, 1, pdMS_TO_TICKS(200));
                if (r <= 0) break;
                if (tb == '\n') break;
                if (tb != '\r') trailer[tlen++] = (char)tb;
            }
            trailer[tlen] = '\0';

            int rssi = 0, snr = 0;
            sscanf(trailer, ",%d,%d", &rssi, &snr);

#ifndef TEST_MODE_LORA_BENCH
            ESP_LOGI(TAG, "LoRa pkt: sender=%d len=%d RSSI=%d SNR=%d",
                     sender, pkt_len, rssi, snr);
#endif

#ifdef TEST_MODE_LORA_BENCH
            ESP_LOGI(TAG, "RCV sender=%d len=%d rssi=%d snr=%d",
                     sender, pkt_len, rssi, snr);
            vTaskDelay(pdMS_TO_TICKS(15));
            char echo_hdr[32];
            snprintf(echo_hdr, sizeof(echo_hdr),
                     "AT+SEND=%d,%d,", sender, pkt_len);
            uart_write_bytes(LORA_UART_NUM, echo_hdr, strlen(echo_hdr));
            uart_write_bytes(LORA_UART_NUM, payload, pkt_len);
            uart_write_bytes(LORA_UART_NUM, "\r\n", 2);
            /* Log module response to echo attempt — critical for diagnosing ERR=x */
            {
                char resp[64];
                if (lora_readline(resp, sizeof(resp), 400))
                    ESP_LOGI(TAG, "  echo resp: %s", resp);
                else
                    ESP_LOGW(TAG, "  echo resp: (timeout)");
            }
#else
            uint8_t pos    = 0;
            uint8_t *frame = reconstruct(payload, pkt_len, &pos);
            if (frame) {
                ws_broadcast(pos, frame_seq++, frame, FRAME_BYTES);
            }
#endif
            line_len = 0;

        } /* in_header */
    }
}

/*******************************************************************************
 * Public API
 ******************************************************************************/

/* Prevent IDLE from executing WAITI on ESP32-S3 with octal PSRAM.
 * esp_vApplicationIdleHook() calls esp_cpu_wait_for_intr() (WAITI) whenever
 * all registered idle hooks return true.  On ESP32-S3, WAITI while the MSPI
 * bus is active causes a deadlock: the CPU waits for an interrupt, but the
 * pending cache-miss that would SERVICE the interrupt needs MSPI, which is
 * stalled by the WAITI.  Tick ISR starves → IWDT fires.
 * Returning false from this hook keeps can_go_idle=false, so the WAITI branch
 * is never reached.  The CPU busy-waits in IDLE instead — harmless. */
static bool IRAM_ATTR no_waiti_idle_hook(void) { return false; }

void lora_start(void)
{
    esp_register_freertos_idle_hook(no_waiti_idle_hook);
    lora_init();
    xTaskCreatePinnedToCore(lora_task, "lora_rx",
                            STACK_LORA, NULL, PRIO_LORA, NULL, CORE_LORA);
}
