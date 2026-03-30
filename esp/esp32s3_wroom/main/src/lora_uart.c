/**
 * lora_uart.c — LoRa UART interface (REYAX RYLR896)
 *
 * AT command config at init, then listens for trigger messages.
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"

#include "lora_uart.h"
#include <string.h>

static const char *TAG = "LORA";

/*******************************************************************************
 * Config
 ******************************************************************************/

#define LORA_UART       UART_NUM_2
#define LORA_TX         GPIO_NUM_17
#define LORA_RX         GPIO_NUM_18
#define LORA_BAUD_RATE       115200

#define AT_BUF_SIZE     64
#define AT_TIMEOUT_MS   500
#define AT_DELAY_MS     300


/*******************************************************************************
 * AT Commands
 ******************************************************************************/

static void at_send(const char *cmd, const char *label)
{
    uint8_t buf[AT_BUF_SIZE];

    uart_flush(LORA_UART);
    uart_write_bytes(LORA_UART, cmd, strlen(cmd));
    vTaskDelay(pdMS_TO_TICKS(AT_DELAY_MS));

    int len = uart_read_bytes(LORA_UART, buf, sizeof(buf) - 1,
                              pdMS_TO_TICKS(AT_TIMEOUT_MS));
    if (len > 0) {
        buf[len] = '\0';
        ESP_LOGI(TAG, "%s: %s", label, buf);
    } else {
        ESP_LOGW(TAG, "%s: no response", label);
    }
}
/*******************************************************************************
 * Public Interface
 ******************************************************************************/

esp_err_t lora_init(void)
{
    uart_config_t cfg = {
        .baud_rate  = LORA_BAUD_RATE,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t ret;

    ret = uart_driver_install(LORA_UART, 2048, 2048, 0, NULL, 0);
    if (ret != ESP_OK) return ret;

    ret = uart_param_config(LORA_UART, &cfg);
    if (ret != ESP_OK) return ret;

    ret = uart_set_pin(LORA_UART, LORA_TX, LORA_RX, -1, -1);
    if (ret != ESP_OK) return ret;

    vTaskDelay(pdMS_TO_TICKS(500));

    /* Configure module */
    at_send("AT+ADDRESS=2\r\n",   "ADDRESS set");
    at_send("AT+NETWORKID=6\r\n", "NETWORKID set");

    /* Verify */
    at_send("AT+ADDRESS?\r\n",   "ADDRESS");
    at_send("AT+NETWORKID?\r\n", "NETWORKID");
    at_send("AT+PARAMETER?\r\n", "PARAMETER");
    at_send("AT+BAND?\r\n",      "BAND");

    ESP_LOGI(TAG, "init complete");
    return ESP_OK;
}

void lora_send_trigger(void)
{
    const char *cmd = "AT+SEND=1,1,T\r\n";
    uart_write_bytes(LORA_UART, cmd, strlen(cmd));
    ESP_LOGI(TAG, "trigger sent");
}

void lora_receive_task(void *arg)
{
    uint8_t buf[UINT8_MAX + 1];
    int polls = 0;

    ESP_LOGI(TAG, "RX task started");

    for (;;) {
        int len = uart_read_bytes(LORA_UART, buf, sizeof(buf) - 1,
                                  pdMS_TO_TICKS(100));

        if (!(++polls & 0x64)) {
            ESP_LOGI(TAG, "listening... (%d polls)", polls);
        }

        if (len <= 0) continue;

        buf[len] = '\0';
        ESP_LOGI(TAG, "RX (%d bytes): [%s]", len, buf);

    }
}
