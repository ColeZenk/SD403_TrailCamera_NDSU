/**
 * @file lora_uart.c
 * @brief LoRa UART Interface - ESP32 DevKitV1 Receiver
 * @author ESP32 Image Pipeline Team
 *
 * Implements UART communication with LoRa module for wireless trigger reception.
 * Configures module parameters and handles incoming trigger messages.
 *
 * @scope
 * File covers LoRa module initialization, AT command configuration, and
 * message reception with optional LED feedback for testing.
 */

/*************************************************************************
 INCLUDES
 *************************************************************************/
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"

#include "isr_signals.h"
#include "lora_uart.h"

#include <string.h>

/*************************************************************************
 CONFIGURATION
 *************************************************************************/
#define DEV_TEST_LORA_UART    1

#define LORA_UART             UART_NUM_2
#define LORA_TX_PIN           GPIO_NUM_17
#define LORA_RX_PIN           GPIO_NUM_16
#define LORA_BAUD             115200

#define AT_RESPONSE_BUF_SIZE  64
#define AT_CMD_TIMEOUT_MS     500
#define AT_CMD_DELAY_MS       300

#ifdef DEV_TEST_LORA_UART
  #define LED_PIN             GPIO_NUM_2
  #define LED_BLINK_COUNT     3
  #define LED_BLINK_ON_MS     200
  #define LED_BLINK_OFF_MS    200
#endif

static const char *TAG = "LORA";

/*************************************************************************
 FILE SCOPED FUNCTIONS
 *************************************************************************/
static esp_err_t sendATCommand(const char *command, const char *description);
static esp_err_t queryATParameter(const char *query, const char *param_name);
static void handleTriggerReceived(void);

#ifdef DEV_TEST_LORA_UART
static esp_err_t initTestLED(void);
static void blinkLED(void);
static void lora_uart_devtest(const uint8_t *buf);
#endif

/*************************************************************************
 AT COMMAND HELPERS
 *************************************************************************/

/**
 * @brief Send AT command to LoRa module and log response
 *
 * @param command AT command string (must include \r\n)
 * @param description Human-readable description for logging
 * @return ESP_OK on success, error code otherwise
 */
static esp_err_t sendATCommand(const char *command, const char *description)
{
    uint8_t buf[AT_RESPONSE_BUF_SIZE];
    int len;

    uart_flush(LORA_UART);
    uart_write_bytes(LORA_UART, command, strlen(command));
    vTaskDelay(pdMS_TO_TICKS(AT_CMD_DELAY_MS));

    len = uart_read_bytes(LORA_UART, buf, sizeof(buf) - 1,
                          pdMS_TO_TICKS(AT_CMD_TIMEOUT_MS));

    if (len > 0) {
        buf[len] = '\0';
        ESP_LOGI(TAG, "%s: %s", description, buf);
        return ESP_OK;
    }

    ESP_LOGW(TAG, "%s: No response", description);
    return ESP_ERR_TIMEOUT;
}

/**
 * @brief Query AT parameter and log result
 *
 * @param query AT query string (must include \r\n)
 * @param param_name Parameter name for logging
 * @return ESP_OK on success, error code otherwise
 */
static esp_err_t queryATParameter(const char *query, const char *param_name)
{
    uint8_t buf[AT_RESPONSE_BUF_SIZE];
    int len;

    uart_flush(LORA_UART);
    uart_write_bytes(LORA_UART, query, strlen(query));
    vTaskDelay(pdMS_TO_TICKS(AT_CMD_DELAY_MS));

    len = uart_read_bytes(LORA_UART, buf, sizeof(buf) - 1,
                          pdMS_TO_TICKS(AT_CMD_TIMEOUT_MS));

    if (len > 0) {
        buf[len] = '\0';
        ESP_LOGI(TAG, "%s: %s", param_name, buf);
        return ESP_OK;
    }

    ESP_LOGW(TAG, "%s: Query failed", param_name);
    return ESP_ERR_TIMEOUT;
}

/*************************************************************************
 TRIGGER HANDLING
 *************************************************************************/

/**
 * @brief Handle received trigger message
 *
 * Processes trigger reception and provides visual feedback in test mode.
 */
static void handleTriggerReceived(void)
{
    ESP_LOGI(TAG, "TRIGGER RECEIVED!");
    ISR_OnLoRaTrigger();
}

/*************************************************************************
 TEST MODE SUPPORT
 *************************************************************************/

#ifdef DEV_TEST_LORA_UART

static esp_err_t initTestLED(void)
{
    gpio_config_t led_conf = {
        .pin_bit_mask = (1ULL << LED_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    esp_err_t ret = gpio_config(&led_conf);
    if (ret == ESP_OK) {
        gpio_set_level(LED_PIN, 0);
    }

    return ret;
}

static void blinkLED(void)
{
    for (int i = 0; i < LED_BLINK_COUNT; i++) {
        gpio_set_level(LED_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(LED_BLINK_ON_MS));
        gpio_set_level(LED_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(LED_BLINK_OFF_MS));
    }
}

#endif // DEV_TEST_LORA_UART

/*************************************************************************
 PUBLIC INTERFACE
 *************************************************************************/

/**
 * @brief Initialize LoRa UART interface and configure module
 *
 * Configures UART peripheral, sets up LoRa module parameters via AT commands,
 * and optionally initializes test LED.
 *
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t lora_init(void)
{
    esp_err_t ret;

#ifdef DEV_TEST_LORA_UART
    ret = initTestLED();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "LED init failed: %s", esp_err_to_name(ret));
    }
#endif

    // Configure UART
    uart_config_t cfg = {
        .baud_rate = LORA_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ret = uart_driver_install(LORA_UART, 2048, 2048, 0, NULL, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART driver install failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = uart_param_config(LORA_UART, &cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART param config failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = uart_set_pin(LORA_UART, LORA_TX_PIN, LORA_RX_PIN, -1, -1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART pin config failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Allow module to boot
    vTaskDelay(pdMS_TO_TICKS(500));

    // Configure LoRa module via AT commands
    ESP_LOGI(TAG, "Configuring LoRa module...");

    sendATCommand("AT+ADDRESS=2\r\n", "ADDRESS set");
    sendATCommand("AT+NETWORKID=6\r\n", "NETWORKID set");

    // Verify configuration
    queryATParameter("AT+ADDRESS?\r\n", "ADDRESS");
    queryATParameter("AT+NETWORKID?\r\n", "NETWORKID");
    queryATParameter("AT+PARAMETER?\r\n", "PARAMETER");
    queryATParameter("AT+BAND?\r\n", "BAND");

    ESP_LOGI(TAG, "LoRa init complete");
    return ESP_OK;
}

/**
 * @brief Send trigger command to remote LoRa device
 *
 * Sends a trigger message to address 1 (ESP32-S3 camera module).
 */
void lora_send_trigger(void)
{
    const char *cmd = "AT+SEND=1,1,T\r\n";  // Send to S3 (addr 1)
    uart_write_bytes(LORA_UART, cmd, strlen(cmd));
    ESP_LOGI(TAG, "Trigger sent");
}

/**
 * @brief LoRa receive task - monitors for incoming trigger messages
 *
 * Continuously polls UART for LoRa messages and processes trigger commands.
 * Provides periodic status logging.
 *
 * @param arg Unused task parameter
 */
void lora_receive_task(void *arg)
{
    uint8_t buf[256];
    int poll_count = 0;

    ESP_LOGI(TAG, "LoRa RX task started, waiting for triggers...");

    for (;;) {
        int len = uart_read_bytes(LORA_UART, buf, sizeof(buf) - 1,
                                  pdMS_TO_TICKS(100));

        // Periodic status update
        poll_count++;
        if (poll_count % 100 == 0) {  // Every 10 seconds
            ESP_LOGI(TAG, "RX task alive - %d polls, listening...", poll_count);
        }

        if (len > 0) {
            buf[len] = '\0';
            ESP_LOGI(TAG, "RX raw (%d bytes): [%s]", len, buf);

#ifdef DEV_TEST_LORA_UART
            lora_uart_devtest(buf);
#endif
        }
    }
}

#ifdef DEV_TEST_LORA_UART
static void lora_uart_devtest(const uint8_t *buf)
{
    if (strstr((const char*)buf, "+RCV=") != NULL) {
        ESP_LOGI(TAG, "GOT +RCV MESSAGE!");

        if (strstr((const char*)buf, ",T,") != NULL ||
            strstr((const char*)buf, ",0,") != NULL ||
            strstr((const char*)buf, ",01,") != NULL) {
            handleTriggerReceived();
        }
    }
}
#endif
