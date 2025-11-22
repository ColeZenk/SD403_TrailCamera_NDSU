/*
 * UART Interface Implementation
 * ESP32-S3 WROOM Master - UART communication with ESP32-CAM
 */

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "uart_interface.h"

static const char *TAG = "UART_INTERFACE";

void uart_interface_init(void) {
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    // Install UART driver
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, UART_BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, UART_TX_PIN, UART_RX_PIN, 
                                  UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    
    ESP_LOGI(TAG, "UART initialized: TX=%d, RX=%d, Baud=%d", 
             UART_TX_PIN, UART_RX_PIN, UART_BAUD_RATE);
}

void uart_test_connection(void) {
    ESP_LOGI(TAG, "Testing UART connection...");
    
    const char *test_msg = "PING\n";
    uart_write_bytes(UART_PORT_NUM, test_msg, strlen(test_msg));
    ESP_LOGI(TAG, "Sent: PING");
    
    uint8_t data[128];
    int len = uart_read_bytes(UART_PORT_NUM, data, sizeof(data) - 1, pdMS_TO_TICKS(5000));
    
    if (len > 0) {
        data[len] = '\0';
        ESP_LOGI(TAG, "Received: %s", (char *)data);
        ESP_LOGI(TAG, "UART connection OK!");
    } else {
        ESP_LOGW(TAG, "No response received. Check connections.");
    }
}

static void uart_passthrough_task(void *arg) {
    uint8_t data[UART_BUF_SIZE];
    
    ESP_LOGI(TAG, "UART passthrough task started");
    
    while (1) {
        // Forward data from USB Serial (UART0) to ESP32-CAM (UART1)
        int len = uart_read_bytes(UART_NUM_0, data, UART_BUF_SIZE, pdMS_TO_TICKS(20));
        if (len > 0) {
            uart_write_bytes(UART_PORT_NUM, (const char *)data, len);
        }
        
        // Forward data from ESP32-CAM (UART1) to USB Serial (UART0)
        len = uart_read_bytes(UART_PORT_NUM, data, UART_BUF_SIZE, pdMS_TO_TICKS(20));
        if (len > 0) {
            uart_write_bytes(UART_NUM_0, (const char *)data, len);
        }
    }
}

void uart_start_passthrough_task(void) {
    xTaskCreate(uart_passthrough_task, "uart_passthrough", 4096, NULL, 5, NULL);
    ESP_LOGI(TAG, "Passthrough task created");
}

int uart_write_to_cam(const char *data, size_t len) {
    return uart_write_bytes(UART_PORT_NUM, data, len);
}

int uart_read_from_cam(uint8_t *data, size_t max_len, uint32_t timeout_ms) {
    return uart_read_bytes(UART_PORT_NUM, data, max_len, pdMS_TO_TICKS(timeout_ms));
}