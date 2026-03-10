/*
 * ESP32-S3 Handheld LoRa Transmitter
 * Press BOOT button to send trigger to trail camera
 */

#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"

static const char *TAG = "LORA_TX";

#define LORA_UART       UART_NUM_1
#define LORA_TX_PIN     GPIO_NUM_17
#define LORA_RX_PIN     GPIO_NUM_18
#define LORA_BAUD       115200

#define BOOT_BUTTON     GPIO_NUM_9  // Boot button on S3 DevKit

void lora_init(void) {
    uart_config_t cfg = {
        .baud_rate = LORA_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    uart_driver_install(LORA_UART, 2048, 2048, 0, NULL, 0);
    uart_param_config(LORA_UART, &cfg);
    uart_set_pin(LORA_UART, LORA_TX_PIN, LORA_RX_PIN, -1, -1);
    
    vTaskDelay(pdMS_TO_TICKS(500));  // Let module boot
    
    uint8_t buf[64];
    int len;
    
    // Set ADDRESS with verification
    uart_flush(LORA_UART);
    uart_write_bytes(LORA_UART, "AT+ADDRESS=1\r\n", 14);
    vTaskDelay(pdMS_TO_TICKS(300));
    len = uart_read_bytes(LORA_UART, buf, sizeof(buf)-1, pdMS_TO_TICKS(500));
    if (len > 0) { buf[len] = '\0'; ESP_LOGI(TAG, "ADDRESS set: %s", buf); }
    
    // Set NETWORKID with verification
    uart_flush(LORA_UART);
    uart_write_bytes(LORA_UART, "AT+NETWORKID=6\r\n", 16);
    vTaskDelay(pdMS_TO_TICKS(300));
    len = uart_read_bytes(LORA_UART, buf, sizeof(buf)-1, pdMS_TO_TICKS(500));
    if (len > 0) { buf[len] = '\0'; ESP_LOGI(TAG, "NETWORKID set: %s", buf); }
    
    // Query back to verify
    uart_flush(LORA_UART);
    uart_write_bytes(LORA_UART, "AT+ADDRESS?\r\n", 13);
    vTaskDelay(pdMS_TO_TICKS(300));
    len = uart_read_bytes(LORA_UART, buf, sizeof(buf)-1, pdMS_TO_TICKS(500));
    if (len > 0) { buf[len] = '\0'; ESP_LOGI(TAG, "ADDRESS: %s", buf); }
    
    uart_flush(LORA_UART);
    uart_write_bytes(LORA_UART, "AT+NETWORKID?\r\n", 15);
    vTaskDelay(pdMS_TO_TICKS(300));
    len = uart_read_bytes(LORA_UART, buf, sizeof(buf)-1, pdMS_TO_TICKS(500));
    if (len > 0) { buf[len] = '\0'; ESP_LOGI(TAG, "NETWORKID: %s", buf); }
    
    uart_flush(LORA_UART);
    uart_write_bytes(LORA_UART, "AT+PARAMETER?\r\n", 15);
    vTaskDelay(pdMS_TO_TICKS(300));
    len = uart_read_bytes(LORA_UART, buf, sizeof(buf)-1, pdMS_TO_TICKS(500));
    if (len > 0) { buf[len] = '\0'; ESP_LOGI(TAG, "PARAMETER: %s", buf); }
    
    uart_flush(LORA_UART);
    uart_write_bytes(LORA_UART, "AT+BAND?\r\n", 10);
    vTaskDelay(pdMS_TO_TICKS(300));
    len = uart_read_bytes(LORA_UART, buf, sizeof(buf)-1, pdMS_TO_TICKS(500));
    if (len > 0) { buf[len] = '\0'; ESP_LOGI(TAG, "BAND: %s", buf); }
    
    ESP_LOGI(TAG, "LoRa init complete");
}

void lora_send_trigger(void) {
    // Send 'T' to address 2 (DevKitV1 trail camera)
    const char *cmd = "AT+SEND=2,1,T\r\n";
    uart_write_bytes(LORA_UART, cmd, strlen(cmd));
    ESP_LOGI(TAG, "Trigger sent to camera!");
    
    // Wait for +OK response
    uint8_t buf[64];
    int len = uart_read_bytes(LORA_UART, buf, sizeof(buf)-1, pdMS_TO_TICKS(300));
    if (len > 0) {
        buf[len] = '\0';
        ESP_LOGI(TAG, "Response: %s", buf);
    }
}

void button_task(void *arg) {
    // Configure boot button
    gpio_config_t btn = {
        .pin_bit_mask = (1ULL << BOOT_BUTTON),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&btn);
    
    ESP_LOGI(TAG, "Button task ready - press BOOT to trigger camera");
    
    bool last_state = 1;  // Button is active-low
    
    while(1) {
        bool current_state = gpio_get_level(BOOT_BUTTON);
        
        // Detect button press (transition from high to low)
        if (last_state == 1 && current_state == 0) {
            ESP_LOGI(TAG, "Button pressed!");
            lora_send_trigger();
            vTaskDelay(pdMS_TO_TICKS(300));  // Debounce
        }
        
        last_state = current_state;
        vTaskDelay(pdMS_TO_TICKS(50));  // 50ms polling
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "ESP32-S3 Handheld LoRa Transmitter");
    
    lora_init();
    xTaskCreate(button_task, "button", 4096, NULL, 5, NULL);
}
