// LoRa_UART_Interface.c - ESP32 DevKitV1 (Receiver)
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <string.h>

#define DEV_TEST_LORA_UART 1

#define LORA_UART       UART_NUM_2
#define LORA_TX_PIN     GPIO_NUM_17
#define LORA_RX_PIN     GPIO_NUM_16
#define LORA_BAUD       115200

#ifdef DEV_TEST_LORA_UART
  #define LED_PIN         GPIO_NUM_2
#endif

static const char *TAG = "LORA";
 
void lora_init(void) {
  #ifdef DEV_TEST_LORA_UART
    gpio_config_t led_conf = {
        .pin_bit_mask = (1ULL << LED_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&led_conf);
    gpio_set_level(LED_PIN, 0);
  #endif
    
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
    uart_write_bytes(LORA_UART, "AT+ADDRESS=2\r\n", 14);
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
    const char *cmd = "AT+SEND=1,1,T\r\n";  // Send to S3 (addr 1)
    uart_write_bytes(LORA_UART, cmd, strlen(cmd));
    ESP_LOGI(TAG, "Trigger sent");
}

void lora_rx_task(void *arg) {
    uint8_t buf[256];
    ESP_LOGI(TAG, "LoRa RX task started, waiting for triggers...");
    
    int poll_count = 0;
    
    while(1) {
        int len = uart_read_bytes(LORA_UART, buf, sizeof(buf)-1, pdMS_TO_TICKS(100));
        
        poll_count++;
        if (poll_count % 100 == 0) {  // Every 10 seconds
            ESP_LOGI(TAG, "RX task alive - %d polls, listening...", poll_count);
        }
        
        if (len > 0) {
            buf[len] = '\0';
            ESP_LOGI(TAG, "RX raw (%d bytes): [%s]", len, buf);
            
            #ifdef DEV_TEST_LORA_UART
            if (strstr((char*)buf, "+RCV=") != NULL) {
                ESP_LOGI(TAG, "GOT +RCV MESSAGE!");
                
                // Check for trigger - data could be "T", "0", or "01"
                if (strstr((char*)buf, ",T,") != NULL || 
                    strstr((char*)buf, ",0,") != NULL || 
                    strstr((char*)buf, ",01,") != NULL) {
                    ESP_LOGI(TAG, "TRIGGER RECEIVED! Blinking LED...");
                    
                    for (int i = 0; i < 3; i++) {
                        gpio_set_level(LED_PIN, 1);
                        vTaskDelay(pdMS_TO_TICKS(200));
                        gpio_set_level(LED_PIN, 0);
                        vTaskDelay(pdMS_TO_TICKS(200));
                    }
                }
            }
            #endif
        }
    }
}
