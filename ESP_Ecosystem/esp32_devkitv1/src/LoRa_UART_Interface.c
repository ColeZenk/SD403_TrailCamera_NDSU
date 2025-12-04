// LoRa_UART_Interface.c

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
#endif /* ifdef DEV_TEST_LORA_UART */

static const char *TAG = "LORA";

void lora_init(void) {
  #ifdef DEV_TEST_LORA_UART
    
  // Configure LED
    gpio_config_t led_conf = {
        .pin_bit_mask = (1ULL << LED_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&led_conf);
    gpio_set_level(LED_PIN, 0);  // Start with LED off

  
  #endif /* ifdef DEV_TEST_LORA_UART */
    
    uart_config_t cfg = {
        .baud_rate = LORA_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    
    uart_driver_install(LORA_UART, 2048, 2048, 0, NULL, 0);
    uart_param_config(LORA_UART, &cfg);
    uart_set_pin(LORA_UART, LORA_TX_PIN, LORA_RX_PIN, -1, -1);
    
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Test connection
    uart_write_bytes(LORA_UART, "AT\r\n", 4);
    
    uint8_t buf[64];
    int len = uart_read_bytes(LORA_UART, buf, sizeof(buf), pdMS_TO_TICKS(500));
    if (len > 0) {
        buf[len] = '\0';
        ESP_LOGI(TAG, "LoRa ready: %s", buf);
    } else {
        ESP_LOGW(TAG, "No LoRa response - check wiring");
    }
}

// Send a single trigger byte
void lora_send_trigger(void) {
    const char *cmd = "AT+SEND=0,1,01\r\n";  // Send byte 0x01 to address 0
    uart_write_bytes(LORA_UART, cmd, strlen(cmd));
    ESP_LOGI(TAG, "Trigger sent");
}

// Receive task - waits for incoming data
void lora_rx_task(void *arg) {
    uint8_t buf[256];
    ESP_LOGI(TAG, "LoRa RX task started, waiting for triggers...");

    while(1) {
        int len = uart_read_bytes(LORA_UART, buf, sizeof(buf)-1, pdMS_TO_TICKS(100));
        if (len > 0) {
            buf[len] = '\0';
            ESP_LOGI(TAG, "RX: %s", buf);
            #ifndef DEV_TEST_LORA_UART
            // Check if it's a received message
            if (strstr((char*)buf, "+RCV=") != NULL) {
                // Parse: "+RCV=0,1,01,-50,40" (addr, len, data, rssi, snr)
                if (strstr((char*)buf, ",01,") != NULL) {  // Look for our trigger byte
                    ESP_LOGI(TAG, "TRIGGER RECEIVED!");
                    trigger_camera_capture();  // Your function
                }
            }
            #else
            // Check if it's a received message with our trigger byte
            if (strstr((char*)buf, "+RCV=") != NULL && strstr((char*)buf, ",01,") != NULL) {
                ESP_LOGI(TAG, "TRIGGER RECEIVED! Blinking LED...");
                
                // Blink LED 3 times
                for (int i = 0; i < 3; i++) {
                    gpio_set_level(LED_PIN, 1);
                    vTaskDelay(pdMS_TO_TICKS(200));
                    gpio_set_level(LED_PIN, 0);
                    vTaskDelay(pdMS_TO_TICKS(200));
                }
            }
            #endif /* ifndef DEV_TEST_LORA_UART */
        }
    }
}
