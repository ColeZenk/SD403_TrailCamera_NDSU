#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "ESP32CAM_SLAVE";

#define LED_GPIO GPIO_NUM_33  // Flash LED on ESP32-CAM

void app_main(void)
{
    ESP_LOGI(TAG, "ESP32-CAM LED Blink Test");
    
    // Configure LED GPIO
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LED_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    
    ESP_LOGI(TAG, "LED configured on GPIO %d", LED_GPIO);
    ESP_LOGI(TAG, "Starting blink sequence...");
    
    bool led_state = false;
    while (1) {
        led_state = !led_state;
        gpio_set_level(LED_GPIO, led_state);
        ESP_LOGI(TAG, "LED %s", led_state ? "ON" : "OFF");
        vTaskDelay(500 / portTICK_PERIOD_MS); // Blink every 500ms}
    }
}

