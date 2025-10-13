#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "ESP32CAM_SLAVE";

void app_main(void)
{
    ESP_LOGI(TAG, "Hello, ESP32-CAM!");
    while (1) {
        vTaskDelay(1000 / portTICK_PERIOD_MS); // Delay for a second
        ESP_LOGI(TAG, "Running...");
    }
}

