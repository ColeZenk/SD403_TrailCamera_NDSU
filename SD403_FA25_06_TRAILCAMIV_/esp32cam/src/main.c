#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "ESP32CAM_SLAVE";

void app_main(void)
{
    ESP_LOGI(TAG, "ESP32-CAM Slave Starting");
    
    while(1) {
        ESP_LOGI(TAG, "Camera running...");
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
