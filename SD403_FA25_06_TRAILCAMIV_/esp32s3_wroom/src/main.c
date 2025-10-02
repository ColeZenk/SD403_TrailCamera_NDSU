#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "TRAILCAM_MASTER";

void app_main(void)
{
    ESP_LOGI(TAG, "Trail Camera IV - ESP32-S3 Master");
    ESP_LOGI(TAG, "Initialization complete");
    
    while(1) {
        ESP_LOGI(TAG, "Master running...");
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
