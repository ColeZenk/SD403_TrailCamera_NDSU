/*
 * ESP32-CAM Main Application
 * Camera + SPI DMA Transmitter
 */

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_log.h"

#include "camera_control.h"
#include "sd_storage.h"
#include "image_buffer_pool.h"

static QueueHandle_t image_queue = NULL;

void app_main(void)
{
    ESP_LOGI("MAIN", "Remote Inspection Unit Starting...");

    /* Initialize SD card first */
    if (sd_card_init() != ESP_OK)
    {
        ESP_LOGE("MAIN", "SD card init failed - halting");
        return;
    }

    /* Initialize camera */
    if (camera_init() != ESP_OK)
    {
        ESP_LOGE("MAIN", "Camera init failed - halting");
        return;
    }

#ifdef ISOLATED_CAM
    /* Start timer-triggered capture */
    if (camera_timer_init(CAPTURE_RATE) != ESP_OK) {
        ESP_LOGE("MAIN", "Timer init failed - halting");
        return;
    }

    /* Create capture task (waits for timer) */
    xTaskCreate(isolated_capture_task, "capture", 4096, NULL, 5, NULL);
#endif

    ESP_LOGI("MAIN", "System initialized successfully!");
}
