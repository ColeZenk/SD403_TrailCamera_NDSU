/**
 * image_processor.c — Camera → FPGA pipeline
 *
 * Dequeues images from cam_spi, forwards to fpga_spi.
 * Raw passthrough for now — compression hooks go here later.
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "image_processor.h"
#include "cam_spi.h"
#include "fpga_spi.h"
#include "utils.h"

static const char *TAG = "IMG_PROC";

esp_err_t image_processor_init(void)
{
    ESP_LOGI(TAG, "initialized");
    return ESP_OK;
}

void image_processor_task(void *pvParameters)
{
    QueueHandle_t queue = cam_spi_get_queue();
    image_data_t img;

    ESP_LOGI(TAG, "task started");

    for (;;) {
        if (xQueueReceive(queue, &img, portMAX_DELAY) == pdTRUE) {
            ESP_LOGI(TAG, "%zu bytes → FPGA", img.size);

            esp_err_t ret = fpga_spi_transmit(img.buffer, img.size);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "FPGA tx failed: %s", esp_err_to_name(ret));
            }

            safe_free((void **)&img.buffer);
        }
    }
}
