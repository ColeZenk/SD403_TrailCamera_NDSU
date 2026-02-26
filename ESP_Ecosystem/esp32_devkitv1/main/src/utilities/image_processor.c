/**
 * @file image_processor.c
 * @brief Image Processor - Camera SPI to FPGA Pipeline
 * @author Cole Zenk
 *
 * Dequeues images from the camera SPI receive queue and forwards
 * them to the FPGA over SPI. No compression at this stage —
 * raw pipeline for initial hardware validation.
 *
 * @scope
 * Sits between cam_spi and fpga_spi. That's it.
 */

/*************************************************************************
 INCLUDES
 *************************************************************************/
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "image_processor.h"
#include "cam_spi.h"
#include "fpga_spi.h"
#include "utils.h"

static const char *TAG = "IMG_PROC";

/*************************************************************************
 PUBLIC INTERFACE
 *************************************************************************/

esp_err_t image_processor_init(void)
{
    ESP_LOGI(TAG, "Image processor initialized");
    return ESP_OK;
}

void image_processor_task(void *pvParameters)
{
    QueueHandle_t queue = cam_spi_get_queue();
    image_data_t img_data;

    ESP_LOGI(TAG, "Image processor task started");

    for (;;) {
        if (xQueueReceive(queue, &img_data, portMAX_DELAY) == pdTRUE) {
            ESP_LOGI(TAG, "Forwarding image: %zu bytes → FPGA", img_data.size);

            esp_err_t ret = fpga_spi_transmit(img_data.buffer, img_data.size);

            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "FPGA transmit failed: %s", esp_err_to_name(ret));
            }

            safe_free((void**)&img_data.buffer);
        }
    }
}
