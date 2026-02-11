/*
 * image_buffer.c
 * Handles image buffering and processing
 */

#include "Image_buffer.h"
#include "Cam_DMA_interface.h"
#include <inttypes.h>
#include "freertos/queue.h"
#include "esp_log.h"

static const char *TAG = "IMG_BUF";

static QueueHandle_t image_queue = NULL;

// Initialize image buffer module
void image_buffer_init(void)
{
    // Queue is created by Cam_DMA_interface, just get reference if needed
    ESP_LOGI(TAG, "Image buffer initialized");
}

// Get queue handle
QueueHandle_t image_buffer_get_queue(void)
{
    return image_queue;
}

// Image processing task
// Image processing task
void image_process_task(void *pvParameters)
{
    image_data_t img_data;

    ESP_LOGI(TAG, "Image processing task started");

    // Get queue from Cam_DMA_interface
    QueueHandle_t queue = spi_slave_get_queue();

    for (;;) {
        if (xQueueReceive(queue, &img_data, portMAX_DELAY) == pdTRUE) {
            ESP_LOGI(TAG, "Processing image: %zu bytes", img_data.size);

            // Sample checksum for verification
            uint32_t sum = 0;
            for (size_t i = 0; i < img_data.size && i < 1000; i++) {
                sum += img_data.buffer[i];
            }
            ESP_LOGI(TAG, "Image sample checksum: 0x%08" PRIX32, sum);

            // TODO: Forward to FPGA here

            // Free the buffer when done
            free(img_data.buffer);
        }
    }
}
