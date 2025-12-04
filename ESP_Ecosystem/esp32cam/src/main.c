/*
 * ESP32-CAM Main Application
 * Camera + SPI DMA Transmitter
 */

#include "camera_control.h"
#include "DMA_SPI_master.h"
#include "image_buffer_pool.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_log.h"

static QueueHandle_t image_queue = NULL;

void app_main(void)
{
  ESP_LOGI("MAIN", "Trail Camera System Starting...");

  // Step 1: Initialize buffer pool in PSRAM
  if (image_buffer_pool_init() != ESP_OK) {
    ESP_LOGE("MAIN", "Failed to initialize buffer pool!");
    return;
  }

  // Step 2: Create image queue
  image_queue = xQueueCreate(3, sizeof(image_data_t));
  if (image_queue == NULL) {
    ESP_LOGE("MAIN", "Failed to create image queue");
    return;
  }

  // Step 3: Initialize SPI DMA
  if (spi_dma_init() != ESP_OK) {
    ESP_LOGE("MAIN", "Failed to initialize SPI DMA");
    return;
  }

  // Step 4: Start camera system
  cam_log_unit_test(image_queue);

  // Step 5: Start SPI transmit task
  xTaskCreate(spi_transmit_task, "spi_tx", 4096, (void*)image_queue, 5, NULL);

  ESP_LOGI("MAIN", "System initialized successfully!");
}
