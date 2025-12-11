/*
 * ESP32-CAM Main Application
 * Camera + SPI DMA Transmitter
 */

// #include "camera_control.h"
#include "DMA_SPI_master.h"
// #include "image_buffer_pool.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_log.h"

static QueueHandle_t image_queue = NULL;

void app_main(void)
{
 
  // Step 1: Initialize SPI DMA
  if (spi_dma_init() != ESP_OK) {
    ESP_LOGE("MAIN", "Failed to initialize SPI DMA");
    return;
  }

  // Step 2: Start SPI transmit task
  xTaskCreate(spi_transmit_task, "spi_tx", 4096, (void*)image_queue, 5, NULL);

  ESP_LOGI("MAIN", "System initialized successfully!");
}
