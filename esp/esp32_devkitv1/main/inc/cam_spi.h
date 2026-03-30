#ifndef CAM_SPI_H
#define CAM_SPI_H

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "config.h"

/*******************************************************************************
 * Camera SPI Slave Interface
 * 
 * Receives image data from ESP32-CAM via SPI slave DMA
 * Hot path optimized for minimal latency
 ******************************************************************************/

/**
 * Initialize camera SPI slave interface
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t cam_spi_init(void);

/**
 * Deinitialize camera SPI slave interface
 */
void cam_spi_deinit(void);

/**
 * Get queue handle for received images
 * 
 * @return Queue handle or NULL if not initialized
 */
QueueHandle_t cam_spi_get_queue(void);

/**
 * SPI receive task - should be run as FreeRTOS task
 * Marked as hot function for optimization
 * 
 * @param pvParameters Task parameters (unused)
 */
HOT_FUNCTION void cam_spi_receive_task(void *pvParameters);

#endif // CAM_SPI_H
