/*
 * DMA_SPI_master.h
 * ESP32-CAM SPI DMA Master Interface Header
 * 
 * Provides high-performance SPI DMA transmission for image data
 */

#ifndef DMA_SPI_MASTER_H
#define DMA_SPI_MASTER_H

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "image_data.h"

// SPI Pin Configuration
#define PIN_NUM_MOSI       13
#define PIN_NUM_MISO       12
#define PIN_NUM_CLK        14
#define PIN_NUM_CS         15

// SPI Configuration
#define SPI_CLOCK_SPEED    (10 * 1000 * 1000)  // 10MHz
#define MAX_TRANSFER_SIZE  (4092)

esp_err_t spi_dma_init(void);
esp_err_t spi_dma_send_image(const uint8_t *image_data, size_t image_size);
void spi_transmit_task(void *pvParameters);

#endif // DMA_SPI_MASTER_H
