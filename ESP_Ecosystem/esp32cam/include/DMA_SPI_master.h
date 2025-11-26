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

// SPI Pin Configuration
#define PIN_NUM_MOSI       13
#define PIN_NUM_MISO       12
#define PIN_NUM_CLK        14
#define PIN_NUM_CS         15

// SPI Configuration
#define SPI_CLOCK_SPEED    (10 * 1000 * 1000)  // 20MHz
#define MAX_TRANSFER_SIZE  (4092)             // Maximum DMA transfer size

/**
 * Initialize SPI DMA master interface
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t spi_dma_init(void);

/**
 * Transmit data via SPI DMA (single chunk, blocking)
 * 
 * @param data Pointer to data buffer
 * @param length Length of data in bytes (max MAX_TRANSFER_SIZE)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t spi_dma_transmit(const uint8_t *data, size_t length);

/**
 * Transmit large buffer via SPI DMA (automatic chunking)
 * 
 * @param data Pointer to data buffer
 * @param total_length Total length of data in bytes
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t spi_dma_transmit_large(const uint8_t *data, size_t total_length);

/**
 * Send image data with protocol header
 * Includes magic number, size, and checksum
 * 
 * @param image_data Pointer to image buffer
 * @param image_size Size of image in bytes
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t spi_dma_send_image(const uint8_t *image_data, size_t image_size);

/**
 * SPI transmit task for continuous image sending
 * Can be used as FreeRTOS task function
 * 
 * @param pvParameters Task parameters (unused)
 */
void spi_transmit_task(void *pvParameters);

#endif // DMA_SPI_MASTER_H
