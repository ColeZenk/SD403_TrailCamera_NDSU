/*
 * dma_spi.h
 * ESP32-CAM SPI DMA Master Interface Header
 *
 * Provides high-performance SPI DMA transmission for img data
 */

#ifndef DMA_SPI_MASTER_H
#define DMA_SPI_MASTER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "img_data.h"

// SPI Pin Configuration
#define PIN_NUM_MOSI 13
#define PIN_NUM_MISO 12
#define PIN_NUM_CLK 14
#define PIN_NUM_CS 15

// Start and stop commands
#define CMD_START     1
#define CMD_STOP      2

// SPI Configuration
#define SPI_CLOCK_SPEED (10 * 1000 * 1000) // 10MHz
#define MAX_TRANSFER_SIZE (4092)

esp_err_t spi_dma_init(void);
void spi_dma_deinit(void);
esp_err_t spi_dma_send_img(const uint8_t *img_data, size_t img_size);
void spi_transmit_task(void *pvParameters);
bool spi_dma_stop_requested(void);
uint32_t spi_dma_epoch(void);

#endif // DMA_SPI_MASTER_H
