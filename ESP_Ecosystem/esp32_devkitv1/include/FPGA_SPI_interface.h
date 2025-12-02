/*
 * FPGA_SPI_interface.h
 * SPI DMA Master interface to Tang Nano 9K FPGA
 */

#ifndef FPGA_SPI_INTERFACE_H
#define FPGA_SPI_INTERFACE_H

#include "esp_err.h"
#include <stdint.h>
#include <stddef.h>

// SPI Configuration
#define FPGA_MOSI_PIN  32
#define FPGA_MISO_PIN  33
#define FPGA_SCLK_PIN  25
#define FPGA_CS_PIN    26
#define SPI_CLOCK_MHZ  9

/**
 * Initialize SPI master interface to FPGA
 * Uses GPIO 32/33/25/26, 9 MHz clock, DMA enabled
 */
esp_err_t fpga_spi_init(void);

/**
 * Transmit image data to FPGA via SPI DMA
 * Automatically chunks data into DMA-sized transfers
 * 
 * @param data Image buffer (must be DMA-capable memory)
 * @param size Size in bytes
 */
esp_err_t fpga_spi_transmit_image(const uint8_t *data, size_t size);

/**
 * Test task - button press sends test pattern to FPGA
 * Press BOOT button (GPIO 0) to trigger transmission
 */
void fpga_test_task(void *pvParameters);

#endif // FPGA_SPI_INTERFACE_H
