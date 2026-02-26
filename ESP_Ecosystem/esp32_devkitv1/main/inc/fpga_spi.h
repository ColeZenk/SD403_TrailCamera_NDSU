#ifndef FPGA_SPI_H
#define FPGA_SPI_H

#include "esp_err.h"
#include "config.h"

/*******************************************************************************
 * FPGA SPI Master Interface
 * 
 * Sends image data to Tang Nano 9K FPGA via SPI master DMA
 * Optimized for throughput
 ******************************************************************************/

/**
 * Initialize FPGA SPI master interface
 * 
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t fpga_spi_init(void);

/**
 * Deinitialize FPGA SPI master interface
 */
void fpga_spi_deinit(void);

/**
 * Transmit image data to FPGA
 * Hot function - optimized for throughput
 * 
 * @param data      Pointer to image data
 * @param size      Size of data in bytes
 * @return ESP_OK on success, error code otherwise
 */
HOT_FUNCTION esp_err_t fpga_spi_transmit(const uint8_t *data, size_t size);

#ifdef TEST_MODE_FPGA_PATTERNS
/**
 * Test task for FPGA interface
 * Generates test patterns on button press (like WarblingWire outputTest)
 * 
 * @param pvParameters Task parameters (unused)
 */
void fpga_test_task(void *pvParameters);
#endif

#endif // FPGA_SPI_H
