#ifndef CAM_DMA_INTERFACE_H
#define CAM_DMA_INTERFACE_H

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// ESP32-S3 SPI2 slave pins (hardware-fixed for DMA)
#define PIN_NUM_MOSI       23
#define PIN_NUM_MISO       19
#define PIN_NUM_CLK        18
#define PIN_NUM_CS         5

#define DMA_CHANNEL        SPI_DMA_CH_AUTO
#define BUFFER_SIZE        4096

// Image header structure
typedef struct {
    uint32_t magic;
    uint32_t size;
    uint32_t checksum;
} __attribute__((packed)) image_header_t;

// Image data structure
typedef struct {
    uint8_t *buffer;
    size_t size;
} image_data_t;

// Public functions
esp_err_t spi_slave_dma_init(void);
void spi_receive_task(void *pvParameters);
QueueHandle_t spi_slave_get_queue(void);

#endif // CAM_DMA_INTERFACE_H
