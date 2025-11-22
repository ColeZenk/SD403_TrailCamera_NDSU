/*
 * ESP32-S3 WROOM SPI DMA Receiver
 * Receives image data over SPI using DMA for maximum throughput
 */

#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/spi_slave.h"
#include "driver/gpio.h"
#include "esp_log.h"

// ESP32-S3 SPI2 slave pins (hardware-fixed for DMA)
#define PIN_NUM_MOSI       11
#define PIN_NUM_MISO       13
#define PIN_NUM_CLK        12
#define PIN_NUM_CS         10

#define DMA_CHANNEL        SPI_DMA_CH_AUTO
#define BUFFER_SIZE        4096

static const char *TAG = "SPI_RX";

typedef struct {
    uint32_t magic;
    uint32_t size;
    uint32_t checksum;
} __attribute__((packed)) image_header_t;

typedef struct {
    uint8_t *buffer;
    size_t size;
} image_data_t;

static QueueHandle_t image_queue;

// DMA buffers (must be in DMA-capable memory)
WORD_ALIGNED_ATTR uint8_t rx_buffer[BUFFER_SIZE];
WORD_ALIGNED_ATTR uint8_t tx_buffer[BUFFER_SIZE];

// Initialize SPI slave with DMA
esp_err_t spi_slave_dma_init(void)
{
    esp_err_t ret;

    image_queue = xQueueCreate(3, sizeof(image_data_t));
    if (image_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create image queue");
        return ESP_FAIL;
    }

    spi_bus_config_t bus_cfg = {
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = BUFFER_SIZE,
    };

    spi_slave_interface_config_t slave_cfg = {
        .mode = 0,
        .spics_io_num = PIN_NUM_CS,
        .queue_size = 3,
        .flags = 0,
    };

    ret = spi_slave_initialize(SPI2_HOST, &bus_cfg, &slave_cfg, DMA_CHANNEL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI slave: %s", esp_err_to_name(ret));
        return ret;
    }

    gpio_set_pull_mode(PIN_NUM_CS, GPIO_PULLUP_ONLY);

    ESP_LOGI(TAG, "SPI DMA slave initialized successfully");
    ESP_LOGI(TAG, "Buffer size: %d bytes", BUFFER_SIZE);
    
    return ESP_OK;
}

esp_err_t spi_slave_receive_chunk(uint8_t *buffer, size_t *received_length, uint32_t timeout_ms)
{
    spi_slave_transaction_t trans = {
        .length = BUFFER_SIZE * 8,
        .rx_buffer = buffer,
        .tx_buffer = tx_buffer,
    };

    esp_err_t ret = spi_slave_transmit(SPI2_HOST, &trans, pdMS_TO_TICKS(timeout_ms));
    if (ret != ESP_OK) {
        if (ret != ESP_ERR_TIMEOUT) {
            ESP_LOGE(TAG, "SPI receive error: %s", esp_err_to_name(ret));
        }
        return ret;
    }

    *received_length = trans.trans_len / 8;
    return ESP_OK;
}

static bool process_image_header(const uint8_t *data, image_header_t *header)
{
    memcpy(header, data, sizeof(image_header_t));
    
    if (header->magic != 0xCAFEBEEF) {
        ESP_LOGW(TAG, "Invalid magic: 0x%08" PRIX32, header->magic);
        return false;
    }
    
    if (header->size == 0 || header->size > (1024 * 1024)) {
        ESP_LOGW(TAG, "Invalid size: %" PRIu32, header->size);
        return false;
    }
    
    ESP_LOGI(TAG, "Valid header - Size: %" PRIu32 " bytes, Checksum: 0x%08" PRIX32, 
             header->size, header->checksum);
    return true;
}

static bool verify_image_checksum(const uint8_t *data, size_t size, uint32_t expected_checksum)
{
    uint32_t calculated = 0;
    for (size_t i = 0; i < size; i++) {
        calculated ^= data[i];
    }
    
    bool valid = (calculated == expected_checksum);
    if (!valid) {
        ESP_LOGE(TAG, "Checksum mismatch! Expected: 0x%08" PRIX32 ", Got: 0x%08" PRIX32,
                 expected_checksum, calculated);
    }
    
    return valid;
}

void spi_receive_task(void *pvParameters)
{
    ESP_LOGI(TAG, "SPI receive task started");
    
    image_header_t current_header = {0};
    uint8_t *image_buffer = NULL;
    size_t bytes_received = 0;
    bool receiving_image = false;

    while (1) {
        size_t chunk_size = 0;
        esp_err_t ret = spi_slave_receive_chunk(rx_buffer, &chunk_size, 5000);
        
        if (ret == ESP_ERR_TIMEOUT) {
            if (receiving_image) {
                ESP_LOGW(TAG, "Transfer timeout, resetting");
                free(image_buffer);
                image_buffer = NULL;
                receiving_image = false;
                bytes_received = 0;
            }
            continue;
        }
        
        if (ret != ESP_OK) {
            continue;
        }

        ESP_LOGD(TAG, "Received chunk: %zu bytes", chunk_size);

        if (!receiving_image && chunk_size >= sizeof(image_header_t)) {
            if (process_image_header(rx_buffer, &current_header)) {
                image_buffer = malloc(current_header.size);
                if (image_buffer == NULL) {
                    ESP_LOGE(TAG, "Failed to allocate %" PRIu32 " bytes", current_header.size);
                    continue;
                }
                
                receiving_image = true;
                bytes_received = 0;
                ESP_LOGI(TAG, "Starting image reception: %" PRIu32 " bytes", current_header.size);
                
                size_t header_size = sizeof(image_header_t);
                if (chunk_size > header_size) {
                    size_t data_in_chunk = chunk_size - header_size;
                    memcpy(image_buffer, rx_buffer + header_size, data_in_chunk);
                    bytes_received += data_in_chunk;
                    ESP_LOGD(TAG, "Header chunk contained %zu data bytes", data_in_chunk);
                }
            }
        }
        else if (receiving_image && image_buffer != NULL) {
            size_t bytes_to_copy = chunk_size;
            if (bytes_received + bytes_to_copy > current_header.size) {
                bytes_to_copy = current_header.size - bytes_received;
            }
            
            memcpy(image_buffer + bytes_received, rx_buffer, bytes_to_copy);
            bytes_received += bytes_to_copy;
            
            ESP_LOGD(TAG, "Image progress: %zu / %" PRIu32 " bytes", bytes_received, current_header.size);
            
            if (bytes_received >= current_header.size) {
                ESP_LOGI(TAG, "Image reception complete: %zu bytes", bytes_received);
                
                if (verify_image_checksum(image_buffer, current_header.size, current_header.checksum)) {
                    ESP_LOGI(TAG, "Image verified successfully!");
                    
                    image_data_t img_data = {
                        .buffer = image_buffer,
                        .size = current_header.size
                    };
                    
                    if (xQueueSend(image_queue, &img_data, 0) != pdTRUE) {
                        ESP_LOGW(TAG, "Image queue full, dropping image");
                        free(image_buffer);
                    }
                } else {
                    ESP_LOGE(TAG, "Image checksum failed, dropping");
                    free(image_buffer);
                }
                
                image_buffer = NULL;
                receiving_image = false;
                bytes_received = 0;
            }
        }
    }
}

void image_process_task(void *pvParameters)
{
    image_data_t img_data;
    
    ESP_LOGI(TAG, "Image processing task started");
    
    while (1) {
        if (xQueueReceive(image_queue, &img_data, portMAX_DELAY) == pdTRUE) {
            ESP_LOGI(TAG, "Processing image: %zu bytes", img_data.size);
            
            uint32_t sum = 0;
            for (size_t i = 0; i < img_data.size && i < 1000; i++) {
                sum += img_data.buffer[i];
            }
            ESP_LOGI(TAG, "Image sample checksum: 0x%08" PRIX32, sum);
            
            free(img_data.buffer);
        }
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "ESP32-WROOM SPI DMA Receiver");
    
    if (spi_slave_dma_init() != ESP_OK) {
        ESP_LOGE(TAG, "SPI initialization failed!");
        return;
    }

    xTaskCreate(spi_receive_task, "spi_rx_task", 4096, NULL, 5, NULL);
    xTaskCreate(image_process_task, "img_proc_task", 8192, NULL, 4, NULL);
    
    ESP_LOGI(TAG, "SPI DMA highway ready!");
}
