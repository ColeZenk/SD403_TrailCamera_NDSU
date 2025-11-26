/*
 * DMA_SPI_master.c
 * ESP32-CAM SPI DMA Master Implementation
 * 
 * Sends image data over SPI using DMA for maximum throughput
 */

#include "DMA_SPI_master.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"

#define DMA_BUFFER_SIZE 4092
static DRAM_ATTR uint8_t dma_buffer[DMA_BUFFER_SIZE];  // DMA-capable buffer
#define SPI_MASTER_HOST    HSPI_HOST
#define DMA_CHANNEL        SPI_DMA_CH_AUTO

static const char *TAG = "SPI_TX";

typedef struct {
    spi_device_handle_t spi;
    SemaphoreHandle_t transfer_complete;
} spi_dma_context_t;

static spi_dma_context_t ctx;

// DMA transaction callback - called when transfer completes
static void IRAM_ATTR spi_post_transfer_callback(spi_transaction_t *trans)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(ctx.transfer_complete, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

esp_err_t spi_dma_init(void)
{
    esp_err_t ret;

    // Create semaphore for transfer completion
    ctx.transfer_complete = xSemaphoreCreateBinary();
    if (ctx.transfer_complete == NULL) {
        ESP_LOGE(TAG, "Failed to create semaphore");
        return ESP_FAIL;
    }

    // SPI bus configuration
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = MAX_TRANSFER_SIZE,
        .flags = SPICOMMON_BUSFLAG_MASTER,
    };

    // Initialize SPI bus
    ret = spi_bus_initialize(SPI_MASTER_HOST, &bus_cfg, DMA_CHANNEL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI bus: %s", esp_err_to_name(ret));
        return ret;
    }

    // SPI device configuration
    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = SPI_CLOCK_SPEED,
        .mode = 0,  // SPI mode 0 (CPOL=0, CPHA=0)
        .spics_io_num = PIN_NUM_CS,
        .queue_size = 3,  // Allow queuing multiple transactions
        .post_cb = spi_post_transfer_callback,  // Callback on completion
        .flags = 0,
    };

    ret = spi_bus_add_device(SPI_MASTER_HOST, &dev_cfg, &ctx.spi);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add SPI device: %s", esp_err_to_name(ret));
        spi_bus_free(SPI_MASTER_HOST);
        return ret;
    }

    ESP_LOGI(TAG, "SPI DMA highway initialized successfully");
    ESP_LOGI(TAG, "Clock: %d MHz, Max transfer: %d bytes", 
             SPI_CLOCK_SPEED / 1000000, MAX_TRANSFER_SIZE);
    
    return ESP_OK;
}

esp_err_t spi_dma_transmit(const uint8_t *data, size_t length)
{
    if (data == NULL || length == 0 || length > MAX_TRANSFER_SIZE) {
        ESP_LOGE(TAG, "Invalid parameters: len=%d", length);
        return ESP_ERR_INVALID_ARG;
    }

    spi_transaction_t trans = {
        .length = length * 8,  // Length in bits
        .tx_buffer = data,
        .rx_buffer = NULL,
    };

    esp_err_t ret = spi_device_queue_trans(ctx.spi, &trans, portMAX_DELAY);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to queue transaction: %s", esp_err_to_name(ret));
        return ret;
    }

    // Wait for transfer to complete
    if (xSemaphoreTake(ctx.transfer_complete, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Transfer timeout");
        return ESP_ERR_TIMEOUT;
    }

    return ESP_OK;
}

esp_err_t spi_dma_transmit_large(const uint8_t *data, size_t total_length)
{
    size_t offset = 0;
    uint64_t start_time = esp_timer_get_time();
    
    ESP_LOGI(TAG, "Starting large transfer: %d bytes", total_length);

    while (offset < total_length) {
        size_t chunk_size = (total_length - offset) > MAX_TRANSFER_SIZE 
                            ? MAX_TRANSFER_SIZE 
                            : (total_length - offset);
        
        esp_err_t ret = spi_dma_transmit(data + offset, chunk_size);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Transfer failed at offset %d", offset);
            return ret;
        }
        
        offset += chunk_size;
    }

    uint64_t elapsed = esp_timer_get_time() - start_time;
    float throughput_mbps = (total_length * 8.0f) / elapsed;
    
    ESP_LOGI(TAG, "Transfer complete: %d bytes in %.2f ms (%.2f Mbps)", 
             total_length, elapsed / 1000.0f, throughput_mbps);
    
    return ESP_OK;
}

esp_err_t spi_dma_send_image(const uint8_t *image_data, size_t image_size)
{
    // Create header with magic number and size
    struct {
        uint32_t magic;      // 0xCAFEBEEF
        uint32_t size;       // Image size in bytes
        uint32_t checksum;   // Simple XOR checksum
    } __attribute__((packed)) header;

    header.magic = 0xCAFEBEEF;
    header.size = image_size;
    
    // Calculate simple checksum
    header.checksum = 0;
    for (size_t i = 0; i < image_size; i++) {
        header.checksum ^= image_data[i];
    }

    // Send header
    esp_err_t ret = spi_dma_transmit((uint8_t*)&header, sizeof(header));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send header");
        return ret;
    }

    // Send image data
    ret = spi_dma_transmit_large(image_data, image_size);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send image data");
        return ret;
    }
    
    ESP_LOGI(TAG, "Image sent: %d bytes, checksum: 0x%08" PRIX32, image_size, header.checksum); 
    return ESP_OK;
}

void spi_transmit_task(void *pvParameters)
{
    // Wait for initialization
    vTaskDelay(pdMS_TO_TICKS(1000));

    // Simulate image data (replace with actual camera buffer)
    uint8_t *test_image = malloc(32768);  // 32KB test image
    if (test_image == NULL) {
        ESP_LOGE(TAG, "Failed to allocate test buffer");
        vTaskDelete(NULL);
        return;
    }

    // Fill with test pattern (0xAA = 10101010)
    memset(test_image, 0xAA, 32768);

    while (1) {
        ESP_LOGI(TAG, "Sending test image...");
        
        esp_err_t ret = spi_dma_send_image(test_image, 32768);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Image transmitted successfully");
        } else {
            ESP_LOGE(TAG, "Image transmission failed");
        }

        vTaskDelay(pdMS_TO_TICKS(2000));  // Send every 2 seconds
    }

    free(test_image);
    vTaskDelete(NULL);
}
