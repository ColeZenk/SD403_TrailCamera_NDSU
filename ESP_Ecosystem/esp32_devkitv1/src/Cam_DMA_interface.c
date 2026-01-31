/*
*  Cam_DMA_interface.c
*  file has two functions:
*    - Initialize slave side DMA interfacing
*    - Receive raw image data from Master(ESP32_CAM)
*/

#include "Cam_DMA_interface.h"
#include "Image_buffer.h"
#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/spi_slave.h"
#include "driver/gpio.h"
#include "esp_log.h"


static const char *TAG = "SPI_RX";

// Static state at file scope
static struct {
    image_header_t header;
    uint8_t *buffer;
    size_t bytes_received;
    bool receiving;
    QueueHandle_t queue;  // <-- add this
} rx_state = {0};

// Forward declarations
static bool process_image_header(const uint8_t *data, image_header_t *header);
static bool verify_image_checksum(const uint8_t *data, size_t size, uint32_t expected_checksum);
static esp_err_t spi_slave_receive_chunk(uint8_t *buffer, size_t *received_length, uint32_t timeout_ms);

// DMA buffers (must be in DMA-capable memory)
WORD_ALIGNED_ATTR uint8_t rx_buffer[BUFFER_SIZE];
WORD_ALIGNED_ATTR uint8_t tx_buffer[BUFFER_SIZE];

// Initialize SPI slave with DMA
esp_err_t spi_slave_dma_init(void)
{
    esp_err_t ret;

    rx_state.queue = xQueueCreate(3, sizeof(image_data_t));
    if (rx_state.queue == NULL) {
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

static void reset_rx_state(void) {
    if (rx_state.buffer) {
        free(rx_state.buffer);  // will fix to static pool later
        rx_state.buffer = NULL;
    }
    rx_state.bytes_received = 0;
    rx_state.receiving = false;
}

static bool handle_header_chunk(const uint8_t *data, size_t size) {
    if (size < sizeof(image_header_t)) return false;

    if (!process_image_header(data, &rx_state.header)) {
        return false;
    }

    rx_state.buffer = malloc(rx_state.header.size);
    if (!rx_state.buffer) {
        ESP_LOGE(TAG, "Failed to allocate %" PRIu32 " bytes", rx_state.header.size);
        return false;
    }

    rx_state.receiving = true;
    rx_state.bytes_received = 0;

    // Copy any data after header in same chunk
    size_t header_size = sizeof(image_header_t);
    if (size > header_size) {
        size_t data_bytes = size - header_size;
        memcpy(rx_state.buffer, data + header_size, data_bytes);
        rx_state.bytes_received += data_bytes;
    }

    return true;
}

static void handle_data_chunk(const uint8_t *data, size_t size) {
    size_t bytes_to_copy = size;
    if (rx_state.bytes_received + bytes_to_copy > rx_state.header.size) {
        bytes_to_copy = rx_state.header.size - rx_state.bytes_received;
    }

    memcpy(rx_state.buffer + rx_state.bytes_received, data, bytes_to_copy);
    rx_state.bytes_received += bytes_to_copy;
}

static void finalize_image(void) {
    ESP_LOGI(TAG, "Image reception complete: %zu bytes", rx_state.bytes_received);

    if (verify_image_checksum(rx_state.buffer, rx_state.header.size, rx_state.header.checksum)) {
        ESP_LOGI(TAG, "Image verified successfully!");

        image_data_t img_data = {
            .buffer = rx_state.buffer,
            .size = rx_state.header.size
        };

        if (xQueueSend(rx_state.queue, &img_data, 0) != pdTRUE) {
            ESP_LOGW(TAG, "Image queue full, dropping image");
            free(rx_state.buffer);
        }
    } else {
        ESP_LOGE(TAG, "Image checksum failed, dropping");
        free(rx_state.buffer);
    }

    rx_state.buffer = NULL;  // don't double-free
    rx_state.receiving = false;
}

void spi_receive_task(void *pvParameters)
{
    ESP_LOGI(TAG, "SPI receive task started");

    while (1) {
        size_t chunk_size = 0;
        esp_err_t ret = spi_slave_receive_chunk(rx_buffer, &chunk_size, 5000);

        if (ret == ESP_ERR_TIMEOUT) {
            if (rx_state.receiving) {
                ESP_LOGW(TAG, "Transfer timeout, resetting");
                reset_rx_state();
            }
            continue;
        }

        if (ret != ESP_OK) continue;

        if (!rx_state.receiving) {
            // Looking for header
            if (handle_header_chunk(rx_buffer, chunk_size)) {
                ESP_LOGI(TAG, "Starting image reception: %" PRIu32 " bytes", rx_state.header.size);
            }
        } else {
            // Collecting data
            handle_data_chunk(rx_buffer, chunk_size);

            if (rx_state.bytes_received >= rx_state.header.size) {
                finalize_image();
            }
        }
    }
}

// Process and validate image header
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

// Verify image checksum
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

// Receive chunk via SPI slave
static esp_err_t spi_slave_receive_chunk(uint8_t *buffer, size_t *received_length, uint32_t timeout_ms)
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

// Get queue handle for image processing
QueueHandle_t spi_slave_get_queue(void)
{
    return rx_state.queue;
}
