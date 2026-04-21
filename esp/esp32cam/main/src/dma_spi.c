/*
 * dma_spi.c
 * ESP32-CAM SPI DMA Master Implementation
 *
 * Sends img data over SPI using DMA for maximum throughput.
 *
 * Uses spi_device_transmit() (synchronous, calls get_trans_result internally)
 * so the ESP-IDF driver never has "unfinished transactions" — required for
 * spi_bus_remove_device / spi_bus_free to succeed during the SD card pin swap.
 *
 * bus_mutex serialises transfers and makes spi_dma_deinit() wait for any
 * in-progress transfer to complete before tearing down the bus.
 */

#include "dma_spi.h"

#include <string.h>

#include "driver/spi_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "img_buf.h"

// Debug flag
#define UNIT_TEST_SPI 0

#if UNIT_TEST_SPI
#define LOG_DEBUG(fmt, ...) ESP_LOGI(TAG, fmt, ##__VA_ARGS__)
#else
#define LOG_DEBUG(fmt, ...)                                                   \
        do {                                                                  \
        } while (0)
#endif

#define SPI_MASTER_HOST HSPI_HOST
#define DMA_CHANNEL     SPI_DMA_CH_AUTO

static const char *TAG = "SPI_TX";

typedef struct {
        spi_device_handle_t spi;
        /* held for full duration of each transfer */
        SemaphoreHandle_t bus_mutex;
        bool initialized;
        uint8_t rx_buf[12];
        uint32_t last_cmd;
        uint32_t epoch;
        bool stop_flag;
} spi_dma_context_t;

static spi_dma_context_t ctx = {.initialized = false};

esp_err_t spi_dma_init(void)
{
        if (ctx.initialized) {
                ESP_LOGW(TAG, "SPI DMA already initialized");
                return ESP_OK;
        }

        ctx.bus_mutex = xSemaphoreCreateMutex();
        if (ctx.bus_mutex == NULL) {
                ESP_LOGE(TAG, "Failed to create bus_mutex");
                return ESP_FAIL;
        }

        spi_bus_config_t bus_cfg = {
            .mosi_io_num = PIN_NUM_MOSI,
            .miso_io_num = PIN_NUM_MISO,
            .sclk_io_num = PIN_NUM_CLK,
            .quadwp_io_num = -1,
            .quadhd_io_num = -1,
            .max_transfer_sz = MAX_TRANSFER_SIZE,
            .flags = SPICOMMON_BUSFLAG_MASTER,
        };

        esp_err_t ret =
            spi_bus_initialize(SPI_MASTER_HOST, &bus_cfg, DMA_CHANNEL);
        if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to initialize SPI bus: %s",
                         esp_err_to_name(ret));
                vSemaphoreDelete(ctx.bus_mutex);
                ctx.bus_mutex = NULL;
                return ret;
        }

        spi_device_interface_config_t dev_cfg = {
            .clock_speed_hz = SPI_CLOCK_SPEED,
            .mode = 0,
            .spics_io_num = PIN_NUM_CS,
            .queue_size = 1,
            .flags = 0,
        };

        ret = spi_bus_add_device(SPI_MASTER_HOST, &dev_cfg, &ctx.spi);
        if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to add SPI device: %s",
                         esp_err_to_name(ret));
                spi_bus_free(SPI_MASTER_HOST);
                vSemaphoreDelete(ctx.bus_mutex);
                ctx.bus_mutex = NULL;
                return ret;
        }

        ctx.initialized = true;
        ESP_LOGI(TAG, "SPI DMA initialized: %d MHz",
                 SPI_CLOCK_SPEED / 1000000);
        return ESP_OK;
}

void spi_dma_deinit(void)
{
        if (!ctx.initialized) return;

        /* Block until any in-progress spi_device_transmit() finishes */
        xSemaphoreTake(ctx.bus_mutex, portMAX_DELAY);

        /* All transactions are complete (spi_device_transmit calls
         * get_trans_result internally), so remove_device and bus_free are safe
         * to call now. */
        spi_bus_remove_device(ctx.spi);
        spi_bus_free(SPI_MASTER_HOST);

        ctx.initialized = false;

        xSemaphoreGive(ctx.bus_mutex);
        vSemaphoreDelete(ctx.bus_mutex);
        ctx.bus_mutex = NULL;

        ESP_LOGI(TAG, "SPI DMA deinitialized");
}

static void check_miso_cmd(const uint8_t *rx)
{
        uint32_t magic;
        memcpy(&magic, rx, 4);
        if (magic != 0xDEADBEEF) return;

        uint32_t cmd;
        memcpy(&cmd, rx + 4, 4);

        if (cmd == CMD_START) memcpy(&ctx.epoch, rx + 8, 4);
        else if (cmd == CMD_STOP) ctx.stop_flag = true;
}

static esp_err_t spi_dma_transmit(const uint8_t *data, size_t length)
{
        if (data == NULL || length == 0 || length > MAX_TRANSFER_SIZE)
                return ESP_ERR_INVALID_ARG;

        spi_transaction_t trans = {
            .length = length * 8,
            .tx_buffer = data,
            .rx_buffer = ctx.rx_buf,
        };

        /* spi_device_transmit = queue_trans + get_trans_result.
         * No transaction remains "pending" after this returns. */
        return spi_device_transmit(ctx.spi, &trans);
}

esp_err_t spi_dma_send_img(const uint8_t *img_data, size_t img_size)
{
        if (!ctx.initialized || !ctx.bus_mutex) return ESP_ERR_INVALID_STATE;

        if (xSemaphoreTake(ctx.bus_mutex, pdMS_TO_TICKS(500)) != pdTRUE) {
                ESP_LOGW(TAG, "Bus mutex timeout — SPI busy");
                return ESP_ERR_TIMEOUT;
        }

        /* Re-check under mutex: deinit may have run between the check above
         * and lock */
        if (!ctx.initialized) {
                xSemaphoreGive(ctx.bus_mutex);
                return ESP_ERR_INVALID_STATE;
        }

        struct {
                uint32_t magic;
                uint32_t size;
                uint32_t checksum;
        } __attribute__((packed)) header;

        header.magic = 0xCAFEBEEF;
        header.size = img_size;

        header.checksum = 0;
        for (size_t i = 0; i < img_size; i++) header.checksum ^= img_data[i];

        esp_err_t ret = spi_dma_transmit((uint8_t *)&header, sizeof(header));
        if (ret != ESP_OK) {
                xSemaphoreGive(ctx.bus_mutex);
                return ret;
        }

        size_t offset = 0;
        while (offset < img_size) {
                size_t chunk = (img_size - offset) > MAX_TRANSFER_SIZE
                                   ? MAX_TRANSFER_SIZE
                                   : (img_size - offset);

                ret = spi_dma_transmit(img_data + offset, chunk);
                if (ret != ESP_OK) {
                        xSemaphoreGive(ctx.bus_mutex);
                        return ret;
                }
                offset += chunk;
        }

        LOG_DEBUG("Sent %d bytes via SPI", img_size);

        xSemaphoreGive(ctx.bus_mutex);
        return ESP_OK;
}

void spi_transmit_task(void *pvParameters)
{
        QueueHandle_t img_queue = (QueueHandle_t)pvParameters;
        img_data_t img_data;

        ESP_LOGI(TAG, "SPI transmit task started");
        vTaskDelay(pdMS_TO_TICKS(1000));

        for (;;) {
                if (xQueueReceive(img_queue, &img_data, portMAX_DELAY) ==
                    pdTRUE) {
                        esp_err_t ret =
                            spi_dma_send_img(img_data.buffer, img_data.size);
                        check_miso_cmd(ctx.rx_buf);

                        if (ret != ESP_OK)
                                ESP_LOGE(TAG, "SPI transmission failed: %s",
                                         esp_err_to_name(ret));
                        else LOG_DEBUG("SPI transmission successful");

                        img_buffer_free(img_data.buffer);

#if UNIT_TEST_SPI
                        int total, in_use, peak;
                        img_buffer_get_stats(&total, &in_use, &peak);
                        ESP_LOGI(TAG, "Buffer pool: %d/%d in use (peak: %d)",
                                 in_use, total, peak);
#endif
                }
        }
}

bool spi_dma_stop_requested(void) { return ctx.stop_flag; }

uint32_t spi_dma_epoch(void) { return ctx.epoch; }
