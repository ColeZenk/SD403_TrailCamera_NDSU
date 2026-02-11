/*
 * DMA_SPI_master.c
 * ESP32-CAM SPI DMA Master Implementation
 *
 * Sends image data over SPI using DMA for maximum throughput
 */

#include "DMA_SPI_master.h"
#include "image_buffer_pool.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"

// Debug flag
#define UNIT_TEST_SPI 0

#if UNIT_TEST_SPI
#define LOG_DEBUG(fmt, ...) ESP_LOGI(TAG, fmt, ##__VA_ARGS__)
#else
#define LOG_DEBUG(fmt, ...) do {} while(0)
#endif

#define SPI_MASTER_HOST    HSPI_HOST
#define DMA_CHANNEL        SPI_DMA_CH_AUTO

static const char *TAG = "SPI_TX";

typedef struct {
  spi_device_handle_t spi;
  SemaphoreHandle_t transfer_complete;
} spi_dma_context_t;

static spi_dma_context_t ctx;

// DMA transaction callback
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

  ctx.transfer_complete = xSemaphoreCreateBinary();
  if (ctx.transfer_complete == NULL) {
    ESP_LOGE(TAG, "Failed to create semaphore");
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

  ret = spi_bus_initialize(SPI_MASTER_HOST, &bus_cfg, DMA_CHANNEL);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize SPI bus: %s", esp_err_to_name(ret));
    return ret;
  }

  spi_device_interface_config_t dev_cfg = {
    .clock_speed_hz = SPI_CLOCK_SPEED,
    .mode = 0,
    .spics_io_num = PIN_NUM_CS,
    .queue_size = 3,
    .post_cb = spi_post_transfer_callback,
    .flags = 0,
  };

  ret = spi_bus_add_device(SPI_MASTER_HOST, &dev_cfg, &ctx.spi);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to add SPI device: %s", esp_err_to_name(ret));
    spi_bus_free(SPI_MASTER_HOST);
    return ret;
  }

  ESP_LOGI(TAG, "SPI DMA initialized: %d MHz", SPI_CLOCK_SPEED / 1000000);

  return ESP_OK;
}

static esp_err_t spi_dma_transmit(const uint8_t *data, size_t length)
{
  if (data == NULL || length == 0 || length > MAX_TRANSFER_SIZE) {
    return ESP_ERR_INVALID_ARG;
  }

  spi_transaction_t trans = {
    .length = length * 8,
    .tx_buffer = data,
    .rx_buffer = NULL,
  };

  esp_err_t ret = spi_device_queue_trans(ctx.spi, &trans, portMAX_DELAY);
  if (ret != ESP_OK) {
    return ret;
  }

  if (xSemaphoreTake(ctx.transfer_complete, pdMS_TO_TICKS(1000)) != pdTRUE) {
    ESP_LOGE(TAG, "Transfer timeout");
    return ESP_ERR_TIMEOUT;
  }

  return ESP_OK;
}

esp_err_t spi_dma_send_image(const uint8_t *image_data, size_t image_size)
{
  struct {
    uint32_t magic;
    uint32_t size;
    uint32_t checksum;
  } __attribute__((packed)) header;

  header.magic = 0xCAFEBEEF;
  header.size = image_size;

  header.checksum = 0;
  for (size_t i = 0; i < image_size; i++) {
    header.checksum ^= image_data[i];
  }

  esp_err_t ret = spi_dma_transmit((uint8_t*)&header, sizeof(header));
  if (ret != ESP_OK) {
    return ret;
  }

  size_t offset = 0;
  while (offset < image_size) {
    size_t chunk_size = (image_size - offset) > MAX_TRANSFER_SIZE
                            ? MAX_TRANSFER_SIZE
                            : (image_size - offset);

    ret = spi_dma_transmit(image_data + offset, chunk_size);
    if (ret != ESP_OK) {
      return ret;
    }

    offset += chunk_size;
  }

  LOG_DEBUG("Sent %d bytes via SPI", image_size);

  return ESP_OK;
}

void spi_transmit_task(void *pvParameters)
{
  QueueHandle_t image_queue = (QueueHandle_t)pvParameters;
  image_data_t img_data;

  ESP_LOGI(TAG, "SPI transmit task started");
  vTaskDelay(pdMS_TO_TICKS(1000));

  for (;;) {
    if (xQueueReceive(image_queue, &img_data, portMAX_DELAY) == pdTRUE) {
      LOG_DEBUG("Sending image via SPI: %d bytes", img_data.size);

      esp_err_t ret = spi_dma_send_image(img_data.buffer, img_data.size);

      if (ret == ESP_OK) {
        LOG_DEBUG("✓ SPI transmission successful");
      } else {
        ESP_LOGE(TAG, "✗ SPI transmission failed");
      }

      // Return buffer to PSRAM pool
      image_buffer_free(img_data.buffer);

#if UNIT_TEST_SPI
      int total, in_use, peak;
      image_buffer_get_stats(&total, &in_use, &peak);
      ESP_LOGI(TAG, "Buffer pool: %d/%d in use (peak: %d)", in_use, total, peak);
#endif
    }
  }
}
