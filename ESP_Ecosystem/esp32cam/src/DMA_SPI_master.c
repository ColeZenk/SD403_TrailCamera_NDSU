/*
 * DMA_SPI_master.c
 * ESP32-CAM SPI DMA Master Implementation
 * 
 * Sends image data over SPI using DMA for maximum throughput
 */
// Debug flag - set to 1 for simulation mode with test data
#define UNIT_TEST_SPI 1

#include "DMA_SPI_master.h"
#if !UNIT_TEST_SPI
#include "image_buffer_pool.h"
#endif
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"

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
  
  // Allocate buffer for header + data
  size_t total_size = sizeof(header) + image_size;
  uint8_t *tx_buffer = heap_caps_malloc(total_size, MALLOC_CAP_DMA);
  if (!tx_buffer) {
    ESP_LOGE(TAG, "Failed to allocate DMA buffer");
    return ESP_ERR_NO_MEM;
  }
  
  // Build header
  header.magic = 0xCAFEBEEF;
  header.size = image_size;
  
  // Calculate checksum
  header.checksum = 0;
  for (size_t i = 0; i < image_size; i++) {
    header.checksum ^= image_data[i];
  }
  
  // Copy header to buffer
  memcpy(tx_buffer, &header, sizeof(header));
  
  // Copy image data after header
  memcpy(tx_buffer + sizeof(header), image_data, image_size);
  
  ESP_LOGI(TAG, "Sending header+data in single transaction: %zu bytes", total_size);
  
  // Send everything in ONE transaction
  esp_err_t ret = spi_dma_transmit(tx_buffer, total_size);
  
  free(tx_buffer);
  
  if (ret == ESP_OK) {
    LOG_DEBUG("✓ Sent %zu bytes via SPI", total_size);
  }
  
  return ret;
}

void spi_transmit_task(void *pvParameters)
{
#if UNIT_TEST_SPI
  // ============================================================================
  // SIMULATION MODE - Use predictable test data for debugging
  // ============================================================================
  (void)pvParameters;  // Unused in test mode
  
  ESP_LOGI(TAG, "SPI transmit task started (SIMULATION MODE)");
  vTaskDelay(pdMS_TO_TICKS(1000));

  // Allocate test buffer with known pattern
  static uint8_t test_image[1024];
  
  // Initialize with incrementing pattern (0x00, 0x01, 0x02, ... 0xFF, 0x00, ...)
  for (int i = 0; i < sizeof(test_image); i++) {
    test_image[i] = i & 0xFF;
  }

  ESP_LOGI(TAG, "Test pattern initialized: %d bytes", sizeof(test_image));
  ESP_LOGI(TAG, "First 32 bytes of test pattern:");
  for (int i = 0; i < 32; i++) {
    printf("%02X ", test_image[i]);
    if ((i + 1) % 16 == 0) printf("\n");
  }
  printf("\n");
  
  uint32_t expected_checksum = 0;
  for (int i = 0; i < sizeof(test_image); i++) {
    expected_checksum ^= test_image[i];
  }
  
  ESP_LOGI(TAG, "Expected checksum: 0x%08" PRIX32, expected_checksum);
  
  while (1) {
    LOG_DEBUG("Sending test image via SPI: %d bytes", sizeof(test_image));
    
    esp_err_t ret = spi_dma_send_image(test_image, sizeof(test_image));
    
    if (ret == ESP_OK) {
      LOG_DEBUG("✓ SPI transmission successful");
    } else {
      ESP_LOGE(TAG, "✗ SPI transmission failed: %s", esp_err_to_name(ret));
    }
    
    // Send test image every second
    vTaskDelay(pdMS_TO_TICKS(1000));
  }

#else
  // ============================================================================
  // PRODUCTION MODE - Use real image buffer from camera
  // ============================================================================
  QueueHandle_t image_queue = (QueueHandle_t)pvParameters;
  image_data_t img_data;

  ESP_LOGI(TAG, "SPI transmit task started (PRODUCTION MODE)");
  vTaskDelay(pdMS_TO_TICKS(1000));

  while (1) {
    if (xQueueReceive(image_queue, &img_data, portMAX_DELAY) == pdTRUE) {
      LOG_DEBUG("Sending image via SPI: %d bytes", img_data.size);

      esp_err_t ret = spi_dma_send_image(img_data.buffer, img_data.size);

      if (ret == ESP_OK) {
        LOG_DEBUG("✓ SPI transmission successful");
      } else {
        ESP_LOGE(TAG, "✗ SPI transmission failed: %s", esp_err_to_name(ret));
      }

      // Return buffer to PSRAM pool
      image_buffer_free(img_data.buffer);

      // Log buffer pool statistics
      int total, in_use, peak;
      image_buffer_get_stats(&total, &in_use, &peak);
      ESP_LOGI(TAG, "Buffer pool: %d/%d in use (peak: %d)", in_use, total, peak);
    }
  }
#endif
}
