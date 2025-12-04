/*
 * ESP32-CAM camera_control
 * camera init, testing, and log to SD
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_camera.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdmmc_host.h"

#include "image_buffer_pool.h"
#include "DMA_SPI_master.h"
#include "camera_control.h"

static sdmmc_card_t *card = NULL;
static int image_counter = 0;
static QueueHandle_t tx_queue = NULL;

static const char *TAG = "CAM_CTRL";

// Debug flag - set to 1 to enable verbose logging
#define UNIT_TEST_CAMERA 0

#if UNIT_TEST_CAMERA
#define LOG_DEBUG(fmt, ...) ESP_LOGI(TAG, fmt, ##__VA_ARGS__)
#else
#define LOG_DEBUG(fmt, ...) do {} while(0)
#endif

// AI-Thinker ESP32-CAM Camera Configuration
static camera_config_t camera_config = {
  .pin_pwdn = 32,
  .pin_reset = -1,
  .pin_xclk = 0,
  .pin_sccb_sda = 26,
  .pin_sccb_scl = 27,

  .pin_d7 = 35,
  .pin_d6 = 34,
  .pin_d5 = 39,
  .pin_d4 = 36,
  .pin_d3 = 21,
  .pin_d2 = 19,
  .pin_d1 = 18,
  .pin_d0 = 5,
  .pin_vsync = 25,
  .pin_href = 23,
  .pin_pclk = 22,

  .xclk_freq_hz = 20000000,
  .ledc_timer = LEDC_TIMER_0,
  .ledc_channel = LEDC_CHANNEL_0,

  .pixel_format = PIXFORMAT_GRAYSCALE,
  .frame_size = FRAMESIZE_QVGA,  // 320x240
  .jpeg_quality = 12,
  .fb_count = 1,
  .fb_location = CAMERA_FB_IN_PSRAM,
  .grab_mode = CAMERA_GRAB_WHEN_EMPTY
};

esp_err_t sd_card_init(void)
{
  LOG_DEBUG("Initializing SD card...");

  esp_vfs_fat_sdmmc_mount_config_t mount_config = {
    .format_if_mount_failed = false,
    .max_files = 5,
    .allocation_unit_size = 16 * 1024
  };

  sdmmc_host_t host = SDMMC_HOST_DEFAULT();
  host.max_freq_khz = SDMMC_FREQ_HIGHSPEED;  // 40 MHz

  sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
  slot_config.width = 4;  // 4-bit mode

  esp_err_t ret = esp_vfs_fat_sdmmc_mount("/sdcard", &host, &slot_config, &mount_config, &card);

  if (ret != ESP_OK) {
    if (ret == ESP_FAIL) {
      ESP_LOGE(TAG, "Failed to mount filesystem. Insert SD card and reset.");
    } else {
      ESP_LOGE(TAG, "Failed to initialize SD card: %s", esp_err_to_name(ret));
    }
    return ret;
  }

#if UNIT_TEST_CAMERA
  sdmmc_card_print_info(stdout, card);
#endif

  ESP_LOGI(TAG, "SD card mounted successfully");

  return ESP_OK;
}

void camera_capture_task(void *pvParameters)
{
  ESP_LOGI(TAG, "Camera capture task started");
  vTaskDelay(pdMS_TO_TICKS(1000));

  while (1) {
    // Capture frame
    camera_fb_t *fb = esp_camera_fb_get();

    if (fb == NULL) {
      ESP_LOGE(TAG, "Camera capture failed");
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }

    LOG_DEBUG("Captured: %d bytes (%dx%d)", fb->len, fb->width, fb->height);

    // ===== SEND TO QUEUE FOR SPI TRANSMISSION =====
    if (tx_queue != NULL) {
      // Get buffer from PSRAM pool
      uint8_t *tx_buffer = image_buffer_alloc();

      if (tx_buffer != NULL) {
        memcpy(tx_buffer, fb->buf, fb->len);

        image_data_t img_data = {
          .buffer = tx_buffer,
          .size = fb->len
        };

        if (xQueueSend(tx_queue, &img_data, 0) == pdTRUE) {
          LOG_DEBUG("→ Queued for SPI transmission");
        } else {
          ESP_LOGW(TAG, "SPI queue full, dropping image");
          image_buffer_free(tx_buffer);
        }
      } else {
        ESP_LOGW(TAG, "No free buffers, skipping SPI transmission");
      }
    }

    // ===== SAVE TO SD CARD =====
    char filename[32];
    snprintf(filename, sizeof(filename), "/sdcard/img_%04d.raw", image_counter++);

    FILE *f = fopen(filename, "wb");
    if (f == NULL) {
      ESP_LOGE(TAG, "Failed to open file: %s", filename);
    } else {
      size_t written = fwrite(fb->buf, 1, fb->len, f);
      fclose(f);

      if (written == fb->len) {
        LOG_DEBUG("Saved: %s (%d bytes)", filename, written);
      } else {
        ESP_LOGE(TAG, "Write failed: %d/%d bytes", written, fb->len);
      }
    }

    // Return camera buffer
    esp_camera_fb_return(fb);

    // Capture every 2 seconds
    vTaskDelay(pdMS_TO_TICKS(2000));
  }
}

void cam_log_unit_test(QueueHandle_t queue)
{
  tx_queue = queue;  // Store queue handle for SPI transmission

  ESP_LOGI(TAG, "ESP32-CAM System Initializing");

  // Initialize SD card FIRST
  if (sd_card_init() != ESP_OK) {
    ESP_LOGE(TAG, "SD card init failed - halting");
    return;
  }

  LOG_DEBUG("Initializing camera...");

  // Initialize camera
  esp_err_t err = esp_camera_init(&camera_config);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Camera init failed: 0x%x", err);
    return;
  }

  ESP_LOGI(TAG, "Camera initialized successfully");

  // Create capture task
  xTaskCreate(camera_capture_task, "cam_task", 4096, NULL, 5, NULL);

  ESP_LOGI(TAG, "System ready");
}
