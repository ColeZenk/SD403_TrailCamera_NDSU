/*
 * camera_control.c
 * ESP32-CAM camera initialization and frame capture
 *
 * Two rates:
 *   - SPI pipeline: grayscale frames as fast as possible (~30fps target)
 *   - SD save: every 3 seconds, software JPEG encode + save
 *
 * SD and SPI share GPIO 13/14. Pin time-sharing:
 *   deinit SPI → mount SD → save → unmount SD → reinit SPI
 */

#include <stdio.h>
#include <string.h>

#include "esp_err.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_log.h"
#include "esp_camera.h"
#include "esp_heap_caps.h"

#include "image_buffer_pool.h"
#include "image_data.h"
#include "sd_storage.h"
#include "DMA_SPI_master.h"
#include "camera_control.h"

/* jpeg encoder — esp_jpeg component */
// #include "esp_jpeg_enc.h"

static QueueHandle_t tx_queue = NULL;
static const char *TAG = "CAM_CTRL";

#define SD_SAVE_INTERVAL_US  (3 * 1000 * 1000)
#define JPEG_BUF_SIZE        (64 * 1024)

static int64_t last_sd_save_us = 0;

/**************************************************************************************************
  File Scope Functions
**************************************************************************************************/

static void send_frame_to_spi_queue(const uint8_t *buffer, size_t length)
{
    uint8_t *tx_buffer = image_buffer_alloc();

    if (tx_buffer == NULL) {
        ESP_LOGW(TAG, "No free buffers, skipping SPI send");
        return;
    }

    size_t copy_len = (length > IMAGE_BUFFER_SIZE) ? IMAGE_BUFFER_SIZE : length;
    memcpy(tx_buffer, buffer, copy_len);

    image_data_t img_data = {
        .buffer = tx_buffer,
        .size = copy_len
    };

    if (xQueueSend(tx_queue, &img_data, 0) != pdTRUE) {
        ESP_LOGW(TAG, "SPI queue full, dropping frame");
        image_buffer_free(tx_buffer);
    }
}

/**
 * Software JPEG encode a grayscale frame.
 * Returns encoded size, or 0 on failure.
 */
static size_t jpeg_encode_grayscale(const uint8_t *raw, uint16_t width, uint16_t height,
                                     uint8_t *jpeg_buf, size_t jpeg_buf_size)
{
    jpeg_enc_info_t enc_info = {
        .width = width,
        .height = height,
        .src_type = JPEG_RAW_TYPE_GRAY,
        .subsampling = JPEG_SUB_SAMPLE_Y,
        .quality = 80,
        .task_enable = false,
    };

    jpeg_enc_handle_t encoder = NULL;
    if (jpeg_enc_open(&enc_info, &encoder) != ESP_OK) {
        ESP_LOGE(TAG, "JPEG encoder open failed");
        return 0;
    }

    int out_len = 0;
    esp_err_t ret = jpeg_enc_process(encoder, raw, raw + (width * height),
                                      jpeg_buf, jpeg_buf_size, &out_len);
    jpeg_enc_close(encoder);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "JPEG encode failed: %s", esp_err_to_name(ret));
        return 0;
    }

    return (size_t)out_len;
}

/**
 * Save a frame to SD as JPEG.
 * Pin time-sharing: deinit SPI → mount SD → save → unmount → reinit SPI
 * JPEG encoding happens BEFORE pin swap (CPU work, no pin conflict).
 */
static void save_frame_to_sd(const uint8_t *raw, size_t raw_len,
                              uint16_t width, uint16_t height)
{
    uint8_t *jpeg_buf = heap_caps_malloc(JPEG_BUF_SIZE, MALLOC_CAP_SPIRAM);
    if (jpeg_buf == NULL) {
        ESP_LOGW(TAG, "No PSRAM for JPEG buffer, skipping SD save");
        return;
    }

    /* Encode while SPI still running — pure CPU work, no pin conflict */
    size_t jpeg_len = jpeg_encode_grayscale(raw, width, height, jpeg_buf, JPEG_BUF_SIZE);
    if (jpeg_len == 0) {
        free(jpeg_buf);
        return;
    }

    ESP_LOGI(TAG, "JPEG: %zu bytes (%.1f:1)", jpeg_len, (float)raw_len / jpeg_len);

    /* --- pin swap: SPI off, SD on --- */
    spi_dma_deinit();

    if (sd_card_init() == ESP_OK) {
        sd_save_image(jpeg_buf, jpeg_len);
        sd_card_deinit();
    } else {
        ESP_LOGW(TAG, "SD mount failed, frame lost");
    }

    /* --- pin swap: SD off, SPI back --- */
    spi_dma_init();

    free(jpeg_buf);
}

/**************************************************************************************************
  Core Capture Loop
**************************************************************************************************/

static void capture_loop(void *pv_parameters)
{
    ESP_LOGI(TAG, "Capture loop started");

    last_sd_save_us = esp_timer_get_time();

    for (;;) {
        camera_fb_t *fb = esp_camera_fb_get();

        if (fb == NULL) {
            ESP_LOGW(TAG, "Capture failed, retrying");
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        /* Always send to SPI pipeline */
        if (tx_queue != NULL) {
            send_frame_to_spi_queue(fb->buf, fb->len);
        }

        /* Periodic SD save */
        int64_t now = esp_timer_get_time();
        if ((now - last_sd_save_us) >= SD_SAVE_INTERVAL_US) {
            save_frame_to_sd(fb->buf, fb->len, fb->width, fb->height);
            last_sd_save_us = esp_timer_get_time();
        }

        esp_camera_fb_return(fb);
        taskYIELD();
    }
}

/**************************************************************************************************
  Global Interface
**************************************************************************************************/

esp_err_t camera_init(void)
{
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
        .frame_size = FRAMESIZE_VGA,
        .fb_count = 2,
        .fb_location = CAMERA_FB_IN_PSRAM,
        .grab_mode = CAMERA_GRAB_LATEST_WHEN_EMPTY
    };

    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera init failed: 0x%x", err);
        return err;
    }

    ESP_LOGI(TAG, "Camera initialized: 640x480 grayscale");
    return ESP_OK;
}

void camera_set_tx_queue(QueueHandle_t queue)
{
    tx_queue = queue;
}

void camera_start_capture(void)
{
    xTaskCreatePinnedToCore(capture_loop, "capture", 8192, NULL, 6, NULL, 0);
}
