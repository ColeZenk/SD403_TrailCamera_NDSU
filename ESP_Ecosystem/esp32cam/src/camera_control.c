/*
 * camera_control.c
 * ESP32-CAM camera initialization and frame capture
 *
 * Production mode:
 *   - SPI pipeline: grayscale frames paced at 30 fps
 *   - SD save: JPEG every SD_SAVE_INTERVAL_FRAMES frames (pin time-shared with SPI)
 *
 * Test burst mode (TEST_BURST_CAPTURE):
 *   - No SPI — SD mounted for entire capture
 *   - Dumps N raw grayscale frames to SD at max rate
 */

#include <stdio.h>
#include <string.h>

#include "esp_err.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_camera.h"
#include "image_buffer_pool.h"
#include "image_data.h"
#include "sd_storage.h"
#include "DMA_SPI_master.h"
#include "camera_control.h"
#include "ov2640_config.h"

#ifndef TEST_BURST_CAPTURE
#include "img_converters.h"
#endif

static const char *TAG = "CAM_CTRL";

/*******************************************************************************
 * Configuration
 ******************************************************************************/

#define SD_SAVE_INTERVAL_FRAMES  10
#define FRAME_PERIOD_MS          33   /* 30 fps target (1000 / 30 ≈ 33 ms) */
#define BURST_FRAME_COUNT    30
#define BURST_DISCARD_COUNT  5

static const camera_config_t hw_config = {
    .pin_pwdn    = 32,
    .pin_reset   = -1,
    .pin_xclk    = 0,
    .pin_sccb_sda = 26,
    .pin_sccb_scl = 27,

    .pin_d7 = 35,  .pin_d6 = 34,  .pin_d5 = 39,  .pin_d4 = 36,
    .pin_d3 = 21,  .pin_d2 = 19,  .pin_d1 = 18,  .pin_d0 = 5,
    .pin_vsync = 25,
    .pin_href  = 23,
    .pin_pclk  = 22,

    .xclk_freq_hz = 20000000,
    .ledc_timer   = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    .pixel_format = PIXFORMAT_GRAYSCALE,
#ifdef TEST_BURST_CAPTURE
    .frame_size   = FRAMESIZE_QVGA,
#else
    .frame_size   = FRAMESIZE_VGA,
#endif
    .fb_count     = 2,
    .fb_location  = CAMERA_FB_IN_PSRAM,
    .grab_mode    = CAMERA_GRAB_WHEN_EMPTY,
};

/*******************************************************************************
 * File Scope State
 ******************************************************************************/

static QueueHandle_t tx_queue    = NULL;
static uint32_t      frame_count = 0;

/*******************************************************************************
 * SPI Queue Helper
 ******************************************************************************/

static void enqueue_frame(const uint8_t *buf, size_t len)
{
    uint8_t *tx = image_buffer_alloc();
    if (!tx) {
        ESP_LOGW(TAG, "No free buffers, dropping frame");
        return;
    }

    size_t n = (len > IMAGE_BUFFER_SIZE) ? IMAGE_BUFFER_SIZE : len;
    memcpy(tx, buf, n);

    image_data_t img = { .buffer = tx, .size = n };

    if (xQueueSend(tx_queue, &img, 0) != pdTRUE) {
        ESP_LOGW(TAG, "SPI queue full, dropping frame");
        image_buffer_free(tx);
    }
}

/*******************************************************************************
 * Production: JPEG SD Save (pin time-shared)
 ******************************************************************************/

#ifndef TEST_BURST_CAPTURE

static void save_jpeg_to_sd(const uint8_t *raw, size_t raw_len,
                            uint16_t w, uint16_t h)
{
    uint8_t *jpg = NULL;
    size_t   jpg_len = 0;

    if (!fmt2jpg((uint8_t *)raw, raw_len, w, h,
                 PIXFORMAT_GRAYSCALE, 80, &jpg, &jpg_len)) {
        ESP_LOGE(TAG, "JPEG encode failed");
        return;
    }

    ESP_LOGI(TAG, "JPEG: %zu bytes (%.1f:1)", jpg_len, (float)raw_len / jpg_len);

    /* Pin swap: SPI off → SD on → save → SD off → SPI on */
    spi_dma_deinit();

    if (sd_card_init() == ESP_OK) {
        sd_save_image(jpg, jpg_len);
        sd_card_deinit();
    }

    spi_dma_init();
    free(jpg);
}

#endif /* !TEST_BURST_CAPTURE */

/*******************************************************************************
 * Capture Tasks
 ******************************************************************************/

#ifdef TEST_BURST_CAPTURE

static void burst_capture_task(void *arg)
{
    ESP_LOGI(TAG, "=== BURST CAPTURE MODE ===");

    /* Discard startup frames */
    for (int i = 0; i < BURST_DISCARD_COUNT; i++) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (fb) esp_camera_fb_return(fb);
    }

    ESP_LOGI(TAG, "Capturing %d raw frames...", BURST_FRAME_COUNT);
    int64_t t0 = esp_timer_get_time();
    int saved = 0, failed = 0;

    for (int i = 0; i < BURST_FRAME_COUNT; i++) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) { failed++; vTaskDelay(pdMS_TO_TICKS(10)); continue; }

        if (sd_save_raw(fb->buf, fb->len) == ESP_OK)
            saved++;
        else
            failed++;

        esp_camera_fb_return(fb);
    }

    float elapsed = (esp_timer_get_time() - t0) / 1e6f;

    ESP_LOGI(TAG, "=== BURST COMPLETE ===");
    ESP_LOGI(TAG, "  %d saved, %d failed, %.2fs, %.1f fps",
             saved, failed, elapsed, saved / elapsed);
    ESP_LOGI(TAG, "Pull SD card and run analysis.");

    for (;;) vTaskDelay(pdMS_TO_TICKS(10000));
}

#else

static void capture_loop(void *arg)
{
    ESP_LOGI(TAG, "Capture loop started — target %d fps, SD every %d frames",
             1000 / FRAME_PERIOD_MS, SD_SAVE_INTERVAL_FRAMES);

    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) {
            vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(FRAME_PERIOD_MS));
            continue;
        }

        if (tx_queue) enqueue_frame(fb->buf, fb->len);

        frame_count++;
        if (frame_count % SD_SAVE_INTERVAL_FRAMES == 0)
            save_jpeg_to_sd(fb->buf, fb->len, fb->width, fb->height);

        esp_camera_fb_return(fb);

        /* Pace to FRAME_PERIOD_MS — sleeps the remainder of the period.
         * If capture + SPI overruns the budget, runs immediately next cycle. */
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(FRAME_PERIOD_MS));
    }
}

#endif /* TEST_BURST_CAPTURE */

/*******************************************************************************
 * Global Interface
 ******************************************************************************/

esp_err_t camera_init(void)
{
    esp_err_t err = esp_camera_init(&hw_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera init failed: 0x%x", err);
        return err;
    }

#ifdef TEST_BURST_CAPTURE
    ESP_LOGI(TAG, "Camera initialized: 320x240 grayscale (burst mode)");
#else
    ESP_LOGI(TAG, "Camera initialized: 640x480 grayscale");
#endif

    /* Sensor config — locked exposure, auto-tuned to scene brightness */
    ov2640_config_t sensor_cfg = OV2640_LOCKED_DEFAULTS;
    ov2640_apply_config(&sensor_cfg);

    return ESP_OK;
}

void camera_set_tx_queue(QueueHandle_t queue)
{
    tx_queue = queue;
}

void camera_start_capture(void)
{
#ifdef TEST_BURST_CAPTURE
    xTaskCreatePinnedToCore(burst_capture_task, "burst",   8192, NULL, 6, NULL, 0);
#else
    xTaskCreatePinnedToCore(capture_loop,       "capture", 8192, NULL, 6, NULL, 0);
#endif
}
