/*
 * cam_ctrl.c
 * ESP32-CAM camera initialization and frame capture
 *
 * Production mode:
 *   - SPI pipeline: every grayscale frame pushed downstream at 30 fps
 *   - Save ring: every SAVE_INTERVAL'th frame copied to PSRAM (no SD, no JPEG)
 *   - Post-capture: after STOP command, JPEG encode + timestamped SD write
 *
 * Test burst mode (TEST_BURST_CAPTURE):
 *   - No SPI — SD mounted for entire capture
 *   - Dumps N raw grayscale frames to SD at max rate
 */

#include "cam_ctrl.h"

#include <stdio.h>
#include <string.h>

#include "dma_spi.h"
#include "esp_camera.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "img_buf.h"
#include "img_data.h"
#include "img_converters.h"
#include "ov2640_cfg.h"
#include "sd_stg.h"

static const char *TAG = "CAM_CTRL";

/*******************************************************************************
 * Configuration
 ******************************************************************************/

#define FRAME_PERIOD_MS     33 /* 30 fps target (1000 / 30) */
#define BURST_FRAME_COUNT   30
#define BURST_DISCARD_COUNT 5
#define JPEG_QUALITY        80

static const camera_config_t hw_config = {
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
    .frame_size = FRAMESIZE_QVGA, /* 320x240 for all modes */
    .fb_count = 3,                /* triple buffer in PSRAM */
    .fb_location = CAMERA_FB_IN_PSRAM,
    .grab_mode = CAMERA_GRAB_LATEST, /* always freshest frame */
};

/*******************************************************************************
 * File Scope State
 ******************************************************************************/

static QueueHandle_t tx_queue = NULL;
static uint32_t frame_count = 0;

/*******************************************************************************
 * SPI Queue Helper
 ******************************************************************************/

static void enqueue_frame(const uint8_t *buf, size_t len)
{
        uint8_t *tx = img_buffer_alloc();
        if (!tx) {
                ESP_LOGW(TAG, "No free buffers, dropping frame");
                return;
        }

        size_t n = (len > IMAGE_BUFFER_SIZE) ? IMAGE_BUFFER_SIZE : len;
        memcpy(tx, buf, n);

        img_data_t img = {.buffer = tx, .size = n};

        if (xQueueSend(tx_queue, &img, 0) != pdTRUE) {
                ESP_LOGW(TAG, "SPI queue full, dropping frame");
                img_buffer_free(tx);
        }
}

/*******************************************************************************
 * Post-Capture: batch JPEG encode + timestamped SD write
 ******************************************************************************/

static void post_capture_save(void)
{
        int n = save_ring_count();
        if (n == 0) {
                ESP_LOGI(TAG, "No frames in save ring");
                return;
        }

        uint32_t epoch = spi_dma_epoch();

        ESP_LOGI(TAG, "Post-capture: %d frames to encode + save", n);

        /* SPI off — release GPIO 13 for SD */
        spi_dma_deinit();

        if (sd_card_init() != ESP_OK) {
                ESP_LOGE(TAG, "SD mount failed — %d frames lost", n);
                return;
        }

        for (int i = 0; i < n; i++) {
                int64_t ts = 0;
                const uint8_t *raw = save_ring_get(i, &ts);
                if (!raw) continue;

                uint8_t *jpg = NULL;
                size_t jpg_len = 0;

                if (!fmt2jpg((uint8_t *)raw, IMAGE_BUFFER_SIZE, 320, 240,
                             PIXFORMAT_GRAYSCALE, JPEG_QUALITY, &jpg,
                             &jpg_len)) {
                        ESP_LOGE(TAG, "JPEG encode failed, frame %d", i);
                        continue;
                }

                esp_err_t ret;
                if (epoch > 0) {
                        ret = sd_save_timestamped(jpg, jpg_len, epoch, ts, i);
                } else {
                        /* No epoch from DevKit — fallback filename */
                        char name[48];
                        snprintf(name, sizeof(name),
                                 "/sdcard/img_unknown_%03d.jpg", i);
                        ret = sd_save_img_named(jpg, jpg_len, name);
                }

                if (ret == ESP_OK) {
                        ESP_LOGI(TAG, "  [%d/%d] %zu bytes (%.1f:1)", i + 1, n,
                                 jpg_len, (float)IMAGE_BUFFER_SIZE / jpg_len);
                }

                free(jpg);
        }

        sd_card_deinit();
        save_ring_reset();

        ESP_LOGI(TAG, "Post-capture save complete");
}

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
                if (!fb) {
                        failed++;
                        vTaskDelay(pdMS_TO_TICKS(10));
                        continue;
                }

                if (sd_save_raw(fb->buf, fb->len) == ESP_OK) saved++;
                else failed++;

                esp_camera_fb_return(fb);
        }

        float elapsed = (esp_timer_get_time() - t0) / 1e6f;

        ESP_LOGI(TAG, "=== BURST COMPLETE ===");
        ESP_LOGI(TAG, "  %d saved, %d failed, %.2fs, %.1f fps", saved, failed,
                 elapsed, saved / elapsed);
        ESP_LOGI(TAG, "Pull SD card and run analysis.");

        for (;;) vTaskDelay(pdMS_TO_TICKS(10000));
}

#else

static void capture_loop(void *arg)
{
        ESP_LOGI(TAG,
                 "Capture loop started — %d fps target, save every %d frames",
                 1000 / FRAME_PERIOD_MS, SAVE_INTERVAL);

        TickType_t last_wake = xTaskGetTickCount();

        for (;;) {
                /* Check for STOP command from DevKit */
                if (spi_dma_stop_requested()) {
                        ESP_LOGI(TAG, "STOP received — exiting capture loop");
                        break;
                }

                camera_fb_t *fb = esp_camera_fb_get();
                if (!fb) {
                        vTaskDelayUntil(&last_wake,
                                        pdMS_TO_TICKS(FRAME_PERIOD_MS));
                        continue;
                }

                /* Every frame goes downstream over SPI */
                if (tx_queue) enqueue_frame(fb->buf, fb->len);

                /* Every SAVE_INTERVAL'th frame → PSRAM ring (no SD, no JPEG)
                 */
                frame_count++;
                if (frame_count % SAVE_INTERVAL == 0) {
                        save_ring_push(fb->buf, fb->len, esp_timer_get_time());
                }

                esp_camera_fb_return(fb);

                vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(FRAME_PERIOD_MS));
        }

        /* Post-capture phase: JPEG encode + SD save + sleep */
        ESP_LOGI(TAG, "Entering post-capture phase");

        post_capture_save();

        ESP_LOGI(TAG, "Entering deep sleep");
        esp_deep_sleep_start();
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

        ESP_LOGI(TAG, "Camera initialized: 320x240 grayscale");

        /* Sensor config — locked exposure, auto-tuned to scene brightness */
        ov2640_cfg_t sensor_cfg = OV2640_LOCKED_DEFAULTS;
        ov2640_apply_config(&sensor_cfg);

        return ESP_OK;
}

void camera_set_tx_queue(QueueHandle_t queue) { tx_queue = queue; }

void camera_start_capture(void)
{
#ifdef TEST_BURST_CAPTURE
        xTaskCreatePinnedToCore(burst_capture_task, "burst", 8192, NULL, 6,
                                NULL, 0);
#else
        xTaskCreatePinnedToCore(capture_loop, "capture", 8192, NULL, 6, NULL,
                                0);
#endif
}
