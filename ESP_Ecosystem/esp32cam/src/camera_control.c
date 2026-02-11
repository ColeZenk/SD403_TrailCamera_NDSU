/*
 * camera_control.c
 * ESP32-CAM camera initialization and frame capture
 */

#include <stdio.h>
#include <string.h>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_log.h"
#include "esp_camera.h"

#include "image_buffer_pool.h"
#include "sd_storage.h"
#include "sensor.h"
#include "camera_control.h"

/* File scope variables */
static QueueHandle_t tx_queue = NULL;
static const char *TAG = "CAM_CTRL";

/**************************************************************************************************
  Core Logical Function
**************************************************************************************************/

/**
 * Capture a single frame from camera
 * Saves to SD and optionally sends to SPI queue
 *
 * @param pv_parameters FreeRTOS task parameters (unused)
 */
static void capture_frame(void *pv_parameters)
{
    ESP_LOGI(TAG, "Camera capture initiated");

    camera_fb_t *fb = esp_camera_fb_get();

    if (fb == NULL) {
        ESP_LOGE(TAG, "Camera capture failed");
        return;
    }

    LOG_DEBUG("Captured: %zu bytes (%dx%d)", fb->len, fb->width, fb->height);

// TODO: Tailor this function to local hosting instead of DMA
//    if (tx_queue != NULL) {
//        send_frame_to_spi_queue(fb->buf, fb->len);
//    }

    /* Save to SD card */
    sd_save_image(fb->buf, fb->len);

    /* Return camera buffer to driver */
    esp_camera_fb_return(fb);
}

/**************************************************************************************************
  Global Interface
**************************************************************************************************/

esp_err_t camera_init(void)
{
    /* AI-Thinker ESP32-CAM Camera Configuration */
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
        .frame_size = FRAMESIZE_VGA,  /* 640x480 */
        /* .jpeg_quality = 12, */
        .fb_count = 2,
        .fb_location = CAMERA_FB_IN_PSRAM,
        .grab_mode = CAMERA_GRAB_WHEN_EMPTY
    };

    LOG_DEBUG("Initializing camera...");

    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera init failed: 0x%x", err);
        return err;
    }

    ESP_LOGI(TAG, "Camera initialized successfully");
    return ESP_OK;
}

/**************************************************************************************************
  File Scope Functions
**************************************************************************************************/
// TODO: Tailor this function to local hosting instead of DMA
/* static inline void send_frame_to_spi_queue(const uint8_t *buffer, size_t length) */
/* { */
/*     /1* Get buffer from PSRAM pool *1/ */
/*     uint8_t *tx_buffer = image_buffer_alloc(); */

/*     if (tx_buffer != NULL) { */
/*         memcpy(tx_buffer, buffer, length); */

/*         image_data_t img_data = { */
/*             .buffer = tx_buffer, */
/*             .size = length */
/*         }; */

/*         if (xQueueSend(tx_queue, &img_data, 0) == pdTRUE) { */
/*             LOG_DEBUG("→ Queued for SPI transmission"); */
/*         } else { */
/*             ESP_LOGW(TAG, "SPI queue full, dropping image"); */
/*             image_buffer_free(tx_buffer); */
/*         } */
/*     } else { */
/*         ESP_LOGW(TAG, "No free buffers, skipping SPI transmission"); */
/*     } */
/* } */

/**************************************************************************************************
  Testing and Experimental
**************************************************************************************************/

#ifdef UNIT_TEST_CAMERA
void cam_log_unit_test(QueueHandle_t queue)
{
    tx_queue = queue;  /* Store queue handle for SPI transmission */

    ESP_LOGI(TAG, "ESP32-CAM System Initializing");

    /* Initialize SD card first */
    if (sd_card_init() != ESP_OK) {
        ESP_LOGE(TAG, "SD card init failed - halting");
        return;
    }

    /* Initialize camera */
    if (camera_init() != ESP_OK) {
        ESP_LOGE(TAG, "Camera init failed - halting");
        return;
    }

    /* Create capture task */
    xTaskCreate(camera_capture_task, "cam_task", 4096, NULL, 5, NULL);

    ESP_LOGI(TAG, "System ready");
}
#endif

#ifdef ISOLATED_CAM

#include "driver/gptimer.h"

static SemaphoreHandle_t trigger_capture = NULL;
static gptimer_handle_t timer_handle = NULL;

static bool IRAM_ATTR time_to_capture(gptimer_handle_t timer,
                                       const gptimer_alarm_event_data_t *event_data,
                                       void *user_ctx)
{
    BaseType_t higher_priority_task_woken = pdFALSE;
    xSemaphoreGiveFromISR(trigger_capture, &higher_priority_task_woken);
    return higher_priority_task_woken == pdTRUE;
}

esp_err_t camera_timer_init(uint32_t interval_ms)
{
    trigger_capture = xSemaphoreCreateBinary();
    if (trigger_capture == NULL) {
        return ESP_FAIL;
    }

    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000,  /* 1MHz, 1 tick = 1µs */
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &timer_handle));

    gptimer_alarm_config_t alarm_config = {
        .reload_count = 0,
        .alarm_count = interval_ms * 1000,  /* Convert ms to µs */
        .flags.auto_reload_on_alarm = true,
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(timer_handle, &alarm_config));

    gptimer_event_callbacks_t cbs = {
        .on_alarm = time_to_capture,
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(timer_handle, &cbs, NULL));

    ESP_ERROR_CHECK(gptimer_enable(timer_handle));
    ESP_ERROR_CHECK(gptimer_start(timer_handle));

    ESP_LOGI(TAG, "Timer initialized: %lu ms interval", interval_ms);
    return ESP_OK;
}

void isolated_capture_task(void *pv_params)
{
    ESP_LOGI(TAG, "Isolated capture task waiting for timer triggers...");

    for (;;) {
        if (xSemaphoreTake(trigger_capture, portMAX_DELAY) == pdTRUE) {
            capture_frame(pv_params);
        }
    }
}
#endif
