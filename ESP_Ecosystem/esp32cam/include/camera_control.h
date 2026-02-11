/*
 * camera_control.h
 * ESP32-CAM camera control interface
 */

#ifndef CAMERA_CONTROL_H
#define CAMERA_CONTROL_H

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include <stdint.h>

#define ISOLATED_CAM

/* Debug flag - set to 1 to enable verbose logging */
#ifndef UNIT_TEST_CAMERA
// #define UNIT_TEST_CAMERA 0
#endif

#if UNIT_TEST_CAMERA
#define LOG_DEBUG(fmt, ...) ESP_LOGI(TAG, fmt, ##__VA_ARGS__)
#else
#define LOG_DEBUG(fmt, ...) do {} while(0)
#endif

/**
 * Initialize camera hardware and driver
 *
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t camera_init(void);

#ifdef ISOLATED_CAM

#define CAPTURE_RATE_S      10
#define CAPTURE_RATE       (CAPTURE_RATE_S * 1000)

/**
 * Initialize hardware timer for periodic camera capture
 *
 * @param interval_ms Capture interval in milliseconds
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t camera_timer_init(uint32_t interval_ms);

/**
 * FreeRTOS task that waits for timer trigger to capture frames
 *
 * @param pv_params FreeRTOS task parameters
 */
void isolated_capture_task(void *pv_params);
#endif

#ifdef UNIT_TEST_CAMERA
/**
 * Unit test function for camera and SD logging
 *
 * @param queue SPI transmission queue handle (can be NULL)
 */
void cam_log_unit_test(QueueHandle_t queue);
#endif

#endif /* CAMERA_CONTROL_H */
