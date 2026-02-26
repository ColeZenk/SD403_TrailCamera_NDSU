/*
 * camera_control.h
 * ESP32-CAM camera control interface
 */

#ifndef CAMERA_CONTROL_H
#define CAMERA_CONTROL_H

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

/**
 * Initialize OV2640 camera — 640x480 grayscale, 2 PSRAM frame buffers
 */
esp_err_t camera_init(void);

/**
 * Set the SPI transmit queue for downstream image data
 */
void camera_set_tx_queue(QueueHandle_t queue);

/**
 * Start the capture loop task.
 * Runs on core 0, captures at max rate, sends every frame to SPI,
 * saves JPEG to SD every 3 seconds.
 */
void camera_start_capture(void);

#endif /* CAMERA_CONTROL_H */
