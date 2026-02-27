/*
 * camera_control.h
 * ESP32-CAM camera control interface
 */

#ifndef CAMERA_CONTROL_H
#define CAMERA_CONTROL_H

/* Uncomment for burst capture test mode */
#define TEST_BURST_CAPTURE

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

/**
 * Initialize OV2640 camera and apply sensor configuration.
 * Resolution set by build mode: QVGA (test) or VGA (production).
 */
esp_err_t camera_init(void);

/**
 * Set the SPI transmit queue for downstream image data
 */
void camera_set_tx_queue(QueueHandle_t queue);

/**
 * Start the capture task.
 * Test mode:  burst dump to SD
 * Production: SPI every frame, JPEG to SD every 3s
 */
void camera_start_capture(void);

#endif /* CAMERA_CONTROL_H */
