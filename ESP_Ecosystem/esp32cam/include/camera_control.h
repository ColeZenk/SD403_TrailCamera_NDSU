#ifndef CAMERA_CONTROL_H
#define CAMERA_CONTROL_H

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "image_data.h"

void camera_capture_task(void *pvParameters);
esp_err_t sd_card_init(void);
void cam_log_unit_test(QueueHandle_t queue);

#endif
