#ifndef ESP32CAM_CONTROL_H
#define ESP32CAM_CONTROL_H
#include "esp_err.h"
#define CAM_RESET_PIN 2
#define CAM_BOOT_PIN 4
esp_err_t esp32cam_control_init(void);
#endif
