#pragma once

#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t  seconds;
    uint8_t  minutes;
    uint8_t  hours;
    uint8_t  day;
    uint8_t  month;
    uint16_t year;
} rtc_time_t;

esp_err_t ds3231_init(void);
esp_err_t ds3231_set_time(rtc_time_t *t);
esp_err_t ds3231_get_time(rtc_time_t *t);
void      ds3231_log_time(const char *tag, int pir, float temp_f, float rh);

#ifdef __cplusplus
}
#endif