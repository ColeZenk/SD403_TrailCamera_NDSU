/**
 * @file rtc.c
 * @brief DS3231 RTC driver + optional one-shot time setter
 */

#include "peripherals/rtc.h"

#include <stdio.h>
#include <string.h>
#include <sys/time.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"

static const char *TAG = "rtc";

#define REG_TIME   0x00
#define REG_STATUS 0x0F
#define REG_TEMP   0x11

#define STATUS_OSF 0x80

#define I2C_TIMEOUT pdMS_TO_TICKS(100)

static i2c_port_t s_port = I2C_NUM_0;

static inline uint8_t bcd2dec(uint8_t v) { return (v >> 4) * 10 + (v & 0x0F); }
static inline uint8_t dec2bcd(int v) { return (uint8_t)(((v / 10) << 4) | (v % 10)); }

/* ---- low-level helpers: swap these two if you move to i2c_bus helpers ---- */

static esp_err_t reg_read(uint8_t reg, uint8_t *buf, size_t len)
{
        return i2c_master_write_read_device(s_port, DS3231_I2C_ADDR, &reg, 1,
                                            buf, len, I2C_TIMEOUT);
}

static esp_err_t reg_write(uint8_t reg, const uint8_t *data, size_t len)
{
        uint8_t buf[8];
        if (len > sizeof(buf) - 1) return ESP_ERR_INVALID_SIZE;

        buf[0] = reg;
        memcpy(&buf[1], data, len);
        return i2c_master_write_to_device(s_port, DS3231_I2C_ADDR, buf,
                                          len + 1, I2C_TIMEOUT);
}

/* ------------------------------------------------------------------------ */

esp_err_t ds3231_init(i2c_port_t port)
{
        s_port = port;

        uint8_t status;
        esp_err_t err = reg_read(REG_STATUS, &status, 1);
        if (err != ESP_OK) {
                ESP_LOGE(TAG, "DS3231 not responding: %s", esp_err_to_name(err));
                return err;
        }

        if (status & STATUS_OSF) {
                ESP_LOGW(TAG, "oscillator was stopped - time is not valid");
        }
        return ESP_OK;
}

esp_err_t ds3231_get_time(struct tm *t)
{
        uint8_t b[7];
        esp_err_t err = reg_read(REG_TIME, b, sizeof(b));
        if (err != ESP_OK) return err;

        memset(t, 0, sizeof(*t));
        t->tm_sec = bcd2dec(b[0] & 0x7F);
        t->tm_min = bcd2dec(b[1] & 0x7F);

        if (b[2] & 0x40) { // 12h mode (we always write 24h, but be tolerant)
                int h = bcd2dec(b[2] & 0x1F);
                bool pm = b[2] & 0x20;
                t->tm_hour = (h % 12) + (pm ? 12 : 0);
        } else {
                t->tm_hour = bcd2dec(b[2] & 0x3F);
        }

        t->tm_wday  = (b[3] & 0x07) - 1;
        t->tm_mday  = bcd2dec(b[4] & 0x3F);
        t->tm_mon   = bcd2dec(b[5] & 0x1F) - 1;
        t->tm_year  = bcd2dec(b[6]) + 100; // 2000 + yy
        t->tm_isdst = 0;
        return ESP_OK;
}

esp_err_t ds3231_set_time(const struct tm *in)
{
        int year = in->tm_year + 1900;
        if (year < 2000 || year > 2099) return ESP_ERR_INVALID_ARG;

        // Let mktime fill in the weekday for us
        struct tm t = *in;
        t.tm_isdst = 0;
        mktime(&t);

        uint8_t b[7] = {
            dec2bcd(t.tm_sec),
            dec2bcd(t.tm_min),
            dec2bcd(t.tm_hour), // bit 6 = 0 -> 24h mode
            (uint8_t)(t.tm_wday + 1),
            dec2bcd(t.tm_mday),
            dec2bcd(t.tm_mon + 1),
            dec2bcd(year - 2000),
        };

        esp_err_t err = reg_write(REG_TIME, b, sizeof(b));
        if (err != ESP_OK) return err;

        // Clear oscillator-stop flag now that the time is valid
        uint8_t status;
        err = reg_read(REG_STATUS, &status, 1);
        if (err != ESP_OK) return err;

        status &= (uint8_t)~STATUS_OSF;
        return reg_write(REG_STATUS, &status, 1);
}

esp_err_t ds3231_lost_power(bool *lost)
{
        uint8_t status;
        esp_err_t err = reg_read(REG_STATUS, &status, 1);
        if (err != ESP_OK) return err;

        *lost = (status & STATUS_OSF) != 0;
        return ESP_OK;
}

esp_err_t ds3231_get_temp(float *temp_c)
{
        uint8_t b[2];
        esp_err_t err = reg_read(REG_TEMP, b, sizeof(b));
        if (err != ESP_OK) return err;

        *temp_c = (int8_t)b[0] + (b[1] >> 6) * 0.25f;
        return ESP_OK;
}

esp_err_t ds3231_sync_system_time(void)
{
        struct tm t;
        esp_err_t err = ds3231_get_time(&t);
        if (err != ESP_OK) return err;

        struct timeval tv = {.tv_sec = mktime(&t), .tv_usec = 0};
        if (settimeofday(&tv, NULL) != 0) return ESP_FAIL;

        ESP_LOGI(TAG, "system clock set from RTC: %04d-%02d-%02d %02d:%02d:%02d",
                 t.tm_year + 1900, t.tm_mon + 1, t.tm_mday, t.tm_hour,
                 t.tm_min, t.tm_sec);
        return ESP_OK;
}

/* ---- one-shot time setter (only compiled when RTC_SET_TIME_ONCE) -------- */

#ifdef RTC_SET_TIME_ONCE

esp_err_t ds3231_set_time_manual(int year, int month, int day, int hour, int min,
                              int sec)
{
        struct tm t = {
            .tm_year = year - 1900,
            .tm_mon  = month - 1,
            .tm_mday = day,
            .tm_hour = hour,
            .tm_min  = min,
            .tm_sec  = sec,
        };

        esp_err_t err = ds3231_set_time(&t);
        if (err != ESP_OK) {
                ESP_LOGE(TAG, "ds3231_set_time failed: %s", esp_err_to_name(err));
                return err;
        }

        ESP_LOGW(TAG, "RTC set to %04d-%02d-%02d %02d:%02d:%02d", year, month,
                 day, hour, min, sec);
        return ESP_OK;
}

esp_err_t ds3231_set_time_from_build(void)
{
        // __DATE__ = "Oct  3 2026", __TIME__ = "14:05:09"
        static const char months[] = "JanFebMarAprMayJunJulAugSepOctNovDec";

        char mon[4] = {0};
        int day, year, hour, min, sec;

        if (sscanf(__DATE__, "%3s %d %d", mon, &day, &year) != 3 ||
            sscanf(__TIME__, "%d:%d:%d", &hour, &min, &sec) != 3) {
                ESP_LOGE(TAG, "could not parse build date/time");
                return ESP_FAIL;
        }

        const char *p = strstr(months, mon);
        if (!p) return ESP_FAIL;
        int month = (int)(p - months) / 3 + 1;

        return ds3231_set_time_manual(year, month, day, hour, min, sec);
}

#endif // RTC_SET_TIME_ONCE
