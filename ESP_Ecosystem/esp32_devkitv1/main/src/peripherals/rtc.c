#include "peripherals/rtc.h"
#include "peripherals/i2c_bus.h"
#include "esp_log.h"

static const char *TAG = "rtc";

#define DS3231_ADDR  0x68
#define REG_SECONDS  0x00

static uint8_t bcd_to_dec(uint8_t bcd)
{
    return (bcd >> 4) * 10 + (bcd & 0x0F);
}

static uint8_t dec_to_bcd(uint8_t dec)
{
    return ((dec / 10) << 4) | (dec % 10);
}

esp_err_t ds3231_set_time(rtc_time_t *t)
{
    if (!t) return ESP_ERR_INVALID_ARG;
    uint8_t buf[8] = {
        REG_SECONDS,
        dec_to_bcd(t->seconds),
        dec_to_bcd(t->minutes),
        dec_to_bcd(t->hours),
        0x01,
        dec_to_bcd(t->day),
        dec_to_bcd(t->month),
        dec_to_bcd(t->year - 2000),
    };
    esp_err_t err = i2c_bus_write(DS3231_ADDR, buf, sizeof(buf), 100);
    if (err != ESP_OK)
        ESP_LOGW(TAG, "ds3231_set_time failed: %s", esp_err_to_name(err));
    return err;
}

esp_err_t ds3231_get_time(rtc_time_t *t)
{
    if (!t) return ESP_ERR_INVALID_ARG;

    uint8_t reg = REG_SECONDS;
    uint8_t buf[7] = {0};

    esp_err_t err = i2c_bus_write_read(DS3231_ADDR, &reg, 1, buf, 7, 100);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "ds3231_get_time failed: %s", esp_err_to_name(err));
        return err;
    }

    t->seconds = bcd_to_dec(buf[0] & 0x7F);
    t->minutes = bcd_to_dec(buf[1] & 0x7F);
    t->hours   = bcd_to_dec(buf[2] & 0x3F);
    t->day     = bcd_to_dec(buf[4] & 0x3F);
    t->month   = bcd_to_dec(buf[5] & 0x1F);
    t->year    = bcd_to_dec(buf[6]) + 2000;

    return ESP_OK;
}

esp_err_t ds3231_init(void)
{
    uint8_t reg = REG_SECONDS;
    uint8_t val = 0;
    esp_err_t err = i2c_bus_write_read(DS3231_ADDR, &reg, 1, &val, 1, 100);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "DS3231 not found: %s", esp_err_to_name(err));
        return err;
    }
    ESP_LOGI(TAG, "DS3231 found at 0x%02X", DS3231_ADDR);

    // Set time if RTC has never been set (year comes back as 2000)
    //rtc_time_t now = {0};
    //ds3231_get_time(&now);
    //if (now.year < 2024) {
    //    rtc_time_t set = {
    //        .seconds = 0,
    //        .minutes = 37,   // <-- update before flashing
    //        .hours   = 13,  // <-- update before flashing (24hr)
    //        .day     = 8,   // <-- update before flashing
    //        .month   = 3,   // <-- update before flashing
    //        .year    = 2026,
    //    };
    //    err = ds3231_set_time(&set);
    //    if (err == ESP_OK)
    //        ESP_LOGI(TAG, "RTC time set to %04d-%02d-%02d %02d:%02d:%02d",
    //                 set.year, set.month, set.day,
    //                 set.hours, set.minutes, set.seconds);
    //}

    return ESP_OK;
}

void ds3231_log_time(const char *tag, int pir, float temp_f, float rh)
{
    rtc_time_t t = {0};
    if (ds3231_get_time(&t) != ESP_OK) {
        ESP_LOGI(tag, "PIR%d | %.1f F | %.0f%% RH | time unavailable",
                 pir, temp_f, rh);
        return;
    }
    ESP_LOGI(tag, "PIR%d | %.1f F | %.0f%% RH | %04d-%02d-%02d %02d:%02d:%02d",
             pir, temp_f, rh,
             t.year, t.month, t.day,
             t.hours, t.minutes, t.seconds);
}