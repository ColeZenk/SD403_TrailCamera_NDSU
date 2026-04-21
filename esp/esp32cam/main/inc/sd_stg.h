/*
 * sd_stg.h
 * SD card storage interface for img logging
 */

#ifndef SD_STORAGE_H
#define SD_STORAGE_H

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

/* Debug flag - set to 1 to enable verbose logging */
#ifndef UNIT_TEST_SD_STORAGE
#define UNIT_TEST_SD_STORAGE 0
#endif

#if UNIT_TEST_SD_STORAGE
#define LOG_DEBUG(fmt, ...) ESP_LOGI(TAG, fmt, ##__VA_ARGS__)
#else
#define LOG_DEBUG(fmt, ...)                                                    \
        do {                                                                   \
        } while (0)
#endif

/**
 * Initialize SD card and mount filesystem
 */
esp_err_t sd_card_init(void);

/**
 * Unmount SD card and release resources
 */
void sd_card_deinit(void);

/**
 * Save img buffer to SD card as .jpg with auto filename
 */
esp_err_t sd_save_img(const uint8_t *buffer, size_t length);

/**
 * Save JPEG to SD with timestamped filename.
 * epoch: UTC seconds from DevKit RTC.
 * elapsed_us: esp_timer microseconds since boot at capture time.
 * seq: sequence number within burst (0-based).
 */
esp_err_t sd_save_timestamped(const uint8_t *jpg, size_t jpg_len,
                              uint32_t epoch, int64_t elapsed_us, int seq);

/**
 * Save raw buffer to SD card as .raw with auto filename
 */
esp_err_t sd_save_raw(const uint8_t *buffer, size_t length);

/**
 * Save img buffer to SD card with specific filename
 */
esp_err_t sd_save_img_named(const uint8_t *buffer, size_t length,
                              const char *filename);

/**
 * Get current img counter value
 */
int sd_get_img_counter(void);

/**
 * Reset img counter to zero
 */
void sd_reset_img_counter(void);

#endif /* SD_STORAGE_H */
