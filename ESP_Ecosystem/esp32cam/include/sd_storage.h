/*
 * sd_storage.h
 * SD card storage interface for image logging
 */

#ifndef SD_STORAGE_H
#define SD_STORAGE_H

#include "esp_err.h"
#include <stddef.h>
#include <stdint.h>

/* Debug flag - set to 1 to enable verbose logging */
#ifndef UNIT_TEST_SD_STORAGE
#define UNIT_TEST_SD_STORAGE 0
#endif

#if UNIT_TEST_SD_STORAGE
#define LOG_DEBUG(fmt, ...) ESP_LOGI(TAG, fmt, ##__VA_ARGS__)
#else
#define LOG_DEBUG(fmt, ...) do {} while(0)
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
 * Save image buffer to SD card as .jpg with auto filename
 */
esp_err_t sd_save_image(const uint8_t *buffer, size_t length);

/**
 * Save raw buffer to SD card as .raw with auto filename
 */
esp_err_t sd_save_raw(const uint8_t *buffer, size_t length);

/**
 * Save image buffer to SD card with specific filename
 */
esp_err_t sd_save_image_named(const uint8_t *buffer, size_t length, const char *filename);

/**
 * Get current image counter value
 */
int sd_get_image_counter(void);

/**
 * Reset image counter to zero
 */
void sd_reset_image_counter(void);

#endif /* SD_STORAGE_H */
