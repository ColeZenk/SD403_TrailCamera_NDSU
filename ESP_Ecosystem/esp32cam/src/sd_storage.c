/*
 * sd_storage.c
 * SD card storage implementation for image logging
 */

#include "sd_storage.h"
#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdmmc_host.h"

static const char *TAG = "SD_STORAGE";

/**************************************************************************************************
  File Scope Variables
**************************************************************************************************/

static sdmmc_card_t *card = NULL;
static int image_counter = 0;
static bool initialized = false;

/**************************************************************************************************
  Global Interface
**************************************************************************************************/

esp_err_t sd_card_init(void)
{
    if (initialized) {
        ESP_LOGW(TAG, "SD card already initialized");
        return ESP_OK;
    }

    LOG_DEBUG("Initializing SD card...");

    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };

    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    host.max_freq_khz = SDMMC_FREQ_HIGHSPEED;  /* 40 MHz */

    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    slot_config.width = 4;  /* 4-bit mode */

    esp_err_t ret = esp_vfs_fat_sdmmc_mount("/sdcard", &host, &slot_config, &mount_config, &card);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount filesystem. Insert SD card and reset.");
        } else {
            ESP_LOGE(TAG, "Failed to initialize SD card: %s", esp_err_to_name(ret));
        }
        return ret;
    }

#if UNIT_TEST_SD_STORAGE
    sdmmc_card_print_info(stdout, card);
#endif

    initialized = true;
    ESP_LOGI(TAG, "SD card mounted successfully");

    return ESP_OK;
}

esp_err_t sd_save_image(const uint8_t *buffer, size_t length)
{
    if (!initialized) {
        ESP_LOGE(TAG, "SD card not initialized!");
        return ESP_FAIL;
    }

    char filename[32];
    snprintf(filename, sizeof(filename), "/sdcard/img_%04d.jpg", image_counter++);

    return sd_save_image_named(buffer, length, filename);
}

esp_err_t sd_save_image_named(const uint8_t *buffer, size_t length, const char *filename)
{
    if (!initialized) {
        ESP_LOGE(TAG, "SD card not initialized!");
        return ESP_FAIL;
    }

    if (buffer == NULL || filename == NULL) {
        ESP_LOGE(TAG, "Invalid parameters");
        return ESP_ERR_INVALID_ARG;
    }

    FILE *f = fopen(filename, "wb");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file: %s", filename);
        return ESP_FAIL;
    }

    size_t written = fwrite(buffer, 1, length, f);
    fclose(f);

    if (written != length) {
        ESP_LOGE(TAG, "Write failed: %zu/%zu bytes", written, length);
        return ESP_ERR_INVALID_SIZE;
    }

    LOG_DEBUG("Saved: %s (%zu bytes)", filename, written);
    return ESP_OK;
}

int sd_get_image_counter(void)
{
    return image_counter;
}

void sd_reset_image_counter(void)
{
    image_counter = 0;
    ESP_LOGI(TAG, "Image counter reset to 0");
}
