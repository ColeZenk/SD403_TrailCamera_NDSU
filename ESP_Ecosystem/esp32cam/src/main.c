/*
 * ESP32-CAM Main Application
 * Camera + SPI DMA Transmitter
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_camera.h"
#include "DMA_SPI_master.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdmmc_host.h"

static sdmmc_card_t *card = NULL;
static int image_counter = 0;


static const char *TAG = "MAIN";

#define SPI_CLOCK_SPEED    (1 * 1000 * 1000)  // 20 MHz - adjust based on your needs
#define DMA_CHANNEL        SPI_DMA_CH_AUTO
#define MAX_TRANSFER_SIZE  (4092)  // Maximum DMA transfer size (must be multiple of 4)

// Add SD card init function
esp_err_t sd_card_init(void)
{
    ESP_LOGI(TAG, "Initializing SD card...");
    
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };
    
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    host.max_freq_khz = SDMMC_FREQ_HIGHSPEED;  // 40 MHz
    
    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    slot_config.width = 4;  // 4-bit mode
    
    esp_err_t ret = esp_vfs_fat_sdmmc_mount("/sdcard", &host, &slot_config, &mount_config, &card);
    
    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount filesystem. Insert SD card and reset.");
        } else {
            ESP_LOGE(TAG, "Failed to initialize SD card: %s", esp_err_to_name(ret));
        }
        return ret;
    }
    
    sdmmc_card_print_info(stdout, card);
    ESP_LOGI(TAG, "SD card mounted successfully");
    
    return ESP_OK;
}

void camera_capture_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Camera capture task started");
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    while (1) {
        // Capture frame
        camera_fb_t *fb = esp_camera_fb_get();
        
        if (fb == NULL) {
            ESP_LOGE(TAG, "Camera capture failed");
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        
        ESP_LOGI(TAG, "Captured: %d bytes (%dx%d)", 
                 fb->len, fb->width, fb->height);
        
        // Save to SD card
        char filename[32];
        snprintf(filename, sizeof(filename), "/sdcard/img_%04d.raw", image_counter++);
        
        FILE *f = fopen(filename, "w");
        if (f == NULL) {
            ESP_LOGE(TAG, "Failed to open file: %s", filename);
        } else {
            size_t written = fwrite(fb->buf, 1, fb->len, f);
            fclose(f);
            
            if (written == fb->len) {
                ESP_LOGI(TAG, "Saved: %s (%d bytes)", filename, written);
            } else {
                ESP_LOGE(TAG, "Write failed: %d/%d bytes", written, fb->len);
            }
        }
        
        // Return buffer
        esp_camera_fb_return(fb);
        
        // Capture every 2 seconds
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "ESP32-CAM SD Card Logger");
    
    // Initialize SD card FIRST
    if (sd_card_init() != ESP_OK) {
        ESP_LOGE(TAG, "SD card init failed - halting");
        return;
    }
    
    ESP_LOGI(TAG, "Initializing camera...");
    
    // Initialize camera
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera init failed: 0x%x", err);
        return;
    }
    
    ESP_LOGI(TAG, "Camera initialized successfully");
    
    // Create capture task
    xTaskCreate(camera_capture_task, "cam_task", 4096, NULL, 5, NULL);
    
    ESP_LOGI(TAG, "System ready - logging images to SD card");
}
