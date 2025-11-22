/*
 * ESP32-CAM Control Implementation
 * ESP32-S3 WROOM Master - Boot mode and reset control for ESP32-CAM
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp32cam_control.h"

static const char *TAG = "CAM_CONTROL";

void esp32cam_control_init(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << CAM_IO0_PIN) | (1ULL << CAM_RESET_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    
    // Initialize to normal operating state
    gpio_set_level(CAM_IO0_PIN, 1);     // IO0 HIGH = normal boot
    gpio_set_level(CAM_RESET_PIN, 1);   // EN HIGH = running
    
    ESP_LOGI(TAG, "Control pins initialized (IO0=%d, RESET=%d)", 
             CAM_IO0_PIN, CAM_RESET_PIN);
}

void esp32cam_enter_programming_mode(void) {
    ESP_LOGI(TAG, "Entering programming mode...");
    
    // Step 1: Pull IO0 LOW (bootloader mode)
    gpio_set_level(CAM_IO0_PIN, 0);
    ESP_LOGI(TAG, "IO0 set LOW (bootloader mode)");
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Step 2: Reset the ESP32-CAM
    gpio_set_level(CAM_RESET_PIN, 0);
    ESP_LOGI(TAG, "Reset asserted...");
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(CAM_RESET_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(500));
    
    ESP_LOGI(TAG, "ESP32-CAM is in PROGRAMMING MODE");
    ESP_LOGI(TAG, "Ready to receive firmware upload");
}

void esp32cam_reset_to_normal_mode(void) {
    ESP_LOGI(TAG, "Resetting to normal mode...");
    
    // Step 1: Set IO0 HIGH (normal boot)
    gpio_set_level(CAM_IO0_PIN, 1);
    ESP_LOGI(TAG, "IO0 set HIGH (normal boot)");
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Step 2: Reset the ESP32-CAM
    gpio_set_level(CAM_RESET_PIN, 0);
    ESP_LOGI(TAG, "Reset asserted...");
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(CAM_RESET_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(500));
    
    ESP_LOGI(TAG, "ESP32-CAM is in NORMAL MODE");
}

void esp32cam_hard_reset(void) {
    ESP_LOGI(TAG, "Performing hard reset...");
    
    gpio_set_level(CAM_RESET_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(CAM_RESET_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(500));
    
    ESP_LOGI(TAG, "Reset complete");
}

void esp32cam_set_boot_mode(bool bootloader_mode) {
    if (bootloader_mode) {
        gpio_set_level(CAM_IO0_PIN, 0);
        ESP_LOGI(TAG, "Boot mode: BOOTLOADER (IO0 LOW)");
    } else {
        gpio_set_level(CAM_IO0_PIN, 1);
        ESP_LOGI(TAG, "Boot mode: NORMAL (IO0 HIGH)");
    }
}