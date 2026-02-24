/**
 * @file main.c
 * @brief ESP32 DevKitV1 - Main Entry Point
 *
 * Image pipeline (cam_spi, fpga_spi, lora, image_processor)
 * + sensor subsystem (AHT20, PIR, stepper motor)
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"

#include "config.h"
#include "isr_signals.h"
#include "cam_spi.h"
#include "fpga_spi.h"
#include "lora_uart.h"
#include "image_processor.h"
#include "peripherals/sensors_temp_humidity.h"

static const char *TAG = "MAIN";

static esp_err_t initializeSystem(void)
{
    esp_err_t ret;

    ESP_LOGI(TAG, "=== ESP32 DevKitV1 - Trail Camera IV ===");
    ESP_LOGI(TAG, "ESP-IDF Version: %s", esp_get_idf_version());
    ESP_LOGI(TAG, "Free heap: %lu bytes", esp_get_free_heap_size());

    // ISR module first — semaphores must exist before tasks
    ret = isr_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ISR init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = cam_spi_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Camera SPI init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = fpga_spi_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "FPGA SPI init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = lora_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "LoRa init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = image_processor_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Image processor init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = sensors_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Sensor init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "All subsystems initialized");
    return ESP_OK;
}

static void createTasks(void)
{
    ESP_LOGI(TAG, "Creating tasks...");

    xTaskCreate(cam_spi_receive_task, "cam_spi_rx", STACK_SIZE_MEDIUM, NULL, TASK_PRIORITY_HIGH, NULL);
    xTaskCreate(image_processor_task, "img_processor", STACK_SIZE_LARGE, NULL, TASK_PRIORITY_MEDIUM, NULL);
    xTaskCreate(lora_receive_task, "lora_rx", STACK_SIZE_MEDIUM, NULL, TASK_PRIORITY_MEDIUM, NULL);
    xTaskCreate(sensors_task, "sensors", STACK_SIZE_MEDIUM, NULL, TASK_PRIORITY_LOW, NULL);

#ifdef TEST_MODE_FPGA_PATTERNS
    xTaskCreate(fpga_test_task, "fpga_test", STACK_SIZE_MEDIUM, NULL, TASK_PRIORITY_MEDIUM, NULL);
    ESP_LOGI(TAG, "FPGA test task created — press BOOT to send patterns");
#endif

    ESP_LOGI(TAG, "All tasks created");
}

static void monitorSystem(void)
{
    uint32_t last_heap = esp_get_free_heap_size();

    while (1) {
        vTaskDelay(SECONDS_TO_TICKS(10));

        uint32_t current_heap = esp_get_free_heap_size();
        int32_t heap_change = (int32_t)(current_heap - last_heap);

        ESP_LOGI(TAG, "Free heap: %lu bytes (%+ld)", current_heap, heap_change);
        last_heap = current_heap;
    }
}

void app_main(void)
{
    esp_err_t ret = initializeSystem();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "System initialization failed — halted");
        return;
    }

    createTasks();
    monitorSystem();
}
