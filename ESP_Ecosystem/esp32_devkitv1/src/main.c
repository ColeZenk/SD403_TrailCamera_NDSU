/**
 * @file main.c
 * @brief ESP32 DevKitV1 - Main Entry Point
 *
 * Pipeline modules (cam_spi, fpga_spi, lora, image_processor)
 * commented out until their sources land on this branch.
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"

#include "config.h"
// #include "isr_signals.h"
// #include "cam_spi.h"
// #include "fpga_spi.h"
// #include "lora_uart.h"
// #include "image_processor.h"
#include "peripherals/sensors_temp_humidity.h"

#define STACK_SIZE_MEDIUM 4096
#define TASK_PRIORITY_LOW (tskIDLE_PRIORITY + 1)
#define SECONDS_TO_TICKS(s) pdMS_TO_TICKS((s) * 1000)

static const char *TAG = "MAIN";

static esp_err_t initializeSystem(void)
{
    ESP_LOGI(TAG, "=== ESP32 DevKitV1 - Image Pipeline v2.0 ===");
    ESP_LOGI(TAG, "ESP-IDF Version: %s", esp_get_idf_version());
    ESP_LOGI(TAG, "Free heap: %lu bytes", esp_get_free_heap_size());

    ESP_LOGI(TAG, "Initializing subsystems...");

    // TODO: re-enable once pipeline sources are on this branch
    // isr_init();
    // cam_spi_init();
    // fpga_spi_init();
    // lora_init();
    // image_processor_init();

    esp_err_t ret = sensors_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Sensor initialization failed: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "All subsystems initialized");
    return ESP_OK;
}

static void createTasks(void)
{
    ESP_LOGI(TAG, "Creating tasks...");

    // TODO: re-enable pipeline tasks
    // xTaskCreate(cam_spi_receive_task, "cam_spi_rx", STACK_SIZE_MEDIUM, NULL, TASK_PRIORITY_HIGH, NULL);
    // xTaskCreate(image_processor_task, "img_processor", STACK_SIZE_LARGE, NULL, TASK_PRIORITY_MEDIUM, NULL);
    // xTaskCreate(lora_receive_task, "lora_rx", STACK_SIZE_MEDIUM, NULL, TASK_PRIORITY_MEDIUM, NULL);

    BaseType_t ret = xTaskCreate(
        sensors_task,
        "sensors",
        STACK_SIZE_MEDIUM,
        NULL,
        TASK_PRIORITY_LOW,
        NULL
    );
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create sensor task");
        return;
    }

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
