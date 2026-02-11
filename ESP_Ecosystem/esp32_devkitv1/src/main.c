/**
 * @file main.c
 * @brief ESP32 DevKitV1 Image Pipeline - Main Entry Point
 * @author ESP32 Image Pipeline Team
 *
 * Implements main application entry point and system initialization.
 * Coordinates camera SPI input, FPGA SPI output, LoRa triggers, and image processing.
 *
 * @scope
 * File covers system initialization, task creation, and main monitoring loop
 * for the complete image capture and transmission pipeline.
 */

/*************************************************************************
 INCLUDES
 *************************************************************************/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"

// Module headers
#include "config.h"
#include "cam_spi.h"
#include "fpga_spi.h"
#include "lora_uart.h"
#include "image_processor.h"

static const char *TAG = "MAIN";

/*************************************************************************
 FILE SCOPED FUNCTIONS
 *************************************************************************/
static esp_err_t initializeSystem(void);
static void createTasks(void);
static void printSystemInfo(void);
static void printOperatingMode(void);
static void monitorSystem(void);

/*************************************************************************
 SYSTEM INITIALIZATION
 *************************************************************************/

/**
 * @brief Print system information banner
 */
static void printSystemInfo(void)
{
    ESP_LOGI(TAG, "=== ESP32 DevKitV1 - Image Pipeline v2.0 ===");
    ESP_LOGI(TAG, "Firmware Version: 2.0.0 (Performance Optimized)");
    ESP_LOGI(TAG, "ESP-IDF Version: %s", esp_get_idf_version());
    ESP_LOGI(TAG, "Free heap: %lu bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "Optimization: %s", OPTIMIZE_FOR_SPEED ? "SPEED" : "SIZE");
}

/**
 * @brief Print current operating mode
 */
static void printOperatingMode(void)
{
#if defined(TEST_MODE_FPGA_PATTERNS)
    ESP_LOGI(TAG, "Mode: FPGA Test Pattern Generation");
#elif defined(TEST_MODE_LORA_LOOPBACK)
    ESP_LOGI(TAG, "Mode: LoRa Loopback Testing");
#elif defined(TEST_MODE_CAMERA_INJECT)
    ESP_LOGI(TAG, "Mode: Camera Injection Testing");
#else
    ESP_LOGI(TAG, "Mode: Production");
#endif
}

/**
 * @brief Initialize all system subsystems
 * 
 * Initializes camera SPI, FPGA SPI, LoRa UART, and image processor
 * in sequence. Halts on any initialization failure.
 * 
 * @return ESP_OK on success, error code on failure
 */
static esp_err_t initializeSystem(void)
{
    esp_err_t ret;

    printSystemInfo();
    printOperatingMode();

    ESP_LOGI(TAG, "Initializing subsystems...");

    // Initialize camera SPI slave (receives from ESP32-CAM)
    ret = cam_spi_init();
    if (UNLIKELY(ret != ESP_OK)) {
        ESP_LOGE(TAG, "Camera SPI initialization failed: %s", 
                 esp_err_to_name(ret));
        return ret;
    }

    // Initialize FPGA SPI master (sends to Tang Nano 9K)
    ret = fpga_spi_init();
    if (UNLIKELY(ret != ESP_OK)) {
        ESP_LOGE(TAG, "FPGA SPI initialization failed: %s", 
                 esp_err_to_name(ret));
        return ret;
    }

    // Initialize LoRa UART
    ret = lora_init();
    if (UNLIKELY(ret != ESP_OK)) {
        ESP_LOGE(TAG, "LoRa initialization failed: %s", 
                 esp_err_to_name(ret));
        return ret;
    }

    // Initialize image processor
    ret = image_processor_init();
    if (UNLIKELY(ret != ESP_OK)) {
        ESP_LOGE(TAG, "Image processor initialization failed: %s", 
                 esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "All subsystems initialized successfully");
    return ESP_OK;
}

/*************************************************************************
 TASK CREATION
 *************************************************************************/

/**
 * @brief Create all system tasks
 * 
 * Creates FreeRTOS tasks for camera reception, image processing,
 * LoRa reception, and optional FPGA testing.
 */
static void createTasks(void)
{
    BaseType_t ret;

    ESP_LOGI(TAG, "Creating tasks...");

    // Camera receive task - high priority, hot path
    ret = xTaskCreate(
        cam_spi_receive_task,
        "cam_spi_rx",
        STACK_SIZE_MEDIUM,
        NULL,
        TASK_PRIORITY_HIGH,
        NULL
    );
    if (UNLIKELY(ret != pdPASS)) {
        ESP_LOGE(TAG, "Failed to create camera receive task");
        return;
    }

    // Image processor task - medium priority
    ret = xTaskCreate(
        image_processor_task,
        "img_processor",
        STACK_SIZE_LARGE,
        NULL,
        TASK_PRIORITY_MEDIUM,
        NULL
    );
    if (UNLIKELY(ret != pdPASS)) {
        ESP_LOGE(TAG, "Failed to create image processor task");
        return;
    }

    // LoRa receive task - medium priority
    ret = xTaskCreate(
        lora_receive_task,
        "lora_rx",
        STACK_SIZE_MEDIUM,
        NULL,
        TASK_PRIORITY_MEDIUM,
        NULL
    );
    if (UNLIKELY(ret != pdPASS)) {
        ESP_LOGE(TAG, "Failed to create LoRa receive task");
        return;
    }

#ifdef TEST_MODE_FPGA_PATTERNS
    // FPGA test task - medium priority (test mode only)
    ret = xTaskCreate(
        fpga_test_task,
        "fpga_test",
        STACK_SIZE_MEDIUM,
        NULL,
        TASK_PRIORITY_MEDIUM,
        NULL
    );
    if (UNLIKELY(ret != pdPASS)) {
        ESP_LOGE(TAG, "Failed to create FPGA test task");
        return;
    }
    ESP_LOGI(TAG, "FPGA test task created - press BOOT button to test");
#endif

    ESP_LOGI(TAG, "All tasks created successfully");
}

/**
 * @brief Print system ready status
 */
static void printReadyStatus(void)
{
    ESP_LOGI(TAG, "=== System Ready ===");
    ESP_LOGI(TAG, "Camera SPI: Listening for images from ESP32-CAM");
    ESP_LOGI(TAG, "FPGA SPI: Ready to transmit to Tang Nano 9K");
    ESP_LOGI(TAG, "LoRa: Listening for triggers");

#ifdef TEST_MODE_FPGA_PATTERNS
    ESP_LOGI(TAG, "Test Mode: Press BOOT button to send test patterns to FPGA");
#endif
}

/*************************************************************************
 MONITORING
 *************************************************************************/

/**
 * @brief Main system monitoring loop
 * 
 * Periodically monitors heap usage and logs system status.
 * Tracks heap changes to detect potential memory leaks.
 */
static void monitorSystem(void)
{
    uint32_t last_heap = esp_get_free_heap_size();

    while (1) {
        vTaskDelay(SECONDS_TO_TICKS(10));

        // Periodic status update
        uint32_t current_heap = esp_get_free_heap_size();
        int32_t heap_change = (int32_t)(current_heap - last_heap);

        ESP_LOGI(TAG, "System running - Free heap: %lu bytes (%+ld)",
                 current_heap, heap_change);

        last_heap = current_heap;
    }
}

/*************************************************************************
 MAIN APPLICATION ENTRY POINT
 *************************************************************************/

/**
 * @brief Application main entry point
 * 
 * Initializes all subsystems, creates tasks, and enters monitoring loop.
 * System halts on initialization failure.
 */
void app_main(void)
{
    // Initialize all subsystems
    esp_err_t ret = initializeSystem();
    if (UNLIKELY(ret != ESP_OK)) {
        ESP_LOGE(TAG, "System initialization failed!");
        ESP_LOGE(TAG, "System halted");
        return;
    }

    // Create all tasks
    createTasks();

    // Print status
    printReadyStatus();

    // Main monitoring loop
    monitorSystem();
}
