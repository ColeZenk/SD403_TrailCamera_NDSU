/**
 * main.c — ESP32-S3 Trail Camera Receiver
 *
 * Core 0: LoRa receive → Gaussian reconstruction
 * Core 1: WiFi AP + WebSocket server → Flutter app
 *
 * Phone connects to "TrailCamera" AP, opens ws://192.168.4.1:80/stream,
 * receives reconstructed grayscale frames as binary WebSocket messages.
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_psram.h"

#include "config.h"
#include "image_reconstruct.h"
#include "lora.h"
#include "wifi_ap.h"
#include "ws_server.h"

static const char *TAG = "MAIN";

static esp_err_t init_system(void)
{
        ESP_LOGI(TAG, "=== Trail Camera IV — ESP32 S3 Wroom ===");
        ESP_LOGI(TAG, "IDF %s | heap %lu bytes", esp_get_idf_version(),
                 esp_get_free_heap_size());

        esp_err_t ret;

        ret = wifi_ap_init();
        if (ret != ESP_OK) return ret;
        ret = ws_server_init();
        if (ret != ESP_OK) return ret;
        ret = alloc_reference_frames();
        if (ret != ESP_OK) return ret;
        ret = lora_init();
        if (ret != ESP_OK) return ret;

        return ESP_OK;
}

static void create_tasks(void)
{
        xTaskCreate(lora_receive_task, "lora_rx", STACK_SIZE_MEDIUM, NULL,
                    TASK_PRIORITY_MEDIUM, NULL);
#ifdef TEST_MODE_WS_PATTERNS
        xTaskCreate(ws_test_task, "ws_test", STACK_SIZE_LARGE, NULL,
                    TASK_PRIORITY_LOW, NULL);
#endif
}

static void monitor_heap(void)
{
        uint32_t prev = esp_get_free_heap_size();

        for (;;) {
                vTaskDelay(SECONDS_TO_TICKS(10));
                uint32_t now = esp_get_free_heap_size();
                ESP_LOGI(TAG, "heap: %lu (%+ld)", now, (int32_t)(now - prev));
                prev = now;
        }
}

void app_main(void)
{
#ifdef TEST_MODE_LORA_BENCH
        /* Bench mode: only LoRa bench task runs — no camera/FPGA/sensor init
         */
        xTaskCreate(lora_bench_task, "lora_bench", STACK_SIZE_LARGE, NULL,
                    TASK_PRIORITY_MEDIUM, NULL);
        monitor_heap();
#else
        if (init_system() != ESP_OK) {
                ESP_LOGE(TAG, "init failed — halted");
                return;
        }
        create_tasks();
        monitor_heap();
#endif
}
