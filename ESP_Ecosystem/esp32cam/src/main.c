/*
 * ESP32-CAM Main Application
 *
 * Production mode:
 *   Camera capture → SPI DMA to DevKit (every frame)
 *                  → JPEG to SD card (every 3 seconds)
 *
 * Test burst mode (define TEST_BURST_CAPTURE):
 *   Camera capture → raw frames to SD at max rate
 *   No SPI, no buffer pool — SD stays mounted
 */

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_log.h"

#include "camera_control.h"
#include "image_buffer_pool.h"
#include "image_data.h"
#include "DMA_SPI_master.h"
#include "sd_storage.h"

static const char *TAG = "MAIN";

#define IMAGE_QUEUE_DEPTH  2

void app_main(void)
{
    ESP_LOGI(TAG, "ESP32-CAM Starting...");

#ifdef TEST_BURST_CAPTURE

    ESP_LOGI(TAG, "*** TEST BURST CAPTURE MODE ***");

    /* Mount SD once — stays mounted for entire burst */
    if (sd_card_init() != ESP_OK) {
        ESP_LOGE(TAG, "SD card init failed — halting");
        return;
    }

    /* Camera with locked exposure/gain */
    if (camera_init() != ESP_OK) {
        ESP_LOGE(TAG, "Camera init failed — halting");
        return;
    }

    /* Fire — burst_capture_task handles everything */
    camera_start_capture();

    ESP_LOGI(TAG, "Burst capture running");

#else

    /* PSRAM buffer pool for DMA transfers */
    if (image_buffer_pool_init() != ESP_OK) {
        ESP_LOGE(TAG, "Buffer pool init failed — halting");
        return;
    }

    /* SPI DMA master — pins 13/14 (shared with SD, time-shared in capture loop) */
    if (spi_dma_init() != ESP_OK) {
        ESP_LOGE(TAG, "SPI DMA init failed — halting");
        return;
    }

    /* Camera */
    if (camera_init() != ESP_OK) {
        ESP_LOGE(TAG, "Camera init failed — halting");
        return;
    }

    /* Image queue: capture → SPI transmit */
    QueueHandle_t image_queue = xQueueCreate(IMAGE_QUEUE_DEPTH, sizeof(image_data_t));
    if (image_queue == NULL) {
        ESP_LOGE(TAG, "Queue creation failed — halting");
        return;
    }

    camera_set_tx_queue(image_queue);

    /* SPI transmit task — dequeues frames, sends via DMA */
    xTaskCreate(spi_transmit_task, "spi_tx", 4096, (void *)image_queue, 5, NULL);

    /* Capture loop — max fps to SPI, JPEG to SD every 3s */
    camera_start_capture();

    ESP_LOGI(TAG, "Pipeline running");

#endif
}
