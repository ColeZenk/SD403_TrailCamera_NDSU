/*
 * ESP32-CAM Main Application
 *
 * Production mode:
 *   Capture → SPI DMA to DevKit (every frame, 30 fps)
 *           → PSRAM save ring (every 10th frame, raw)
 *   On STOP → JPEG batch encode → timestamped SD write → deep sleep
 *
 * Test burst mode (define TEST_BURST_CAPTURE):
 *   Camera capture → raw frames to SD at max rate
 *   No SPI, no buffer pool — SD stays mounted
 */

#include "dma_spi.h"
#include "cam_ctrl.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "img_buf.h"
#include "img_data.h"
#include "sd_stg.h"

static const char *TAG = "MAIN";

#define IMAGE_QUEUE_DEPTH 2

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

        /* PSRAM buffer pool for SPI DMA transfers */
        if (img_buf_init() != ESP_OK) {
                ESP_LOGE(TAG, "Buffer pool init failed — halting");
                return;
        }

        /* PSRAM save ring for post-capture JPEG batch */
        if (save_ring_init() != ESP_OK) {
                ESP_LOGE(TAG, "Save ring init failed — halting");
                return;
        }

        /* SPI DMA master — GPIO 13 shared with SD, no conflict:
         * SD only accessed in post-capture phase when SPI is idle */
        if (spi_dma_init() != ESP_OK) {
                ESP_LOGE(TAG, "SPI DMA init failed — halting");
                return;
        }

        /* Camera — 320x240 grayscale, locked exposure */
        if (camera_init() != ESP_OK) {
                ESP_LOGE(TAG, "Camera init failed — halting");
                return;
        }

        /* Image queue: capture task → SPI transmit task */
        QueueHandle_t img_queue =
            xQueueCreate(IMAGE_QUEUE_DEPTH, sizeof(img_data_t));
        if (img_queue == NULL) {
                ESP_LOGE(TAG, "Queue creation failed — halting");
                return;
        }

        camera_set_tx_queue(img_queue);

        /* SPI transmit task — dequeues frames, sends via DMA */
        xTaskCreate(spi_transmit_task, "spi_tx", 4096, (void *)img_queue, 5,
                    NULL);

        /* Capture loop — every frame to SPI, 1-in-10 to PSRAM save ring.
         * On STOP command: JPEG encode batch → SD write → deep sleep */
        camera_start_capture();

        ESP_LOGI(TAG, "Pipeline running");

#endif
}
