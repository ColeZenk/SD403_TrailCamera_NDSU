/**
 * image_processor.c -- Motion event pipeline
 *
 * Flow per event:
 *   1. Block on g_motion_sem (given by sensors_task on PIR detect)
 *   2. Receive keyframe from CAM (first frame of event, format byte = kf index)
 *   3. Load keyframe to FPGA prev_base via SPI -- no receive (output discarded)
 *   4. Send start metadata over LoRa
 *   5. Diff loop: receive frame -> FPGA -> compressed coefficients -> LoRa
 *      Exit when no frame arrives for 5 seconds (CAM disabled by sensors_task)
 *   6. Send end metadata over LoRa
 */

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "cam_spi.h"
#include "fpga_spi.h"
#include "image_processor.h"
#include "isr_signals.h"
#include "lora.h"
#include "peripherals/sensors_temp_humidity.h"
#include "utils.h"

static const char *TAG = "IMG_PROC";

#define FPGA_RX_BUF_SIZE   512
#define MOTION_TIMEOUT_MS  5000
#define FPGA_PROC_DELAY_MS 50

typedef struct {
        uint8_t  magic[2];       /* 0xAA 0x01 */
        uint8_t  kf_idx;         /* keyframe index 0-2 */
        uint8_t  stepper_phase;
        uint32_t timestamp_s;    /* seconds since boot */
        int16_t  temp_x10;       /* tenths of Celsius */
        uint16_t humidity_x10;   /* tenths of %RH */
} __attribute__((packed)) event_start_t;

typedef struct {
        uint8_t  magic[2];       /* 0xAA 0x02 */
        uint32_t timestamp_s;
} __attribute__((packed)) event_end_t;

static size_t compressed_len(const uint8_t *buf, size_t buf_size)
{
        size_t pos = 0;
        while (pos + 2 <= buf_size) {
                uint8_t hi = buf[pos];
                uint8_t lo = buf[pos + 1];
                if (hi == 0 && lo == 0) break;
                pos += 2 + (lo & 0x07) * 3;
        }
        return pos;
}

static uint32_t now_s(void)
{
        return (uint32_t)(esp_timer_get_time() / 1000000ULL);
}

esp_err_t image_processor_init(void)
{
        ESP_LOGI(TAG, "initialized");
        return ESP_OK;
}

void image_processor_task(void *pvParameters)
{
        QueueHandle_t queue = cam_spi_get_queue();
        image_data_t img;

        uint8_t *rx_buf = dma_malloc(FPGA_RX_BUF_SIZE);
        if (!rx_buf) {
                ESP_LOGE(TAG, "DMA alloc failed");
                vTaskDelete(NULL);
                return;
        }

        ESP_LOGI(TAG, "task started");

        for (;;) {
                /* Wait for PIR motion trigger */
                xSemaphoreTake(g_motion_sem, portMAX_DELAY);
                ESP_LOGI(TAG, "motion event start");

                /* --- Keyframe load ---
                 * First frame from CAM is the keyframe (read from CAM SD).
                 * CAM sets image_header.format to the keyframe index (0-2).
                 * Transmit to FPGA -- this becomes prev_base after the swap.
                 * Do NOT read back coefficients; diff against uninit prev is
                 * irrelevant. */
                if (xQueueReceive(queue, &img,
                                  pdMS_TO_TICKS(10000)) != pdTRUE) {
                        ESP_LOGW(TAG, "keyframe timeout -- aborting event");
                        continue;
                }

                uint8_t kf_idx = img.header.format;
                esp_err_t ret  = fpga_spi_transmit(img.buffer, img.size);
                safe_free((void **)&img.buffer);

                if (ret != ESP_OK) {
                        ESP_LOGE(TAG, "keyframe FPGA tx failed: %s",
                                 esp_err_to_name(ret));
                        continue;
                }

                /* --- Start metadata packet --- */
                float temp_c = 0.0f, hum = 0.0f;
                sensors_get_last_readings(&temp_c, &hum);

                event_start_t start = {
                    .magic         = {0xAA, 0x01},
                    .kf_idx        = kf_idx,
                    .stepper_phase = (uint8_t)sensors_get_stepper_phase(),
                    .timestamp_s   = now_s(),
                    .temp_x10      = (int16_t)(temp_c * 10.0f),
                    .humidity_x10  = (uint16_t)(hum * 10.0f),
                };

                ret = lora_send_packet((uint8_t *)&start, sizeof(start));
                if (ret != ESP_OK)
                        ESP_LOGW(TAG, "start meta LoRa failed: %s",
                                 esp_err_to_name(ret));

                /* --- Diff loop ---
                 * FPGA ping-pong: first live frame diffs against keyframe
                 * (prev_base), then each subsequent frame diffs against the
                 * previous live frame. Runs until CAM stops sending (5s gap). */
                for (;;) {
                        if (xQueueReceive(queue, &img,
                                          pdMS_TO_TICKS(MOTION_TIMEOUT_MS))
                            != pdTRUE) {
                                ESP_LOGI(TAG, "motion timeout -- event end");
                                break;
                        }

                        ret = fpga_spi_transmit(img.buffer, img.size);
                        safe_free((void **)&img.buffer);
                        if (ret != ESP_OK) {
                                ESP_LOGE(TAG, "FPGA tx failed: %s",
                                         esp_err_to_name(ret));
                                continue;
                        }

                        vTaskDelay(pdMS_TO_TICKS(FPGA_PROC_DELAY_MS));

                        ret = fpga_spi_receive(rx_buf, FPGA_RX_BUF_SIZE);
                        if (ret != ESP_OK) {
                                ESP_LOGE(TAG, "FPGA rx failed: %s",
                                         esp_err_to_name(ret));
                                continue;
                        }

                        size_t pkt_len =
                            compressed_len(rx_buf, FPGA_RX_BUF_SIZE);
                        if (pkt_len == 0) continue;

                        ret = lora_send_packet(rx_buf, pkt_len);
                        if (ret != ESP_OK)
                                ESP_LOGW(TAG, "LoRa send failed (%zu B): %s",
                                         pkt_len, esp_err_to_name(ret));
                }

                /* --- End metadata packet --- */
                event_end_t end = {
                    .magic       = {0xAA, 0x02},
                    .timestamp_s = now_s(),
                };

                ret = lora_send_packet((uint8_t *)&end, sizeof(end));
                if (ret != ESP_OK)
                        ESP_LOGW(TAG, "end meta LoRa failed: %s",
                                 esp_err_to_name(ret));

                ESP_LOGI(TAG, "motion event complete");
        }
}
