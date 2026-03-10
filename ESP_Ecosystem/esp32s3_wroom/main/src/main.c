/**
 * main.c — ESP32-S3 Trail Camera Receiver
 *
 * Core 0: LoRa receive → Gaussian reconstruction
 * Core 1: WiFi AP + WebSocket server → Flutter app
 *
 * Phone connects to "TrailCamera" AP, opens ws://192.168.4.1:80/stream,
 * receives reconstructed grayscale frames as binary WebSocket messages.
 */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_psram.h"
#include "esp_heap_caps.h"

#include "config.h"
#include "lora.h"

#ifndef TEST_MODE_LORA_BENCH
#include "wifi_ap.h"
#include "ws_server.h"
#endif

static const char *TAG = "MAIN";

/*******************************************************************************
 * Reference frame storage (PSRAM)
 * 3 positions × 307,200 bytes = ~921 KB
 ******************************************************************************/

#ifndef TEST_MODE_LORA_BENCH
static uint8_t *ref_frames[NUM_POSITIONS];
uint32_t frame_seq = 0;

static esp_err_t alloc_reference_frames(void)
{
    for (int i = 0; i < NUM_POSITIONS; i++) {
        ref_frames[i] = heap_caps_calloc(FRAME_BYTES, 1, MALLOC_CAP_SPIRAM);
        if (!ref_frames[i]) {
            ESP_LOGE(TAG, "PSRAM alloc failed for position %d", i);
            return ESP_ERR_NO_MEM;
        }
    }
    ESP_LOGI(TAG, "reference frames allocated: %d × %d KB in PSRAM",
             NUM_POSITIONS, FRAME_BYTES / 1024);
    return ESP_OK;
}

/*******************************************************************************
 * Gaussian Reconstruction
 *
 * Parses a raw SDD502 LoRa packet, applies Gaussian-interpolated diff
 * to the stored reference frame, returns the reconstructed frame buffer.
 *
 * Caller must NOT free the returned pointer — it points into ref_frames[].
 ******************************************************************************/

typedef struct {
    int    bx, by;   /* block coordinates in frame space */
    float  dc;       /* dequantized per-pixel average delta */
} active_block_t;

uint8_t *reconstruct(const uint8_t *packet, size_t pkt_len,
                     uint8_t *position_out)
{
    if (pkt_len < 7) return NULL;

    /* --- Parse header --- */
    const uint8_t flags       = packet[0];
    const uint8_t pos_idx     = (flags >> 2) & 0x03;
    const uint16_t scale      = ((uint16_t)packet[1] << 8) | packet[2];
    const int bx1 = packet[3], by1 = packet[4];
    const int bx2 = packet[5], by2 = packet[6];

    if (pos_idx >= NUM_POSITIONS) return NULL;
    *position_out = pos_idx;

    const int bbox_w      = bx2 - bx1 + 1;
    const int bbox_h      = by2 - by1 + 1;
    const int bbox_blocks = bbox_w * bbox_h;
    const int bitmap_bytes = (bbox_blocks + 7) >> 3;

    if ((size_t)(7 + bitmap_bytes) > pkt_len) return NULL;

    /* --- Decode active block bitmap --- */
    int active_count = 0;
    const uint8_t *bitmap = packet + 7;
    for (int i = 0; i < bbox_blocks; i++)
        if ((bitmap[i >> 3] >> (7 - (i & 7))) & 1)
            active_count++;

    const int dc_bytes = (active_count + 1) >> 1;
    if ((size_t)(7 + bitmap_bytes + dc_bytes) > pkt_len) return NULL;

    /* Allocate active block list on stack (max 4800 blocks) */
    active_block_t *blocks = malloc(active_count * sizeof(active_block_t));
    if (!blocks) return NULL;

    const uint8_t *dc_data = bitmap + bitmap_bytes;
    int block_idx = 0, nibble_idx = 0;

    for (int row = 0; row < bbox_h; row++) {
        for (int col = 0; col < bbox_w; col++) {
            const int bit = row * bbox_w + col;
            if (!((bitmap[bit >> 3] >> (7 - (bit & 7))) & 1)) continue;

            const uint8_t byte   = dc_data[nibble_idx >> 1];
            const uint8_t nibble = (nibble_idx & 1) ? (byte & 0x0F)
                                                     : (byte >> 4);
            blocks[block_idx].bx = bx1 + col;
            blocks[block_idx].by = by1 + row;
            blocks[block_idx].dc = nibble * scale / 15.0f;
            block_idx++;
            nibble_idx++;
        }
    }

    /* --- Gaussian interpolation over bounding box pixels --- */
    uint8_t *ref = ref_frames[pos_idx];
    const float sigma2_inv = 1.0f / (2.0f * GAUSS_SIGMA * GAUSS_SIGMA);
    const float radius_px  = GAUSS_RADIUS_BLOCKS * BLOCK_SIZE;

    const int px1 = bx1 * BLOCK_SIZE;
    const int py1 = by1 * BLOCK_SIZE;
    const int px2 = (bx2 + 1) * BLOCK_SIZE;
    const int py2 = (by2 + 1) * BLOCK_SIZE;

    for (int py = py1; py < py2 && py < FRAME_H; py++) {
        for (int px = px1; px < px2 && px < FRAME_W; px++) {
            float wsum = 0.0f, wtotal = 0.0f;

            for (int k = 0; k < block_idx; k++) {
                const float cx = blocks[k].bx * BLOCK_SIZE + BLOCK_SIZE / 2.0f;
                const float cy = blocks[k].by * BLOCK_SIZE + BLOCK_SIZE / 2.0f;
                const float dx = px - cx;
                const float dy = py - cy;

                /* Skip blocks outside Gaussian radius */
                if (fabsf(dx) > radius_px || fabsf(dy) > radius_px) continue;

                const float w = expf(-(dx*dx + dy*dy) * sigma2_inv);
                wsum   += w * blocks[k].dc;
                wtotal += w;
            }

            if (wtotal > 0.0f) {
                const int idx  = py * FRAME_W + px;
                const int val  = (int)ref[idx] + (int)(wsum / wtotal);
                ref[idx] = (uint8_t)(val < 0 ? 0 : val > 255 ? 255 : val);
            }
        }
    }

    free(blocks);
    return ref;
}
#endif /* TEST_MODE_LORA_BENCH */


/*******************************************************************************
 * app_main
 ******************************************************************************/

void app_main(void)
{
    ESP_LOGI(TAG, "=== Trail Camera Receiver ===");
    ESP_LOGI(TAG, "IDF %s | heap %"PRIu32" bytes | PSRAM %u bytes",
             esp_get_idf_version(),
             esp_get_free_heap_size(),
             (unsigned)esp_psram_get_size());

#ifndef TEST_MODE_LORA_BENCH
    ESP_ERROR_CHECK(alloc_reference_frames());
    ESP_ERROR_CHECK(wifi_ap_init());
    ESP_ERROR_CHECK(ws_server_init());
#endif

    lora_start();

#ifdef TEST_MODE_LORA_BENCH
    ESP_LOGI(TAG, "Bench echo slave ready — waiting for packets from DevKitV1");
#else
    ESP_LOGI(TAG, "ready — connect to \"%s\", open ws://192.168.4.1:%d%s",
             WIFI_AP_SSID, WS_PORT, WS_PATH);
#endif
}
