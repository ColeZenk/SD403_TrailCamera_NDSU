#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "esp_heap_caps.h"
#include "esp_log.h"

#include "config.h"
#include "image_reconstruct.h"

static const char *TAG = "RECON";

static uint8_t *ref_frames[NUM_POSITIONS];

esp_err_t alloc_reference_frames(void)
{
        for (int i = 0; i < NUM_POSITIONS; i++) {
                ref_frames[i] =
                    heap_caps_calloc(FRAME_BYTES, 1, MALLOC_CAP_SPIRAM);
                if (!ref_frames[i]) {
                        ESP_LOGE(TAG, "PSRAM alloc failed for position %d", i);
                        return ESP_ERR_NO_MEM;
                }
        }
        ESP_LOGI(TAG, "reference frames allocated: %d x %d KB in PSRAM",
                 NUM_POSITIONS, FRAME_BYTES / 1024);
        return ESP_OK;
}

/*******************************************************************************
 * Inverse 8-point WHT -- maps to wht8.v butterfly
 * 3 stages, stride 1/2/4, pure add/sub, no multiply.
 ******************************************************************************/

static void iwht8(const float *x, float *y)
{
        float s1[8], s2[8];

        s1[0] = x[0] + x[1]; s1[1] = x[0] - x[1];
        s1[2] = x[2] + x[3]; s1[3] = x[2] - x[3];
        s1[4] = x[4] + x[5]; s1[5] = x[4] - x[5];
        s1[6] = x[6] + x[7]; s1[7] = x[6] - x[7];

        s2[0] = s1[0] + s1[2]; s2[2] = s1[0] - s1[2];
        s2[1] = s1[1] + s1[3]; s2[3] = s1[1] - s1[3];
        s2[4] = s1[4] + s1[6]; s2[6] = s1[4] - s1[6];
        s2[5] = s1[5] + s1[7]; s2[7] = s1[5] - s1[7];

        y[0] = s2[0] + s2[4]; y[4] = s2[0] - s2[4];
        y[1] = s2[1] + s2[5]; y[5] = s2[1] - s2[5];
        y[2] = s2[2] + s2[6]; y[6] = s2[2] - s2[6];
        y[3] = s2[3] + s2[7]; y[7] = s2[3] - s2[7];
}

static void iwht_2d(float C[8][8], float P[8][8])
{
        float tmp[8][8];
        float row_in[8], row_out[8];

        for (int r = 0; r < 8; r++) {
                for (int c = 0; c < 8; c++) row_in[c] = C[r][c];
                iwht8(row_in, row_out);
                for (int c = 0; c < 8; c++) tmp[r][c] = row_out[c];
        }

        for (int c = 0; c < 8; c++) {
                for (int r = 0; r < 8; r++) row_in[r] = tmp[r][c];
                iwht8(row_in, row_out);
                for (int r = 0; r < 8; r++) P[r][c] = row_out[r] / 64.0f;
        }
}

/*******************************************************************************
 * Reconstruct from FPGA compressed packet
 *
 * Wire format (from blk_schd.v):
 *   Per block: [header_hi][header_lo] then per coeff [rc][q_hi][q_lo]
 *     header_hi = {2'b0, by[4:0], bx[5]}
 *     header_lo = {bx[4:0], coeff_n[2:0]}
 *     rc        = {0, 0, r[2:0], c[2:0]}
 *     q         = signed 16-bit (q_hi:q_lo)
 *
 * Reconstruction (matches decode_frame.m):
 *   1. Dequantize: val = q << Q_SHIFT
 *   2. Inverse 2D WHT -> spatial patch
 *   3. Gaussian smooth diff
 *   4. Add to reference frame
 ******************************************************************************/

uint8_t *reconstruct(const uint8_t *packet, size_t pkt_len,
                     uint8_t *position_out)
{
        if (pkt_len < 2) return NULL;

        /* Position is encoded in first block's header if present,
           but the FPGA doesn't send position in the compressed stream.
           For now, hardcode to center (1). */
        *position_out = 1;
        uint8_t *ref = ref_frames[*position_out];

        /* Build diff image from compressed blocks */
        float *diff = heap_caps_calloc(FRAME_BYTES, sizeof(float),
                                       MALLOC_CAP_SPIRAM);
        if (!diff) return NULL;

        size_t pos = 0;
        int n_blocks = 0;

        while (pos + 2 <= pkt_len) {
                uint8_t hdr_hi = packet[pos];
                uint8_t hdr_lo = packet[pos + 1];

                if (hdr_hi == 0 && hdr_lo == 0)
                        break;

                int by = (hdr_hi >> 1) & 0x1F;
                int bx = ((hdr_hi & 0x01) << 5) | ((hdr_lo >> 3) & 0x1F);
                int coeff_n = hdr_lo & 0x07;
                pos += 2;

                if (pos + coeff_n * 3 > pkt_len)
                        break;

                float C[8][8];
                memset(C, 0, sizeof(C));

                for (int k = 0; k < coeff_n; k++) {
                        uint8_t rc = packet[pos];
                        uint8_t q_hi = packet[pos + 1];
                        uint8_t q_lo = packet[pos + 2];
                        pos += 3;

                        int r = (rc >> 3) & 0x07;
                        int c = rc & 0x07;
                        int16_t q = (int16_t)((q_hi << 8) | q_lo);

                        C[r][c] = (float)((int32_t)q << Q_SHIFT);
                }

                float P[8][8];
                iwht_2d(C, P);

                int py0 = by * 8;
                int px0 = bx * 8;
                for (int r = 0; r < 8; r++) {
                        for (int c = 0; c < 8; c++) {
                                int y = py0 + r;
                                int x = px0 + c;
                                if (y < FRAME_H && x < FRAME_W)
                                        diff[y * FRAME_W + x] = P[r][c];
                        }
                }
                n_blocks++;
        }

        ESP_LOGI(TAG, "%d blocks, %zu bytes", n_blocks, pos);

        /* TODO: LPF smoothing pass here */

        /* Add diff to reference frame */
        for (int i = 0; i < FRAME_BYTES; i++) {
                int val = (int)ref[i] + (int)roundf(diff[i]);
                ref[i] = (uint8_t)(val < 0 ? 0 : val > 255 ? 255 : val);
        }

        free(diff);
        return ref;
}
