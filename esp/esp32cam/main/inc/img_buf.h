#ifndef IMAGE_BUFFER_POOL_H
#define IMAGE_BUFFER_POOL_H

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#include <stdbool.h>

#define IMAGE_BUFFER_COUNT 3
#define IMAGE_BUFFER_SIZE 76800 // 320x240 grayscale

/* Save ring buffer — 1-in-N frames buffered in PSRAM for post-capture JPEG */
#define SAVE_RING_SIZE     30
#define SAVE_INTERVAL      10

/**
 * Initialize img buffer pool in PSRAM
 */
esp_err_t img_buf_init(void);

/**
 * Allocate a buffer from the pool
 */
uint8_t *img_buffer_alloc(void);

/**
 * Return a buffer to the pool
 */
void img_buffer_free(uint8_t *buffer);

/**
 * Get statistics about buffer pool usage
 */
void img_buffer_get_stats(int *total, int *in_use, int *peak);

/**
 * Initialize save ring buffer in PSRAM.
 * Call once at boot, after img_buf_init().
 */
esp_err_t save_ring_init(void);

/**
 * Copy a frame into the next save ring slot.
 * Overwrites oldest if full (circular).
 * timestamp: esp_timer_get_time() value at capture.
 */
void save_ring_push(const uint8_t *frame, size_t len, int64_t timestamp);

/**
 * Get pointer and timestamp for a saved frame by index.
 * idx 0 = oldest, idx count-1 = newest.
 * Returns NULL if idx out of range.
 */
const uint8_t *save_ring_get(int idx, int64_t *timestamp_out);

/**
 * Number of frames currently in the ring.
 */
int save_ring_count(void);

/**
 * Reset ring to empty (does not free PSRAM slots).
 */
void save_ring_reset(void);

/**
 * Run unit test on buffer pool
 */
void img_buf_unit_test(void);

#endif // IMAGE_BUFFER_POOL_H
