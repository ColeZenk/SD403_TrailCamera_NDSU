#ifndef IMAGE_BUFFER_POOL_H
#define IMAGE_BUFFER_POOL_H

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"

#define IMAGE_BUFFER_COUNT 3
#define IMAGE_BUFFER_SIZE 76800  // 320x240 grayscale

/**
 * Initialize image buffer pool in PSRAM
 */
esp_err_t image_buffer_pool_init(void);

/**
 * Allocate a buffer from the pool
 */
uint8_t* image_buffer_alloc(void);

/**
 * Return a buffer to the pool
 */
void image_buffer_free(uint8_t* buffer);

/**
 * Get statistics about buffer pool usage
 */
void image_buffer_get_stats(int *total, int *in_use, int *peak);

/**
 * Run unit test on buffer pool
 */
void image_buffer_pool_unit_test(void);

#endif // IMAGE_BUFFER_POOL_H
