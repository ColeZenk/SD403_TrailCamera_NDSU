#ifndef UTILS_H
#define UTILS_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "esp_err.h"
#include "config.h"

/*******************************************************************************
 * Checksum Utilities - Inlined for Performance
 ******************************************************************************/

/**
 * Calculate simple XOR checksum
 * Inlined for hot paths (image verification)
 */
ALWAYS_INLINE uint32_t checksum_calculate_xor(const uint8_t *data, size_t size)
{
    if (UNLIKELY(!data || size == 0)) {
        return 0;
    }

    uint32_t checksum = 0;
    for (size_t i = 0; i < size; i++) {
        checksum ^= data[i];
    }
    return checksum;
}

/**
 * Verify XOR checksum
 * Inlined for hot paths
 */
ALWAYS_INLINE bool checksum_verify_xor_fast(const uint8_t *data, size_t size, 
                                             uint32_t expected)
{
    return (checksum_calculate_xor(data, size) == expected);
}

/**
 * Verify checksum with logging (not inlined - for debugging)
 */
bool checksum_verify_xor(const uint8_t *data, size_t size, uint32_t expected);

/*******************************************************************************
 * Memory Utilities
 ******************************************************************************/

/**
 * Allocate DMA-capable memory with error checking
 */
void* dma_malloc(size_t size);

/**
 * Safe free (handles NULL)
 */
ALWAYS_INLINE void safe_free(void **ptr)
{
    if (ptr && *ptr) {
        free(*ptr);
        *ptr = NULL;
    }
}

/*******************************************************************************
 * Validation Utilities - Inlined for Performance
 ******************************************************************************/

/**
 * Validate image header (fast inline version)
 */
ALWAYS_INLINE bool validate_image_header_fast(const image_header_t *header)
{
    if (UNLIKELY(!header)) {
        return false;
    }

    // Fast path - common checks
    if (header->magic != IMAGE_HEADER_MAGIC) {
        return false;
    }

    if (header->size == 0 || header->size > IMAGE_MAX_SIZE_BYTES) {
        return false;
    }

    return true;
}

/**
 * Validate image header (with logging - not inlined)
 */
bool validate_image_header(const image_header_t *header);

/**
 * Check if size is within acceptable bounds
 */
ALWAYS_INLINE bool validate_size_fast(size_t size, size_t max_size)
{
    return (size > 0 && size <= max_size);
}

/**
 * Validate size with logging
 */
bool validate_size(size_t size, size_t max_size);

/*******************************************************************************
 * Fast Math Utilities
 ******************************************************************************/

/**
 * Fast min/max (inlined)
 */
ALWAYS_INLINE size_t min_size(size_t a, size_t b)
{
    return (a < b) ? a : b;
}

ALWAYS_INLINE size_t max_size(size_t a, size_t b)
{
    return (a > b) ? a : b;
}

/**
 * Align size to 4-byte boundary (for DMA)
 */
ALWAYS_INLINE size_t align_4(size_t size)
{
    return (size + 3) & ~3;
}

#endif // UTILS_H
