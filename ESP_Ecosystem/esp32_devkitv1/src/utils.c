/**
 * @file utils.c
 * @brief Utility Functions - Checksums, Memory, and Validation
 * @author ESP32 Image Pipeline Team
 *
 * Implements core utility functions for checksum calculation, DMA memory
 * allocation, and data validation with comprehensive error logging.
 *
 * @scope
 * File covers non-inlined implementations of checksum verification,
 * DMA-capable memory allocation, and validation helpers with logging.
 */

/*************************************************************************
 INCLUDES
 *************************************************************************/
#include "utils.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "UTILS";

/*************************************************************************
 CHECKSUM IMPLEMENTATION (Non-Inlined Versions)
 *************************************************************************/

/**
 * @brief Verify XOR checksum with logging
 * 
 * Calculates XOR checksum over data buffer and compares against expected value.
 * Logs warning on mismatch.
 * 
 * @param data Pointer to data buffer
 * @param size Size of data in bytes
 * @param expected Expected checksum value
 * @return true if checksum matches, false otherwise
 */
bool checksum_verify_xor(const uint8_t *data, size_t size, uint32_t expected)
{
    uint32_t calculated = checksum_calculate_xor(data, size);
    bool valid = (calculated == expected);

    if (UNLIKELY(!valid)) {
        ESP_LOGW(TAG, "Checksum mismatch - expected: 0x%08X, got: 0x%08X",
                 expected, calculated);
    }

    return valid;
}

/*************************************************************************
 MEMORY MANAGEMENT
 *************************************************************************/

/**
 * @brief Allocate DMA-capable memory
 * 
 * Allocates memory suitable for DMA operations with 4-byte alignment.
 * Automatically aligns requested size to 4-byte boundary.
 * 
 * @param size Number of bytes to allocate
 * @return Pointer to allocated memory, or NULL on failure
 */
void* dma_malloc(size_t size)
{
    if (UNLIKELY(size == 0)) {
        ESP_LOGW(TAG, "Attempted to allocate 0 bytes");
        return NULL;
    }

    // Align to 4-byte boundary for DMA
    size = align_4(size);

    void *ptr = heap_caps_malloc(size, MALLOC_CAP_DMA);

    if (UNLIKELY(!ptr)) {
        ESP_LOGE(TAG, "DMA allocation failed for %zu bytes", size);
    }

    return ptr;
}

/*************************************************************************
 VALIDATION (Non-Inlined Versions with Logging)
 *************************************************************************/

/**
 * @brief Validate image header structure
 * 
 * Checks header magic number and image size constraints.
 * Logs specific validation failures.
 * 
 * @param header Pointer to image header structure
 * @return true if header is valid, false otherwise
 */
bool validate_image_header(const image_header_t *header)
{
    if (!header) {
        ESP_LOGE(TAG, "NULL header");
        return false;
    }

    if (header->magic != IMAGE_HEADER_MAGIC) {
        ESP_LOGW(TAG, "Invalid magic: 0x%08X (expected 0x%08X)",
                 header->magic, IMAGE_HEADER_MAGIC);
        return false;
    }

    if (!validate_size_fast(header->size, IMAGE_MAX_SIZE_BYTES)) {
        ESP_LOGW(TAG, "Invalid image size: %u bytes (max: %u)",
                 header->size, IMAGE_MAX_SIZE_BYTES);
        return false;
    }

    return true;
}

/**
 * @brief Validate size against maximum with logging
 * 
 * Wraps fast validation with error logging.
 * 
 * @param size Size value to validate
 * @param max_size Maximum allowed size
 * @return true if size is valid, false otherwise
 */
bool validate_size(size_t size, size_t max_size)
{
    bool valid = validate_size_fast(size, max_size);

    if (UNLIKELY(!valid)) {
        ESP_LOGW(TAG, "Size validation failed: %zu (max: %zu)", size, max_size);
    }

    return valid;
}
