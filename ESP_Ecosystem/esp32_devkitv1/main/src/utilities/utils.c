/**
 * utils.c — Checksums, DMA memory, validation
 *
 * Non-inlined versions with logging. Fast/inline versions in utils.h.
 */

#include "utils.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include <string.h>
#include <inttypes.h>

static const char *TAG = "UTILS";

/*******************************************************************************
 * Checksum
 ******************************************************************************/

bool checksum_verify_xor(const uint8_t *data, size_t size, uint32_t expected)
{
    uint32_t calc = checksum_calculate_xor(data, size);

    if (calc != expected) {
        ESP_LOGW(TAG, "checksum: expected 0x%08" PRIX32 " got 0x%08" PRIX32,
                 expected, calc);
        return false;
    }

    return true;
}

/*******************************************************************************
 * Memory
 ******************************************************************************/

void *dma_malloc(size_t size)
{
    if (size == 0) return NULL;

    size = align_4(size);
    void *ptr = heap_caps_malloc(size, MALLOC_CAP_DMA);

    if (!ptr) {
        ESP_LOGE(TAG, "DMA alloc failed: %zu bytes", size);
    }

    return ptr;
}

/*******************************************************************************
 * Validation
 ******************************************************************************/

bool validate_image_header(const image_header_t *header)
{
    if (!header) return false;

    if (header->magic != IMAGE_HEADER_MAGIC) {
        ESP_LOGW(TAG, "bad magic: 0x%08" PRIX32, (uint32_t)header->magic);
        return false;
    }

    if (!validate_size_fast(header->size, IMAGE_MAX_SIZE_BYTES)) {
        ESP_LOGW(TAG, "bad size: %" PRIu32, (uint32_t)header->size);
        return false;
    }

    return true;
}

bool validate_size(size_t size, size_t max_size)
{
    if (size == 0 || size > max_size) {
        ESP_LOGW(TAG, "size %zu exceeds max %zu", size, max_size);
        return false;
    }
    return true;
}
