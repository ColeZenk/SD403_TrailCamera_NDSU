/*
 * img_buf.c
 * PSRAM memory pool for img buffers
 * Thread-safe allocation/deallocation with usage tracking
 */

#include "img_buf.h"

#include <string.h>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static const char *TAG = "IMG_POOL";

/* Debug flag - set to 1 to enable verbose logging */
#define UNIT_TEST_BUFF_POOL 0

#if UNIT_TEST_BUFF_POOL
#define LOG_DEBUG(fmt, ...) ESP_LOGI(TAG, fmt, ##__VA_ARGS__)
#else
#define LOG_DEBUG(fmt, ...)                                                   \
        do {                                                                  \
        } while (0)
#endif

/**************************************************************************************************
  File Scope Variables
**************************************************************************************************/

/* Buffer pool structure */
static struct {
        uint8_t *buffers[IMAGE_BUFFER_COUNT];
        bool in_use[IMAGE_BUFFER_COUNT];
        SemaphoreHandle_t mutex;
        int peak_usage;
        bool initialized;
} pool = {0};

/**************************************************************************************************
  Global Interface
**************************************************************************************************/

esp_err_t img_buf_init(void)
{
        /* Redundancy check */
        if (pool.initialized) {
                ESP_LOGW(TAG, "Buffer pool already initialized");
                return ESP_OK;
        }

        LOG_DEBUG("Initializing buffer pool in PSRAM...");

        /* Create mutex for thread-safe access */
        pool.mutex = xSemaphoreCreateMutex();
        if (pool.mutex == NULL) {
                ESP_LOGE(TAG, "Failed to create mutex");
                return ESP_FAIL;
        }

        /* Allocate buffers in PSRAM (external RAM) */
        for (int i = 0; i < IMAGE_BUFFER_COUNT; i++) {
                pool.buffers[i] =
                    heap_caps_malloc(IMAGE_BUFFER_SIZE, MALLOC_CAP_SPIRAM);

                if (pool.buffers[i] == NULL) {
                        ESP_LOGE(TAG, "Failed to allocate buffer %d in PSRAM",
                                 i);

                        /* Clean up any allocated buffers */
                        for (int j = 0; j < i; j++) {
                                heap_caps_free(pool.buffers[j]);
                        }

                        vSemaphoreDelete(pool.mutex);
                        return ESP_ERR_NO_MEM;
                }

                pool.in_use[i] = false;
                LOG_DEBUG("Buffer %d allocated at %p", i, pool.buffers[i]);
        }

        pool.peak_usage = 0;
        pool.initialized = true;

        ESP_LOGI(
            TAG,
            "Buffer pool initialized: %d buffers x %d bytes = %d KB in PSRAM",
            IMAGE_BUFFER_COUNT, IMAGE_BUFFER_SIZE,
            (IMAGE_BUFFER_COUNT * IMAGE_BUFFER_SIZE) / 1024);

        return ESP_OK;
}

uint8_t *img_buffer_alloc(void)
{
        if (!pool.initialized) {
                ESP_LOGE(TAG, "Buffer pool not initialized!");
                return NULL;
        }

        xSemaphoreTake(pool.mutex, portMAX_DELAY);

        /* Find free buffer */
        for (int i = 0; i < IMAGE_BUFFER_COUNT; i++) {
                if (!pool.in_use[i]) {
                        pool.in_use[i] = true;

                        /* Update peak usage stats */
                        int current_usage = 0;
                        for (int j = 0; j < IMAGE_BUFFER_COUNT; j++) {
                                if (pool.in_use[j]) current_usage++;
                        }
                        if (current_usage > pool.peak_usage) {
                                pool.peak_usage = current_usage;
                        }

                        xSemaphoreGive(pool.mutex);

                        LOG_DEBUG("Allocated buffer %d (%d/%d in use)", i,
                                  current_usage, IMAGE_BUFFER_COUNT);

                        return pool.buffers[i];
                }
        }

        xSemaphoreGive(pool.mutex);

        ESP_LOGW(TAG, "No free buffers available! All %d buffers in use",
                 IMAGE_BUFFER_COUNT);

        return NULL;
}

void img_buffer_free(uint8_t *buffer)
{
        if (!pool.initialized) {
                ESP_LOGE(TAG, "Buffer pool not initialized!");
                return;
        }

        if (buffer == NULL) {
                ESP_LOGW(TAG, "Attempted to free NULL buffer");
                return;
        }

        xSemaphoreTake(pool.mutex, portMAX_DELAY);

        /* Find buffer in pool */
        for (int i = 0; i < IMAGE_BUFFER_COUNT; i++) {
                if (pool.buffers[i] == buffer) {
                        if (!pool.in_use[i]) {
                                ESP_LOGW(TAG, "Buffer %d was already free!",
                                         i);
                        }

                        pool.in_use[i] = false;
                        xSemaphoreGive(pool.mutex);

                        LOG_DEBUG("Freed buffer %d", i);
                        return;
                }
        }

        xSemaphoreGive(pool.mutex);

        ESP_LOGE(TAG, "Tried to free unknown buffer at %p!", buffer);
}

void img_buffer_get_stats(int *total, int *in_use, int *peak)
{
        if (!pool.initialized) {
                if (total) *total = 0;
                if (in_use) *in_use = 0;
                if (peak) *peak = 0;
                return;
        }

        xSemaphoreTake(pool.mutex, portMAX_DELAY);

        if (total) *total = IMAGE_BUFFER_COUNT;

        if (in_use) {
                int count = 0;
                for (int i = 0; i < IMAGE_BUFFER_COUNT; i++) {
                        if (pool.in_use[i]) count++;
                }
                *in_use = count;
        }

        if (peak) *peak = pool.peak_usage;

        xSemaphoreGive(pool.mutex);
}

/**************************************************************************************************
  Save Ring Buffer
**************************************************************************************************/

static struct {
        uint8_t *slots[SAVE_RING_SIZE];
        int64_t timestamps[SAVE_RING_SIZE];
        int head;
        int count;
        bool initialized;
} ring = {0};

esp_err_t save_ring_init(void)
{
        if (ring.initialized) {
                ESP_LOGW(TAG, "Save ring already initialized");
                return ESP_OK;
        }

        for (int i = 0; i < SAVE_RING_SIZE; i++) {
                ring.slots[i] =
                    heap_caps_malloc(IMAGE_BUFFER_SIZE, MALLOC_CAP_SPIRAM);
                if (ring.slots[i] == NULL) {
                        ESP_LOGE(TAG, "Save ring alloc failed at slot %d", i);
                        for (int j = 0; j < i; j++)
                                heap_caps_free(ring.slots[j]);
                        return ESP_ERR_NO_MEM;
                }
        }

        ring.head = 0;
        ring.count = 0;
        ring.initialized = true;

        ESP_LOGI(TAG, "Save ring: %d slots x %d bytes = %d KB in PSRAM",
                 SAVE_RING_SIZE, IMAGE_BUFFER_SIZE,
                 (SAVE_RING_SIZE * IMAGE_BUFFER_SIZE) / 1024);

        return ESP_OK;
}

void save_ring_push(const uint8_t *frame, size_t len, int64_t timestamp)
{
        if (!ring.initialized) return;

        size_t n = (len > IMAGE_BUFFER_SIZE) ? IMAGE_BUFFER_SIZE : len;
        memcpy(ring.slots[ring.head], frame, n);
        ring.timestamps[ring.head] = timestamp;

        ring.head = (ring.head + 1) % SAVE_RING_SIZE;
        if (ring.count < SAVE_RING_SIZE) ring.count++;
}

const uint8_t *save_ring_get(int idx, int64_t *timestamp_out)
{
        if (!ring.initialized || idx < 0 || idx >= ring.count) return NULL;

        /* idx 0 = oldest */
        int pos;
        if (ring.count < SAVE_RING_SIZE) pos = idx;
        else pos = (ring.head + idx) % SAVE_RING_SIZE;

        if (timestamp_out) *timestamp_out = ring.timestamps[pos];
        return ring.slots[pos];
}

int save_ring_count(void) { return ring.count; }

void save_ring_reset(void)
{
        ring.head = 0;
        ring.count = 0;
}

/**************************************************************************************************
  Testing
**************************************************************************************************/

#ifdef UNIT_TEST_BUFF_POOL
void img_buf_unit_test(void)
{
        ESP_LOGI(TAG, "=== Buffer Pool Unit Test ===");

        if (!pool.initialized) {
                ESP_LOGE(TAG, "Pool not initialized!");
                return;
        }

        /* Test 1: Allocate all buffers */
        ESP_LOGI(TAG, "Test 1: Allocating all buffers...");
        uint8_t *bufs[IMAGE_BUFFER_COUNT];
        for (int i = 0; i < IMAGE_BUFFER_COUNT; i++) {
                bufs[i] = img_buffer_alloc();
                if (bufs[i] == NULL) {
                        ESP_LOGE(TAG, "Failed to allocate buffer %d", i);
                        return;
                }
                ESP_LOGI(TAG, "  Buffer %d: %p", i, bufs[i]);
        }

        /* Test 2: Try to allocate one more (should fail) */
        ESP_LOGI(TAG, "Test 2: Attempting overflow allocation...");
        uint8_t *overflow = img_buffer_alloc();
        if (overflow == NULL) {
                ESP_LOGI(TAG,
                         "  ✓ Correctly returned NULL (all buffers in use)");
        } else {
                ESP_LOGE(TAG, "  ✗ ERROR: Allocated beyond pool size!");
        }

        /* Test 3: Free all buffers */
        ESP_LOGI(TAG, "Test 3: Freeing all buffers...");
        for (int i = 0; i < IMAGE_BUFFER_COUNT; i++) {
                img_buffer_free(bufs[i]);
                ESP_LOGI(TAG, "  Freed buffer %d", i);
        }

        /* Test 4: Check stats */
        int total, in_use, peak;
        img_buffer_get_stats(&total, &in_use, &peak);
        ESP_LOGI(TAG, "Test 4: Stats - Total: %d, In use: %d, Peak: %d", total,
                 in_use, peak);

        if (in_use == 0 && peak == IMAGE_BUFFER_COUNT) {
                ESP_LOGI(TAG, "✓ All tests PASSED");
        } else {
                ESP_LOGE(TAG,
                         "✗ Test FAILED: in_use=%d (expected 0), peak=%d "
                         "(expected %d)",
                         in_use, IMAGE_BUFFER_COUNT, IMAGE_BUFFER_COUNT);
        }

        ESP_LOGI(TAG, "=== Unit Test Complete ===");
}
#endif
