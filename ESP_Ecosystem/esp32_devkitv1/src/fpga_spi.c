/**
 * @file fpga_spi.c
 * @brief FPGA SPI Master Interface - Tang Nano 9K Communication
 * @author ESP32 Image Pipeline Team
 *
 * Implements SPI master interface for transmitting image data to Tang Nano 9K FPGA.
 * Supports test pattern generation for development and debugging.
 *
 * @scope
 * File covers SPI transmission, DMA operations, and optional test pattern generation
 * with button-triggered pattern cycling.
 */

/*************************************************************************
 INCLUDES
 *************************************************************************/
#include "fpga_spi.h"
#include "isr_signals.h"
#include "utils.h"
#include "esp_log.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <string.h>

static const char *TAG = "FPGA_SPI";

/*************************************************************************
 STATE MANAGEMENT
 *************************************************************************/

typedef struct {
    spi_device_handle_t device;
    bool initialized;
} fpga_spi_context_t;

/*************************************************************************
 FILE SCOPED FUNCTIONS
 *************************************************************************/
__attribute__((always_inline))
static inline fpga_spi_context_t* getContext(void);

__attribute__((always_inline))
static inline esp_err_t transmitChunk(fpga_spi_context_t *ctx,
                                      const uint8_t *data,
                                      size_t size);

#ifdef TEST_MODE_FPGA_PATTERNS

typedef enum {
    PATTERN_WHITE,
    PATTERN_BLACK,
    PATTERN_GRADIENT,
    PATTERN_CHECKERBOARD,
    PATTERN_COUNT
} test_pattern_t;

static esp_err_t setupTestButton(void);

__attribute__((always_inline))
static inline void generateTestPattern(uint8_t *buffer, size_t size,
                                       test_pattern_t pattern);
#endif

/*************************************************************************
 CONTEXT ACCESSOR
 *************************************************************************/

__attribute__((always_inline))
static inline fpga_spi_context_t* getContext(void)
{
    static fpga_spi_context_t ctx = {0};
    return &ctx;
}

/*************************************************************************
 TEST MODE IMPLEMENTATION
 *************************************************************************/

#ifdef TEST_MODE_FPGA_PATTERNS

static esp_err_t setupTestButton(void)
{
    gpio_config_t btn_cfg = {
        .pin_bit_mask = (1ULL << TEST_BUTTON_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };

    esp_err_t ret = gpio_config(&btn_cfg);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = gpio_install_isr_service(0);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        return ret;
    }

    return gpio_isr_handler_add(TEST_BUTTON_PIN, ISR_OnButtonPress, NULL);
}

__attribute__((always_inline))
static inline void generateTestPattern(uint8_t *buffer, size_t size,
                                       test_pattern_t pattern)
{
    switch (pattern) {
        case PATTERN_WHITE:
            memset(buffer, 0xFF, size);
            ESP_LOGI(TAG, "Pattern: WHITE");
            break;

        case PATTERN_BLACK:
            memset(buffer, 0x00, size);
            ESP_LOGI(TAG, "Pattern: BLACK");
            break;

        case PATTERN_GRADIENT:
            for (size_t i = 0; i < size; i++) {
                buffer[i] = (i / 16) & 0xFF;
            }
            ESP_LOGI(TAG, "Pattern: GRADIENT");
            break;

        case PATTERN_CHECKERBOARD:
            for (size_t i = 0; i < size; i++) {
                buffer[i] = (i & 1) ? 0xAA : 0x55;
            }
            ESP_LOGI(TAG, "Pattern: CHECKERBOARD");
            break;

        default:
            memset(buffer, 0xFF, size);
            break;
    }
}

#endif // TEST_MODE_FPGA_PATTERNS

/*************************************************************************
 HOT PATH - TRANSMISSION
 *************************************************************************/

__attribute__((always_inline))
static inline esp_err_t transmitChunk(fpga_spi_context_t *ctx,
                                      const uint8_t *data,
                                      size_t size)
{
    spi_transaction_t trans = {
        .length = size * 8,
        .tx_buffer = data,
        .rx_buffer = NULL,
    };

    return spi_device_transmit(ctx->device, &trans);
}

/*************************************************************************
 PUBLIC INTERFACE
 *************************************************************************/

__attribute__((hot))
esp_err_t fpga_spi_transmit(const uint8_t *data, size_t size)
{
    fpga_spi_context_t *ctx = getContext();

    // FIXED: Added proper braces to prevent bug
    if (UNLIKELY(!ctx->initialized)) {
        ESP_LOGE(TAG, "Not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (UNLIKELY(!data || size == 0)) {
        ESP_LOGE(TAG, "Invalid parameters");
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Transmitting %zu bytes to FPGA", size);

    size_t offset = 0;
    while (offset < size) {
        size_t chunk_size = min_size(size - offset, FPGA_MAX_TRANSFER_SIZE);

        esp_err_t ret = transmitChunk(ctx, data + offset, chunk_size);
        if (UNLIKELY(ret != ESP_OK)) {
            ESP_LOGE(TAG, "Transmit failed at offset %zu: %s",
                     offset, esp_err_to_name(ret));
            return ret;
        }

        offset += chunk_size;
    }

    ESP_LOGI(TAG, "Transmission complete");
    return ESP_OK;
}

/*************************************************************************
  FPGA Communicatio State Management
 *************************************************************************/

esp_err_t fpga_spi_init(void)
{
    fpga_spi_context_t *ctx = getContext();

    ESP_LOGI(TAG, "Initializing FPGA SPI master interface");

    if (UNLIKELY(ctx->initialized)) {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_OK;
    }

    // Configure SPI bus
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = FPGA_PIN_MOSI,
        .miso_io_num = FPGA_PIN_MISO,
        .sclk_io_num = FPGA_PIN_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = FPGA_MAX_TRANSFER_SIZE,
        .flags = SPICOMMON_BUSFLAG_MASTER,
    };

    esp_err_t ret = spi_bus_initialize(FPGA_SPI_HOST, &bus_cfg, FPGA_SPI_DMA_CHAN);
    if (UNLIKELY(ret != ESP_OK)) {
        ESP_LOGE(TAG, "SPI bus init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Configure SPI device
    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = FPGA_SPI_CLOCK_HZ,
        .mode = FPGA_SPI_MODE,
        .spics_io_num = FPGA_PIN_CS,
        .queue_size = FPGA_SPI_QUEUE_SIZE,
        .flags = 0,
    };

    ret = spi_bus_add_device(FPGA_SPI_HOST, &dev_cfg, &ctx->device);
    if (UNLIKELY(ret != ESP_OK)) {
        ESP_LOGE(TAG, "SPI device add failed: %s", esp_err_to_name(ret));
        spi_bus_free(FPGA_SPI_HOST);
        return ret;
    }

    ctx->initialized = true;

    ESP_LOGI(TAG, "FPGA SPI initialized - MOSI=%d MISO=%d CLK=%d CS=%d",
             FPGA_PIN_MOSI, FPGA_PIN_MISO, FPGA_PIN_SCLK, FPGA_PIN_CS);
    ESP_LOGI(TAG, "Clock: %d MHz", FPGA_SPI_CLOCK_MHZ);

#ifdef TEST_MODE_FPGA_PATTERNS
    ret = setupTestButton();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Test button setup failed: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Test button configured on GPIO %d", TEST_BUTTON_PIN);
    }
#endif

    return ESP_OK;
}

void fpga_spi_deinit(void)
{
    fpga_spi_context_t *ctx = getContext();

    if (!ctx->initialized) return;

#ifdef TEST_MODE_FPGA_PATTERNS
    gpio_isr_handler_remove(TEST_BUTTON_PIN);
#endif

    spi_bus_remove_device(ctx->device);
    spi_bus_free(FPGA_SPI_HOST);

    ctx->device = NULL;
    ctx->initialized = false;

    ESP_LOGI(TAG, "FPGA SPI deinitialized");
}

/*************************************************************************
 TEST TASK
 *************************************************************************/

#ifdef TEST_MODE_FPGA_PATTERNS

void fpga_test_task(void *pvParameters)
{
    if (!g_button_sem) {
        ESP_LOGE(TAG, "Button semaphore not created, test task exiting");
        vTaskDelete(NULL);
        return;
    }

    uint8_t *test_buffer = dma_malloc(TEST_IMAGE_SIZE);
    if (!test_buffer) {
        ESP_LOGE(TAG, "Failed to allocate test buffer");
        vTaskDelete(NULL);
        return;
    }

    test_pattern_t pattern = PATTERN_WHITE;
    ESP_LOGI(TAG, "Test task ready - press BOOT button to cycle patterns");

    for (;;) {
        if (xSemaphoreTake(g_button_sem, portMAX_DELAY) == pdTRUE) {
            ESP_LOGI(TAG, "Button pressed - Pattern %d", pattern);
            generateTestPattern(test_buffer, TEST_IMAGE_SIZE, pattern);

            esp_err_t ret = fpga_spi_transmit(test_buffer, TEST_IMAGE_SIZE);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Test pattern transmission failed");
            }

            pattern = (pattern + 1) % PATTERN_COUNT;
        }
    }

    free(test_buffer);
    vTaskDelete(NULL);
}

#endif // TEST_MODE_FPGA_PATTERNS
