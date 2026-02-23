/**
 * @file cam_spi.c
 * @brief Camera SPI Slave Interface - ESP32-CAM Communication
 * @author ESP32 Image Pipeline Team
 *
 * Implements SPI slave interface for receiving image data from ESP32-CAM.
 * Handles chunked transfers, header validation, and checksum verification.
 *
 * @scope
 * File covers SPI slave configuration, DMA receive operations, image buffer
 * management, and state machine for reliable multi-packet reception.
 */

/*************************************************************************
 INCLUDES
 *************************************************************************/
#include "cam_spi.h"
#include "utils.h"
#include "esp_log.h"
#include "driver/spi_slave.h"
#include "driver/gpio.h"
#include <string.h>
#include <inttypes.h>

static const char *TAG = "CAM_SPI";

/*************************************************************************
 STATE MANAGEMENT
 *************************************************************************/

typedef enum {
    RX_STATE_IDLE,
    RX_STATE_RECEIVING_HEADER,
    RX_STATE_RECEIVING_DATA,
    RX_STATE_COMPLETE
} rx_state_t;

typedef struct {
    rx_state_t state;
    image_header_t header;
    uint8_t *buffer;
    size_t bytes_received;
    QueueHandle_t queue;
} cam_spi_context_t;

/*************************************************************************
 DMA BUFFERS (must be in DMA-capable memory and aligned)
 *************************************************************************/
static DMA_ALIGNED uint8_t rx_buffer[SPI_BUFFER_SIZE];
static DMA_ALIGNED uint8_t tx_buffer[SPI_BUFFER_SIZE];

/*************************************************************************
 FILE SCOPED FUNCTIONS
 *************************************************************************/
__attribute__((always_inline))
static inline cam_spi_context_t* getContext(void);

__attribute__((always_inline))
static inline void resetRxState(cam_spi_context_t *ctx);

__attribute__((always_inline))
static inline bool processHeader(cam_spi_context_t *ctx,
                                  const uint8_t *data,
                                  size_t size);

__attribute__((always_inline))
static inline void processData(cam_spi_context_t *ctx,
                               const uint8_t *data,
                               size_t size);

__attribute__((always_inline))
static inline void finalizeImage(cam_spi_context_t *ctx);

__attribute__((always_inline))
static inline void handleChunk(cam_spi_context_t *ctx,
                               const uint8_t *data,
                               size_t size);

static esp_err_t receiveChunk(uint8_t *buffer, size_t *length,
                               uint32_t timeout_ticks);

/*************************************************************************
 CONTEXT ACCESSOR
 *************************************************************************/

__attribute__((always_inline))
static inline cam_spi_context_t* getContext(void)
{
    static cam_spi_context_t ctx = {0};
    return &ctx;
}

/*************************************************************************
 HOT PATH FUNCTIONS - AGGRESSIVELY INLINED
 *************************************************************************/

__attribute__((always_inline))
static inline void resetRxState(cam_spi_context_t *ctx)
{
    safe_free((void**)&ctx->buffer);
    ctx->bytes_received = 0;
    ctx->state = RX_STATE_IDLE;
    memset(&ctx->header, 0, sizeof(ctx->header));
}

__attribute__((always_inline))
static inline bool processHeader(cam_spi_context_t *ctx,
                                  const uint8_t *data,
                                  size_t size)
{
    if (UNLIKELY(size < sizeof(image_header_t))) {
        return false;
    }

    // Fast copy
    memcpy(&ctx->header, data, sizeof(image_header_t));

    // Fast validation (inlined)
    if (UNLIKELY(!validate_image_header_fast(&ctx->header))) {
        return false;
    }

    // Allocate buffer for image data
    ctx->buffer = dma_malloc(ctx->header.size);
    if (UNLIKELY(!ctx->buffer)) {
        ESP_LOGE(TAG, "Failed to allocate %" PRIu32 " bytes for image", ctx->header.size);
        return false;
    }

    ctx->state = RX_STATE_RECEIVING_DATA;
    ctx->bytes_received = 0;

    // Copy any data that came with the header (fast path)
    size_t header_size = sizeof(image_header_t);
    if (LIKELY(size > header_size)) {
        size_t data_bytes = size - header_size;
        size_t copy_bytes = min_size(data_bytes, ctx->header.size);

        memcpy(ctx->buffer, data + header_size, copy_bytes);
        ctx->bytes_received = copy_bytes;
    }

    ESP_LOGI(TAG, "Header received - Image size: %" PRIu32 " bytes", ctx->header.size);
    return true;
}

__attribute__((always_inline))
static inline void processData(cam_spi_context_t *ctx,
                               const uint8_t *data,
                               size_t size)
{
    size_t remaining = ctx->header.size - ctx->bytes_received;
    size_t copy_bytes = min_size(size, remaining);

    memcpy(ctx->buffer + ctx->bytes_received, data, copy_bytes);
    ctx->bytes_received += copy_bytes;

    if (UNLIKELY(ctx->bytes_received >= ctx->header.size)) {
        ctx->state = RX_STATE_COMPLETE;
    }
}

__attribute__((always_inline))
static inline void finalizeImage(cam_spi_context_t *ctx)
{
    ESP_LOGI(TAG, "Image received: %zu bytes", ctx->bytes_received);

    // Fast checksum verification (inlined)
    if (UNLIKELY(!checksum_verify_xor_fast(ctx->buffer, ctx->header.size,
                                            ctx->header.checksum))) {
        ESP_LOGE(TAG, "Checksum verification failed - dropping image");
        resetRxState(ctx);
        return;
    }

    ESP_LOGI(TAG, "Image verified successfully");

    // Package for queue
    image_data_t img_data = {
        .buffer = ctx->buffer,
        .size = ctx->header.size,
        .header = ctx->header
    };

    // Send to processing queue
    if (UNLIKELY(xQueueSend(ctx->queue, &img_data, 0) != pdTRUE)) {
        ESP_LOGW(TAG, "Image queue full - dropping image");
        safe_free((void**)&ctx->buffer);
    } else {
        ctx->buffer = NULL; // Queue now owns the buffer
    }

    ctx->state = RX_STATE_IDLE;
}

__attribute__((always_inline))
static inline void handleChunk(cam_spi_context_t *ctx,
                               const uint8_t *data,
                               size_t size)
{
    switch (ctx->state) {
        case RX_STATE_IDLE:
            if (processHeader(ctx, data, size)) {
                ESP_LOGI(TAG, "Started receiving image");
            }
            break;

        case RX_STATE_RECEIVING_DATA:
            processData(ctx, data, size);

            if (ctx->state == RX_STATE_COMPLETE) {
                finalizeImage(ctx);
            }
            break;

        default:
            ESP_LOGW(TAG, "Unexpected state: %d", ctx->state);
            resetRxState(ctx);
            break;
    }
}

/*************************************************************************
 DMA OPERATIONS
 *************************************************************************/

/**
 * @brief Receive one SPI chunk via DMA
 *
 * @param buffer Buffer to receive data into
 * @param length Output parameter for received length
 * @param timeout_ticks FreeRTOS timeout in ticks
 * @return ESP_OK on success, error code otherwise
 */
static esp_err_t receiveChunk(uint8_t *buffer, size_t *length,
                               uint32_t timeout_ticks)
{
    spi_slave_transaction_t trans = {
        .length = SPI_BUFFER_SIZE * 8,
        .rx_buffer = buffer,
        .tx_buffer = tx_buffer,
    };

    esp_err_t ret = spi_slave_transmit(CAM_SPI_HOST, &trans, timeout_ticks);

    if (LIKELY(ret == ESP_OK)) {
        *length = trans.trans_len / 8;
    } else if (ret != ESP_ERR_TIMEOUT) {
        ESP_LOGE(TAG, "SPI receive error: %s", esp_err_to_name(ret));
    }

    return ret;
}

/*************************************************************************
 PUBLIC INTERFACE
 *************************************************************************/

/**
 * @brief Initialize camera SPI slave interface
 *
 * Configures SPI slave peripheral, creates image queue, and sets up
 * GPIO pull configuration.
 *
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t cam_spi_init(void)
{
    cam_spi_context_t *ctx = getContext();

    ESP_LOGI(TAG, "Initializing camera SPI slave interface");

    // Create image queue
    ctx->queue = xQueueCreate(IMAGE_QUEUE_LENGTH, sizeof(image_data_t));
    if (UNLIKELY(!ctx->queue)) {
        ESP_LOGE(TAG, "Failed to create image queue");
        return ESP_FAIL;
    }

    // Configure SPI bus
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = CAM_PIN_MOSI,
        .miso_io_num = CAM_PIN_MISO,
        .sclk_io_num = CAM_PIN_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = SPI_BUFFER_SIZE,
    };

    spi_slave_interface_config_t slave_cfg = {
        .mode = CAM_SPI_MODE,
        .spics_io_num = CAM_PIN_CS,
        .queue_size = CAM_SPI_QUEUE_SIZE,
        .flags = 0,
    };

    esp_err_t ret = spi_slave_initialize(CAM_SPI_HOST, &bus_cfg, &slave_cfg,
                                          CAM_SPI_DMA_CHAN);
    if (UNLIKELY(ret != ESP_OK)) {
        ESP_LOGE(TAG, "SPI slave init failed: %s", esp_err_to_name(ret));
        vQueueDelete(ctx->queue);
        ctx->queue = NULL;
        return ret;
    }

    // Enable pull-up on CS line
    gpio_set_pull_mode(CAM_PIN_CS, GPIO_PULLUP_ONLY);

    ESP_LOGI(TAG, "Camera SPI initialized - MOSI=%d MISO=%d CLK=%d CS=%d",
             CAM_PIN_MOSI, CAM_PIN_MISO, CAM_PIN_SCLK, CAM_PIN_CS);

    return ESP_OK;
}

/**
 * @brief Deinitialize camera SPI interface
 *
 * Frees all resources and resets state.
 */
void cam_spi_deinit(void)
{
    cam_spi_context_t *ctx = getContext();

    resetRxState(ctx);

    spi_slave_free(CAM_SPI_HOST);

    if (ctx->queue) {
        vQueueDelete(ctx->queue);
        ctx->queue = NULL;
    }

    ESP_LOGI(TAG, "Camera SPI deinitialized");
}

/**
 * @brief Get handle to image queue
 *
 * @return Queue handle for image data
 */
QueueHandle_t cam_spi_get_queue(void)
{
    return getContext()->queue;
}

/*************************************************************************
 RECEIVE TASK
 *************************************************************************/

/**
 * @brief Camera SPI receive task - handles incoming image data
 *
 * Continuously receives SPI chunks, processes headers and data,
 * and manages state machine for multi-packet image reception.
 *
 * @param pvParameters Unused task parameter
 */
__attribute__((hot))
void cam_spi_receive_task(void *pvParameters)
{
    cam_spi_context_t *ctx = getContext();

    ESP_LOGI(TAG, "Camera SPI receive task started");

    for (;;) {
        size_t chunk_size = 0;
        esp_err_t ret = receiveChunk(rx_buffer, &chunk_size,
                                      CAM_TRANSFER_TIMEOUT_TICKS);

        if (ret == ESP_ERR_TIMEOUT) {
            if (UNLIKELY(ctx->state != RX_STATE_IDLE)) {
                ESP_LOGW(TAG, "Transfer timeout - resetting state");
                resetRxState(ctx);
            }
            continue;
        }

        if (UNLIKELY(ret != ESP_OK)) {
            continue;
        }

        // Hot path - handle received chunk
        handleChunk(ctx, rx_buffer, chunk_size);
    }
}
