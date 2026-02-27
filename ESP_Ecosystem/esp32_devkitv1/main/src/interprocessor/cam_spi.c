/**
 * cam_spi.c — Camera SPI slave interface
 *
 * Receives chunked image data from ESP32-CAM over SPI DMA.
 * State machine: IDLE → header → data chunks → checksum → queue.
 */

#include "cam_spi.h"
#include "utils.h"
#include "esp_log.h"
#include "driver/spi_slave.h"
#include "driver/gpio.h"
#include <string.h>
#include <inttypes.h>

static const char *TAG = "CAM_SPI";

/*******************************************************************************
 * State
 ******************************************************************************/

typedef enum {
    RX_IDLE,
    RX_DATA,
    RX_COMPLETE
} rx_state_t;

typedef struct {
    rx_state_t     state;
    image_header_t header;
    uint8_t       *buffer;
    size_t         bytes_received;
    QueueHandle_t  queue;
} cam_spi_ctx_t;

static cam_spi_ctx_t ctx;

static DMA_ALIGNED uint8_t rx_buf[SPI_BUFFER_SIZE];
static DMA_ALIGNED uint8_t tx_buf[SPI_BUFFER_SIZE];

/*******************************************************************************
 * Receive State Machine
 ******************************************************************************/

static void rx_reset(void)
{
    safe_free((void **)&ctx.buffer);
    ctx.bytes_received = 0;
    ctx.state = RX_IDLE;
    memset(&ctx.header, 0, sizeof(ctx.header));
}

static bool rx_process_header(const uint8_t *data, size_t len)
{
    if (len < sizeof(image_header_t)) return false;

    memcpy(&ctx.header, data, sizeof(image_header_t));

    if (!validate_image_header_fast(&ctx.header)) return false;

    ctx.buffer = dma_malloc(ctx.header.size);
    if (!ctx.buffer) {
        ESP_LOGE(TAG, "alloc failed: %" PRIu32 " bytes", ctx.header.size);
        return false;
    }

    ctx.state = RX_DATA;
    ctx.bytes_received = 0;

    /* Copy trailing data that arrived with the header */
    size_t extra = len - sizeof(image_header_t);
    if (extra > 0) {
        size_t n = min_size(extra, ctx.header.size);
        memcpy(ctx.buffer, data + sizeof(image_header_t), n);
        ctx.bytes_received = n;
    }

    ESP_LOGI(TAG, "header: %" PRIu32 " bytes incoming", ctx.header.size);
    return true;
}

static void rx_process_data(const uint8_t *data, size_t len)
{
    size_t remaining = ctx.header.size - ctx.bytes_received;
    size_t n = min_size(len, remaining);

    memcpy(ctx.buffer + ctx.bytes_received, data, n);
    ctx.bytes_received += n;

    if (ctx.bytes_received >= ctx.header.size) {
        ctx.state = RX_COMPLETE;
    }
}

static void rx_finalize(void)
{
    ESP_LOGI(TAG, "received: %zu bytes", ctx.bytes_received);

    if (!checksum_verify_xor_fast(ctx.buffer, ctx.header.size,
                                   ctx.header.checksum)) {
        ESP_LOGE(TAG, "checksum failed — dropping");
        rx_reset();
        return;
    }

    image_data_t img = {
        .buffer = ctx.buffer,
        .size   = ctx.header.size,
        .header = ctx.header,
    };

    if (xQueueSend(ctx.queue, &img, 0) == pdTRUE) {
        ctx.buffer = NULL;   /* queue owns it now */
    } else {
        ESP_LOGW(TAG, "queue full — dropping");
        safe_free((void **)&ctx.buffer);
    }

    ctx.state = RX_IDLE;
}

static void rx_handle_chunk(const uint8_t *data, size_t len)
{
    switch (ctx.state) {
    case RX_IDLE:
        rx_process_header(data, len);
        break;
    case RX_DATA:
        rx_process_data(data, len);
        if (ctx.state == RX_COMPLETE) rx_finalize();
        break;
    default:
        rx_reset();
        break;
    }
}

/*******************************************************************************
 * SPI DMA
 ******************************************************************************/

static esp_err_t spi_receive_chunk(size_t *out_len, uint32_t timeout_ticks)
{
    spi_slave_transaction_t trans = {
        .length    = SPI_BUFFER_SIZE * 8,
        .rx_buffer = rx_buf,
        .tx_buffer = tx_buf,
    };

    esp_err_t ret = spi_slave_transmit(CAM_SPI_HOST, &trans, timeout_ticks);

    if (ret == ESP_OK) {
        *out_len = trans.trans_len / 8;
    }

    return ret;
}

/*******************************************************************************
 * Public Interface
 ******************************************************************************/

esp_err_t cam_spi_init(void)
{
    ESP_LOGI(TAG, "Initializing camera SPI slave");

    ctx.queue = xQueueCreate(IMAGE_QUEUE_LENGTH, sizeof(image_data_t));
    if (!ctx.queue) return ESP_FAIL;

    spi_bus_config_t bus = {
        .mosi_io_num   = CAM_PIN_MOSI,
        .miso_io_num   = CAM_PIN_MISO,
        .sclk_io_num   = CAM_PIN_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = SPI_BUFFER_SIZE,
    };

    spi_slave_interface_config_t slave = {
        .mode        = CAM_SPI_MODE,
        .spics_io_num = CAM_PIN_CS,
        .queue_size  = CAM_SPI_QUEUE_SIZE,
        .flags       = 0,
    };

    esp_err_t ret = spi_slave_initialize(CAM_SPI_HOST, &bus, &slave,
                                          CAM_SPI_DMA_CHAN);
    if (ret != ESP_OK) {
        vQueueDelete(ctx.queue);
        ctx.queue = NULL;
        return ret;
    }

    gpio_set_pull_mode(CAM_PIN_CS, GPIO_PULLUP_ONLY);

    ESP_LOGI(TAG, "ready — MOSI=%d MISO=%d CLK=%d CS=%d",
             CAM_PIN_MOSI, CAM_PIN_MISO, CAM_PIN_SCLK, CAM_PIN_CS);
    return ESP_OK;
}

void cam_spi_deinit(void)
{
    rx_reset();
    spi_slave_free(CAM_SPI_HOST);
    if (ctx.queue) { vQueueDelete(ctx.queue); ctx.queue = NULL; }
}

QueueHandle_t cam_spi_get_queue(void)
{
    return ctx.queue;
}

/*******************************************************************************
 * Receive Task
 ******************************************************************************/

void cam_spi_receive_task(void *pvParameters)
{
    ESP_LOGI(TAG, "receive task started");

    for (;;) {
        size_t chunk_len = 0;
        esp_err_t ret = spi_receive_chunk(&chunk_len, CAM_TRANSFER_TIMEOUT_TICKS);

        if (ret == ESP_ERR_TIMEOUT) {
            if (ctx.state != RX_IDLE) {
                ESP_LOGW(TAG, "timeout — resetting");
                rx_reset();
            }
            continue;
        }

        if (ret != ESP_OK) continue;

        rx_handle_chunk(rx_buf, chunk_len);
    }
}
