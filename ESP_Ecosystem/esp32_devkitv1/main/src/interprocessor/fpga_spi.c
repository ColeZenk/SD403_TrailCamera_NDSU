/**
 * fpga_spi.c — FPGA SPI master interface (Tang Nano 9K)
 *
 * Async queue/get pattern: feeds DMA pipeline continuously,
 * no idle gaps between chunks.
 */

#include "fpga_spi.h"
#include "isr_signals.h"
#include "utils.h"
#include "esp_log.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "FPGA_SPI";

/*******************************************************************************
 * State
 ******************************************************************************/

static struct {
    spi_device_handle_t device;
    bool                initialized;
} ctx;

static spi_transaction_t trans_pool[FPGA_SPI_QUEUE_SIZE];

/*******************************************************************************
 * Test Patterns
 ******************************************************************************/

#ifdef TEST_MODE_FPGA_PATTERNS

typedef enum {
    PAT_WHITE,
    PAT_BLACK,
    PAT_GRADIENT,
    PAT_CHECKER,
    PAT_COUNT
} test_pattern_t;

static void generate_pattern(uint8_t *buf, size_t size, test_pattern_t pat)
{
    switch (pat) {
    case PAT_WHITE:   memset(buf, 0xFF, size); break;
    case PAT_BLACK:   memset(buf, 0x00, size); break;
    case PAT_GRADIENT:
        for (size_t i = 0; i < size; i++) buf[i] = (i / 16) & 0xFF;
        break;
    case PAT_CHECKER:
        for (size_t i = 0; i < size; i++) buf[i] = (i & 1) ? 0xAA : 0x55;
        break;
    default:
        memset(buf, 0xFF, size);
        break;
    }
}

static esp_err_t setup_test_button(void)
{
    gpio_config_t cfg = {
        .pin_bit_mask  = (1ULL << TEST_BUTTON_PIN),
        .mode          = GPIO_MODE_INPUT,
        .pull_up_en    = GPIO_PULLUP_ENABLE,
        .pull_down_en  = GPIO_PULLDOWN_DISABLE,
        .intr_type     = GPIO_INTR_NEGEDGE,
    };

    esp_err_t ret = gpio_config(&cfg);
    if (ret != ESP_OK) return ret;

    ret = gpio_install_isr_service(0);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) return ret;

    return gpio_isr_handler_add(TEST_BUTTON_PIN, ISR_OnButtonPress, NULL);
}

#endif

/*******************************************************************************
 * Transmit — async queue/get keeps DMA fed
 ******************************************************************************/

esp_err_t fpga_spi_transmit(const uint8_t *data, size_t size)
{
    if (!ctx.initialized) return ESP_ERR_INVALID_STATE;
    if (!data || size == 0) return ESP_ERR_INVALID_ARG;

    size_t offset    = 0;
    size_t in_flight = 0;
    size_t pool_idx  = 0;

    while (offset < size || in_flight > 0) {

        /* Fill queue slots */
        while (offset < size && in_flight < FPGA_SPI_QUEUE_SIZE) {
            size_t chunk = min_size(size - offset, FPGA_MAX_TRANSFER_SIZE);

            spi_transaction_t *t = &trans_pool[pool_idx % FPGA_SPI_QUEUE_SIZE];
            t->length    = chunk * 8;
            t->tx_buffer = data + offset;
            t->rx_buffer = NULL;
            t->flags     = 0;

            esp_err_t ret = spi_device_queue_trans(ctx.device, t, portMAX_DELAY);
            if (ret != ESP_OK) return ret;

            offset += chunk;
            in_flight++;
            pool_idx++;
        }

        /* Collect one completed transfer */
        spi_transaction_t *result;
        esp_err_t ret = spi_device_get_trans_result(ctx.device, &result,
                                                     portMAX_DELAY);
        if (ret != ESP_OK) return ret;
        in_flight--;
    }

    return ESP_OK;
}

/*******************************************************************************
 * Init / Deinit
 ******************************************************************************/

esp_err_t fpga_spi_init(void)
{
    if (ctx.initialized) return ESP_OK;

    spi_bus_config_t bus = {
        .mosi_io_num     = FPGA_PIN_MOSI,
        .miso_io_num     = FPGA_PIN_MISO,
        .sclk_io_num     = FPGA_PIN_SCLK,
        .quadwp_io_num   = -1,
        .quadhd_io_num   = -1,
        .max_transfer_sz = FPGA_MAX_TRANSFER_SIZE,
        .flags           = SPICOMMON_BUSFLAG_MASTER,
    };

    esp_err_t ret = spi_bus_initialize(FPGA_SPI_HOST, &bus, FPGA_SPI_DMA_CHAN);
    if (ret != ESP_OK) return ret;

    spi_device_interface_config_t dev = {
        .clock_speed_hz = FPGA_SPI_CLOCK_HZ,
        .mode           = FPGA_SPI_MODE,
        .spics_io_num   = FPGA_PIN_CS,
        .queue_size     = FPGA_SPI_QUEUE_SIZE,
        .flags          = 0,
    };

    ret = spi_bus_add_device(FPGA_SPI_HOST, &dev, &ctx.device);
    if (ret != ESP_OK) {
        spi_bus_free(FPGA_SPI_HOST);
        return ret;
    }

    ctx.initialized = true;
    ESP_LOGI(TAG, "ready — %d MHz, queue depth %d",
             FPGA_SPI_CLOCK_MHZ, FPGA_SPI_QUEUE_SIZE);

#ifdef TEST_MODE_FPGA_PATTERNS
    setup_test_button();
#endif

    return ESP_OK;
}

void fpga_spi_deinit(void)
{
    if (!ctx.initialized) return;

#ifdef TEST_MODE_FPGA_PATTERNS
    gpio_isr_handler_remove(TEST_BUTTON_PIN);
#endif

    spi_bus_remove_device(ctx.device);
    spi_bus_free(FPGA_SPI_HOST);
    ctx.device = NULL;
    ctx.initialized = false;
}

/*******************************************************************************
 * Test Task
 ******************************************************************************/

#ifdef TEST_MODE_FPGA_PATTERNS

void fpga_test_task(void *pvParameters)
{
    if (!g_button_sem) { vTaskDelete(NULL); return; }

    uint8_t *buf = dma_malloc(TEST_IMAGE_SIZE);
    if (!buf) { vTaskDelete(NULL); return; }

    test_pattern_t pat = PAT_WHITE;

    for (;;) {
        if (xSemaphoreTake(g_button_sem, portMAX_DELAY) == pdTRUE) {
            generate_pattern(buf, TEST_IMAGE_SIZE, pat);
            fpga_spi_transmit(buf, TEST_IMAGE_SIZE);
            pat = (pat + 1) % PAT_COUNT;
        }
    }
}

#endif
