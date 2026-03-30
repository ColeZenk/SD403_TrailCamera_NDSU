/**
 * fpga_gpio.c — FPGA I2C GPIO Expander Driver
 *
 * PCA9534-compatible I2C slave on Tang Nano 9K at address 0x27.
 * Requires i2c_bus to be initialised first.
 */

#include "peripherals/fpga_gpio.h"
#include "peripherals/i2c_bus.h"
#include "config.h"
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "FPGA_GPIO";

/*******************************************************************************
 * Internal helpers
 ******************************************************************************/

static esp_err_t write_reg(uint8_t reg, uint8_t value)
{
    uint8_t buf[2] = { reg, value };
    return i2c_bus_write(FPGA_I2C_ADDR, buf, sizeof(buf), FPGA_I2C_TIMEOUT_MS);
}

static esp_err_t read_reg(uint8_t reg, uint8_t *value)
{
    return i2c_bus_write_read(FPGA_I2C_ADDR,
                              &reg, 1,
                              value, 1,
                              FPGA_I2C_TIMEOUT_MS);
}

/*******************************************************************************
 * Public API
 ******************************************************************************/

esp_err_t fpga_gpio_init(void)
{
    esp_err_t err;

    /* Direction: bits[7:4]=inputs(unused), bits[3:0]=outputs(steppers) */
    err = write_reg(FPGA_GPIO_REG_DIR, FPGA_GPIO_DIR_BOOT);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "DIR_REG write failed: %s", esp_err_to_name(err));
        return err;
    }

    /* Polarity invert: bits[2:0] inverted so 1=pressed for active-low buttons */
    err = write_reg(FPGA_GPIO_REG_INV, FPGA_GPIO_INV_BOOT);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "INV_REG write failed: %s", esp_err_to_name(err));
        return err;
    }

    /* Clear steppers on startup */
    err = write_reg(FPGA_GPIO_REG_OUT, 0x00);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "OUT_REG clear failed: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "ready — addr 0x%02X, DIR=0x%02X, INV=0x%02X",
             FPGA_I2C_ADDR, FPGA_GPIO_DIR_BOOT, FPGA_GPIO_INV_BOOT);
    return ESP_OK;
}

esp_err_t fpga_gpio_set_steppers(uint8_t pattern)
{
    return write_reg(FPGA_GPIO_REG_OUT, pattern & 0x0F);
}

esp_err_t fpga_gpio_read_buttons(uint8_t *buttons)
{
    if (!buttons) return ESP_ERR_INVALID_ARG;
    uint8_t raw;
    esp_err_t err = read_reg(FPGA_GPIO_REG_IN, &raw);
    if (err != ESP_OK) return err;
    *buttons = raw & 0x07;  /* mask to 3 button bits */
    return ESP_OK;
}

/*******************************************************************************
 * Test Task
 ******************************************************************************/

#ifdef TEST_MODE_FPGA_GPIO

#include "fpga_spi.h"
#include "utils.h"

/* Test patterns sent to FPGA BRAM over SPI */
typedef enum {
    BPAT_GRADIENT = 0,
    BPAT_CHECKER,
    BPAT_WHITE,
    BPAT_BLACK,
    BPAT_COUNT
} bpat_t;

static const char *bpat_names[] = { "gradient", "checker", "white", "black" };

static void fill_bpat(uint8_t *buf, bpat_t pat)
{
    switch (pat) {
    case BPAT_GRADIENT:
        for (int i = 0; i < TEST_IMAGE_SIZE; i++) buf[i] = (uint8_t)i;
        break;
    case BPAT_CHECKER:
        for (int i = 0; i < TEST_IMAGE_SIZE; i++) buf[i] = (i & 0x10) ? 0xFF : 0x00;
        break;
    case BPAT_WHITE:
        memset(buf, 0xFF, TEST_IMAGE_SIZE);
        break;
    case BPAT_BLACK:
        memset(buf, 0x00, TEST_IMAGE_SIZE);
        break;
    default:
        memset(buf, 0x80, TEST_IMAGE_SIZE);
        break;
    }
}

void fpga_gpio_test_task(void *pvParameters)
{
    (void)pvParameters;

    uint8_t *buf = dma_malloc(TEST_IMAGE_SIZE);
    if (!buf) {
        ESP_LOGE(TAG, "DMA alloc failed — task exiting");
        vTaskDelete(NULL);
        return;
    }

    bpat_t pat = BPAT_GRADIENT;
    uint8_t prev_buttons = 0xFF;

    /* Send initial pattern so the LCD shows something on boot */
    fill_bpat(buf, pat);
    fpga_spi_transmit(buf, TEST_IMAGE_SIZE);
    ESP_LOGI(TAG, "boot pattern: %s", bpat_names[pat]);

    for (;;) {
        uint8_t buttons = 0;
        esp_err_t err = fpga_gpio_read_buttons(&buttons);

        if (err != ESP_OK) {
            ESP_LOGE(TAG, "read failed: %s", esp_err_to_name(err));
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        if (buttons != prev_buttons) {
            bool btn_l = (buttons >> FPGA_BTN_L_BIT) & 1;
            bool btn_r = (buttons >> FPGA_BTN_R_BIT) & 1;
            bool btn_s = (buttons >> FPGA_BTN_S_BIT) & 1;

            ESP_LOGI(TAG, "buttons: L=%d R=%d S=%d  (raw=0x%02X)",
                     btn_l, btn_r, btn_s, buttons);

            /* L = prev pattern, R = next pattern, S = reset to gradient */
            if (btn_l) pat = (bpat_t)((pat + BPAT_COUNT - 1) % BPAT_COUNT);
            if (btn_r) pat = (bpat_t)((pat + 1) % BPAT_COUNT);
            if (btn_s) pat = BPAT_GRADIENT;

            if (btn_l || btn_r || btn_s) {
                fill_bpat(buf, pat);
                esp_err_t tx = fpga_spi_transmit(buf, TEST_IMAGE_SIZE);
                if (tx == ESP_OK)
                    ESP_LOGI(TAG, "sent pattern: %s", bpat_names[pat]);
                else
                    ESP_LOGE(TAG, "SPI tx failed: %s", esp_err_to_name(tx));
            }

            /* Mirror to steppers */
            uint8_t step_pattern = 0x00;
            if (btn_s)      step_pattern = 0x0F;
            else if (btn_l) step_pattern = 0x01;
            else if (btn_r) step_pattern = 0x02;
            fpga_gpio_set_steppers(step_pattern);

            prev_buttons = buttons;
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

#endif
