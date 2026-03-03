#include "peripherals/sensors_temp_humidity.h"
#include <stdio.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/gpio.h"
#include "peripherals/i2c_bus.h"
#include "peripherals/aht20.h"
#include "peripherals/motor_i2c.h"

static const char *TAG = "sensors";

#define I2C_SDA_GPIO     21
#define I2C_SCL_GPIO     22
#define PIR1_GPIO        35
#define PIR2_GPIO        34
#define PIR3_GPIO         4

#define STEPS_90         1024
#define HOLD_MS          3000
#define PIR_SETTLE_MS    30000

#define FPGA_ADDR        0x27
#define FPGA_REG_TEMP    0x02
#define FPGA_REG_RH      0x03

static aht20_t s_aht20;
static bool    s_sensor_ok;

static void pir_init_pin(gpio_num_t pin, bool input_only)
{
    gpio_config_t io = {
        .pin_bit_mask  = (1ULL << pin),
        .mode          = GPIO_MODE_INPUT,
        .pull_up_en    = GPIO_PULLUP_DISABLE,
        .pull_down_en  = input_only ? GPIO_PULLDOWN_DISABLE : GPIO_PULLDOWN_ENABLE,
        .intr_type     = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io));
}

static int detect_triggered_pir(void)
{
    if (gpio_get_level((gpio_num_t)PIR1_GPIO)) return 1;
    if (gpio_get_level((gpio_num_t)PIR2_GPIO)) return 2;
    if (gpio_get_level((gpio_num_t)PIR3_GPIO)) return 3;
    return 0;
}

static void wait_for_all_pirs_low(void)
{
    int elapsed = 0;
    while (gpio_get_level((gpio_num_t)PIR1_GPIO) ||
           gpio_get_level((gpio_num_t)PIR2_GPIO) ||
           gpio_get_level((gpio_num_t)PIR3_GPIO))
    {
        if (elapsed >= PIR_SETTLE_MS) break;
        vTaskDelay(pdMS_TO_TICKS(100));
        elapsed += 100;
    }
    vTaskDelay(pdMS_TO_TICKS(200));
}

static void fpga_send_sensor_data(float temp_f, float rh)
{
    uint8_t t_byte = (uint8_t)(temp_f < 0 ? 0 : temp_f > 255 ? 255 : temp_f);
    uint8_t r_byte = (uint8_t)(rh < 0 ? 0 : rh > 100 ? 100 : rh);

    uint8_t buf_t[2] = { FPGA_REG_TEMP, t_byte };
    uint8_t buf_r[2] = { FPGA_REG_RH,   r_byte };

    esp_err_t err;
    err = i2c_bus_write(FPGA_ADDR, buf_t, sizeof(buf_t), 100);
    if (err != ESP_OK)
        ESP_LOGW(TAG, "FPGA temp write failed: %s", esp_err_to_name(err));

    err = i2c_bus_write(FPGA_ADDR, buf_r, sizeof(buf_r), 100);
    if (err != ESP_OK)
        ESP_LOGW(TAG, "FPGA RH write failed: %s", esp_err_to_name(err));
}

static inline float c_to_f(float c) { return c * 9.0f / 5.0f + 32.0f; }

esp_err_t sensors_init(void)
{
    pir_init_pin((gpio_num_t)PIR1_GPIO, true);
    pir_init_pin((gpio_num_t)PIR2_GPIO, true);
    pir_init_pin((gpio_num_t)PIR3_GPIO, false);

    if (!i2c_bus_is_init()) {
        const i2c_bus_config_t bus_cfg = {
            .port                   = I2C_NUM_0,
            .sda_gpio               = I2C_SDA_GPIO,
            .scl_gpio               = I2C_SCL_GPIO,
            .enable_internal_pullup = true,
        };
        esp_err_t err = i2c_bus_init(&bus_cfg);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "I2C bus init failed: %s", esp_err_to_name(err));
            return err;
        }
    }

    s_sensor_ok = (aht20_init(&s_aht20, AHT20_I2C_ADDR_DEFAULT) == ESP_OK);
    if (!s_sensor_ok)
        ESP_LOGW(TAG, "AHT20 unavailable — continuing without temp/humidity");

    esp_err_t err = motor_i2c_init();
    if (err != ESP_OK)
        ESP_LOGW(TAG, "motor_i2c_init failed — FPGA not connected?");

    ESP_LOGI(TAG, "sensor subsystem initialised");
    return ESP_OK;
}

void sensors_task(void *pvParameters)
{
    ESP_LOGI(TAG, "sensor task started — polling PIRs");

    for (;;) {
        int trig = detect_triggered_pir();

        if (trig == 0) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        float t_f = 0, rh = 0;
        if (s_sensor_ok) {
            float t_c = 0;
            if (aht20_read(&s_aht20, &t_c, &rh) == ESP_OK) {
                t_f = c_to_f(t_c);
                ESP_LOGI(TAG, "PIR%d | %.1f F | %.0f%% RH", trig, t_f, rh);
                fpga_send_sensor_data(t_f, rh);
            } else {
                ESP_LOGW(TAG, "PIR%d | AHT20 read failed", trig);
            }
        } else {
            ESP_LOGI(TAG, "PIR%d | sensor unavailable", trig);
        }

        switch (trig) {
            case 1: motor_i2c_swing_cw (STEPS_90, HOLD_MS); break;
            case 2: motor_i2c_swing_ccw(STEPS_90, HOLD_MS); break;
            case 3: break;
        }

        wait_for_all_pirs_low();
    }
}