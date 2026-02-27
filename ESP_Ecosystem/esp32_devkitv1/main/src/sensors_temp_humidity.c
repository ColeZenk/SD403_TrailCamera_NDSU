/**
 * sensors_temp_humidity.c — Sensor subsystem task
 *
 * PIR wake → AHT20 read → motor swing → sleep.
 * Runs as a task under main.c, not standalone.
 *
 * NOTE: This file uses the v5.1 new I2C master driver
 * (i2c_master_bus_handle_t) separately from i2c_bus.c which
 * uses the legacy driver. These are on different I2C ports
 * or need to be reconciled eventually.
 */

#include <stdio.h>
#include <stdbool.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"

#include "peripherals/sensors_temp_humidity.h"
#include "peripherals/motor.h"

static const char *TAG = "SENSORS";

/*******************************************************************************
 * Config
 ******************************************************************************/

#define I2C_PORT      I2C_NUM_0
#define I2C_SDA       21
#define I2C_SCL       22
#define I2C_SPEED     100000
#define AHT20_ADDR    0x38

#define PIR1_PIN      35
#define PIR2_PIN      34
#define PIR3_PIN       4

#define STEP_IN1      13
#define STEP_IN2      12
#define STEP_IN3      14
#define STEP_IN4      27

#define STEPS_90      1024
#define HOLD_MS       3000
#define SETTLE_TIMEOUT_MS  30000

/*******************************************************************************
 * PIR Helpers
 ******************************************************************************/

static void pir_init_pin(gpio_num_t pin, bool input_only)
{
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << pin),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = input_only ? GPIO_PULLDOWN_DISABLE : GPIO_PULLDOWN_ENABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&io);
}

static int which_pir_triggered(void)
{
    if (gpio_get_level(PIR1_PIN)) return 1;
    if (gpio_get_level(PIR2_PIN)) return 2;
    if (gpio_get_level(PIR3_PIN)) return 3;
    return 0;
}

static void wait_pirs_low(void)
{
    int elapsed = 0;
    while (gpio_get_level(PIR1_PIN) ||
           gpio_get_level(PIR2_PIN) ||
           gpio_get_level(PIR3_PIN))
    {
        if (elapsed >= SETTLE_TIMEOUT_MS) break;
        vTaskDelay(pdMS_TO_TICKS(100));
        elapsed += 100;
    }
    vTaskDelay(pdMS_TO_TICKS(200));
}

static inline float c_to_f(float c) { return c * 9.0f / 5.0f + 32.0f; }

/*******************************************************************************
 * AHT20 (new I2C master driver)
 ******************************************************************************/

typedef struct {
    i2c_master_dev_handle_t dev;
    bool ok;
} aht20_new_t;

static esp_err_t aht20_new_init(aht20_new_t *s, i2c_master_bus_handle_t bus)
{
    i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = AHT20_ADDR,
        .scl_speed_hz    = I2C_SPEED,
    };

    esp_err_t err = i2c_master_bus_add_device(bus, &cfg, &s->dev);
    if (err != ESP_OK) return err;

    /* Init sequence: reset → calibrate → wait */
    uint8_t reset = 0xBA;
    i2c_master_transmit(s->dev, &reset, 1, 1000);
    vTaskDelay(pdMS_TO_TICKS(40));

    uint8_t init_cmd[3] = { 0xBE, 0x08, 0x00 };
    err = i2c_master_transmit(s->dev, init_cmd, 3, 1000);
    if (err != ESP_OK) return err;
    vTaskDelay(pdMS_TO_TICKS(20));

    s->ok = true;
    return ESP_OK;
}

static esp_err_t aht20_new_read(aht20_new_t *s, float *temp_c, float *rh)
{
    uint8_t cmd[3] = { 0xAC, 0x33, 0x00 };
    esp_err_t err = i2c_master_transmit(s->dev, cmd, 3, 1000);
    if (err != ESP_OK) return err;

    vTaskDelay(pdMS_TO_TICKS(90));

    uint8_t data[6];
    for (int i = 0; i < 10; i++) {
        err = i2c_master_receive(s->dev, data, 6, 1000);
        if (err != ESP_OK) return err;

        if ((data[0] & 0x80) == 0) {
            uint32_t h = ((uint32_t)data[1] << 12) | ((uint32_t)data[2] << 4)
                       | ((data[3] >> 4) & 0x0F);
            uint32_t t = (((uint32_t)data[3] & 0x0F) << 16)
                       | ((uint32_t)data[4] << 8) | data[5];

            *rh     = (float)h * 100.0f / 1048576.0f;
            *temp_c = (float)t * 200.0f / 1048576.0f - 50.0f;
            return ESP_OK;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    return ESP_ERR_TIMEOUT;
}

/*******************************************************************************
 * Public Interface
 ******************************************************************************/

esp_err_t sensors_init(void)
{
    pir_init_pin(PIR1_PIN, true);
    pir_init_pin(PIR2_PIN, true);
    pir_init_pin(PIR3_PIN, false);

    ESP_LOGI(TAG, "PIRs configured: %d %d %d", PIR1_PIN, PIR2_PIN, PIR3_PIN);
    return ESP_OK;
}

void sensors_task(void *arg)
{
    /* Motor */
    motor_stepper_t motor = {
        .in1_gpio = STEP_IN1, .in2_gpio = STEP_IN2,
        .in3_gpio = STEP_IN3, .in4_gpio = STEP_IN4,
        .wire_map = {1, 0, 3, 2},
        .phase = 0,
    };
    motor_stepper_init(&motor);
    motor_stepper_release(&motor);

    /* I2C + AHT20 */
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port   = I2C_PORT,
        .sda_io_num = I2C_SDA,
        .scl_io_num = I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    i2c_master_bus_handle_t bus = NULL;
    aht20_new_t sensor = {0};

    if (i2c_new_master_bus(&bus_cfg, &bus) == ESP_OK) {
        if (i2c_master_probe(bus, AHT20_ADDR, 1000) == ESP_OK) {
            aht20_new_init(&sensor, bus);
        }
    }

    /* Configure GPIO wake from light sleep */
    gpio_wakeup_enable(PIR1_PIN, GPIO_INTR_HIGH_LEVEL);
    gpio_wakeup_enable(PIR2_PIN, GPIO_INTR_HIGH_LEVEL);
    gpio_wakeup_enable(PIR3_PIN, GPIO_INTR_HIGH_LEVEL);
    esp_sleep_enable_gpio_wakeup();

    ESP_LOGI(TAG, "sensor loop started");

    for (;;) {
        esp_light_sleep_start();

        int pir = which_pir_triggered();
        if (pir == 0) continue;

        /* Read temperature */
        if (sensor.ok) {
            float t = 0, rh = 0;
            if (aht20_new_read(&sensor, &t, &rh) == ESP_OK) {
                ESP_LOGI(TAG, "PIR%d | %.1f°F | %.0f%% RH", pir, c_to_f(t), rh);
            } else {
                ESP_LOGW(TAG, "PIR%d | read failed", pir);
            }
        }

        /* Motor response */
        switch (pir) {
        case 1: motor_stepper_swing_cw(&motor,  STEPS_90, HOLD_MS); break;
        case 2: motor_stepper_swing_ccw(&motor, STEPS_90, HOLD_MS); break;
        case 3: break;  /* temp/humidity only */
        }

        motor_stepper_release(&motor);
        wait_pirs_low();
    }
}
