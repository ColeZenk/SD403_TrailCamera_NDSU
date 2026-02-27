/**
 * @file sensors_temp_humidity.c
 * @brief Sensor subsystem — AHT20 + PIR + stepper motor
 */

#include "peripherals/sensors_temp_humidity.h"

#include <stdio.h>
#include <stdbool.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_err.h"
#include "driver/gpio.h"

#include "peripherals/i2c_bus.h"
#include "peripherals/aht20.h"
#include "peripherals/motor.h"

static const char *TAG = "sensors";

#define I2C_SDA_GPIO      21
#define I2C_SCL_GPIO      22

#define PIR1_GPIO         35
#define PIR2_GPIO         34
#define PIR3_GPIO         4

#define STEP_IN1_GPIO     13
#define STEP_IN2_GPIO     12
#define STEP_IN3_GPIO     14
#define STEP_IN4_GPIO     27

#define STEPS_90                1024
#define HOLD_MS                 3000
#define PIR_SETTLE_TIMEOUT_MS   30000

static aht20_t          s_aht20;
static bool             s_sensor_ok;
static motor_stepper_t  s_motor;

static void pir_init_pin(gpio_num_t pin, bool input_only)
{
    gpio_config_t io = {
        .pin_bit_mask  = (1ULL << pin),
        .mode          = GPIO_MODE_INPUT,
        .pull_up_en    = GPIO_PULLUP_DISABLE,
        .pull_down_en  = input_only ? GPIO_PULLDOWN_DISABLE
                                    : GPIO_PULLDOWN_ENABLE,
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
        if (elapsed >= PIR_SETTLE_TIMEOUT_MS) break;
        vTaskDelay(pdMS_TO_TICKS(100));
        elapsed += 100;
    }
    vTaskDelay(pdMS_TO_TICKS(200));
}

static inline float c_to_f(float c) { return c * 9.0f / 5.0f + 32.0f; }

esp_err_t sensors_init(void)
{
    pir_init_pin((gpio_num_t)PIR1_GPIO, true);
    pir_init_pin((gpio_num_t)PIR2_GPIO, true);
    pir_init_pin((gpio_num_t)PIR3_GPIO, false);

    s_motor = (motor_stepper_t){
        .in1_gpio = STEP_IN1_GPIO,
        .in2_gpio = STEP_IN2_GPIO,
        .in3_gpio = STEP_IN3_GPIO,
        .in4_gpio = STEP_IN4_GPIO,
        .wire_map = {1, 0, 3, 2},
        .phase    = 0,
    };
    esp_err_t err = motor_stepper_init(&s_motor);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "motor init failed: %s", esp_err_to_name(err));
        return err;
    }
    motor_stepper_set_phase(&s_motor, 0);
    motor_stepper_release(&s_motor);

    if (!i2c_bus_is_init()) {
        const i2c_bus_config_t bus_cfg = {
            .port     = I2C_NUM_0,
            .sda_gpio = I2C_SDA_GPIO,
            .scl_gpio = I2C_SCL_GPIO,
            .enable_internal_pullup = true,
        };
        err = i2c_bus_init(&bus_cfg);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "I2C bus init failed: %s", esp_err_to_name(err));
            return err;
        }
    }

    s_sensor_ok = (aht20_init(&s_aht20, AHT20_I2C_ADDR_DEFAULT) == ESP_OK);
    if (!s_sensor_ok) {
        ESP_LOGW(TAG, "AHT20 unavailable — continuing without temp/humidity");
    }

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

        if (s_sensor_ok) {
            float t_c = 0, rh = 0;
            if (aht20_read(&s_aht20, &t_c, &rh) == ESP_OK) {
                ESP_LOGI(TAG, "PIR%d | %.1f F | %.0f%% RH", trig, c_to_f(t_c), rh);
            } else {
                ESP_LOGW(TAG, "PIR%d | AHT20 read failed", trig);
            }
        } else {
            ESP_LOGI(TAG, "PIR%d | sensor unavailable", trig);
        }

        switch (trig) {
            case 1: motor_stepper_swing_cw(&s_motor,  STEPS_90, HOLD_MS); break;
            case 2: motor_stepper_swing_ccw(&s_motor, STEPS_90, HOLD_MS); break;
            case 3: break;
        }

        motor_stepper_release(&s_motor);
        wait_for_all_pirs_low();
    }
}
