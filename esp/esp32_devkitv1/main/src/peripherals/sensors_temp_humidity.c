/**
 * @file sensors_temp_humidity.c
 * @brief Sensor subsystem - AHT20 + PIR + stepper motor + DS3231 RTC
 */

#include "peripherals/sensors_temp_humidity.h"
#include "config.h"
#include "isr_signals.h"

#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>
#include <time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"

#include "peripherals/aht20.h"
#include "peripherals/i2c_bus.h"
#include "peripherals/motor.h"
#include "peripherals/rtc.h"

static const char *TAG = "sensors";

#define I2C_SDA_GPIO 21
#define I2C_SCL_GPIO 22

#define PIR1_GPIO 35
#define PIR2_GPIO 34
#define PIR3_GPIO 4

#define STEP_IN1_GPIO 13
#define STEP_IN2_GPIO 12
#define STEP_IN3_GPIO 14
#define STEP_IN4_GPIO 27

// FPGA + camera enable pins
#define FPGA_EN_GPIO 2
#define CAM_EN_GPIO  15

#define STEPS_90              1024
#define HOLD_MS               3000
#define PIR_SETTLE_TIMEOUT_MS 30000
#define PIR3_ACTIVE_MS        (HOLD_MS + 2000)

// Force PIR debug mode OFF so the PIRs are actually polled.
// (When it was on, the task just held the outputs high and never polled.)
// To turn the always-on debug mode back on for image testing, delete this line.
#undef DEBUG_PIR_ALWAYS_ON

static aht20_t s_aht20;
static bool s_sensor_ok;
static motor_stepper_t s_motor;
static float s_last_temp = 0.0f;
static float s_last_hum  = 0.0f;
static bool s_rtc_ok;
static struct tm s_last_time;
static bool s_last_time_valid = false;

static void pir_init_pin(gpio_num_t pin, bool input_only)
{
        gpio_config_t io = {
            .pin_bit_mask = (1ULL << pin),
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en =
                input_only ? GPIO_PULLDOWN_DISABLE : GPIO_PULLDOWN_ENABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        ESP_ERROR_CHECK(gpio_config(&io));
}

static void enable_outputs_init(void)
{
        gpio_config_t io = {
            .pin_bit_mask = (1ULL << FPGA_EN_GPIO) | (1ULL << CAM_EN_GPIO),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        ESP_ERROR_CHECK(gpio_config(&io));

        gpio_set_level((gpio_num_t)FPGA_EN_GPIO, 1);
        gpio_set_level((gpio_num_t)CAM_EN_GPIO, 1);
}

static inline void enable_outputs_set(bool on)
{
        int level = on ? 1 : 0;
        gpio_set_level((gpio_num_t)FPGA_EN_GPIO, level);
        gpio_set_level((gpio_num_t)CAM_EN_GPIO, level);
}

static int detect_triggered_pir(void)
{
#ifdef DEBUG_PIR_ALWAYS_ON
        return 3;
#else
        if (gpio_get_level((gpio_num_t)PIR1_GPIO)) return 1;
        if (gpio_get_level((gpio_num_t)PIR2_GPIO)) return 2;
        if (gpio_get_level((gpio_num_t)PIR3_GPIO)) return 3;
        return 0;
#endif
}

static void wait_for_all_pirs_low(void)
{
        int elapsed = 0;

        while (gpio_get_level((gpio_num_t)PIR1_GPIO) ||
               gpio_get_level((gpio_num_t)PIR2_GPIO) ||
               gpio_get_level((gpio_num_t)PIR3_GPIO)) {
                if (elapsed >= PIR_SETTLE_TIMEOUT_MS) { break; }

                vTaskDelay(pdMS_TO_TICKS(100));
                elapsed += 100;
        }

        vTaskDelay(pdMS_TO_TICKS(200));
}

static inline float c_to_f(float c) { return c * 9.0f / 5.0f + 32.0f; }

// Read the DS3231, cache the result (like temp/humidity) and format it.
static void read_rtc_timestamp(char *buf, size_t len)
{
        struct tm t;

        if (s_rtc_ok && ds3231_get_time(&t) == ESP_OK) {
                s_last_time       = t;
                s_last_time_valid = true;
                strftime(buf, len, "%Y-%m-%d %H:%M:%S", &t);
        } else {
                snprintf(buf, len, "no-rtc");
        }
}

// Periodic status line so temp/humidity + date/time print even with no PIR
// motion. Set to 0 to disable.
#define STATUS_LOG_PERIOD_MS 5000

static void log_status(void)
{
        char ts[24];
        read_rtc_timestamp(ts, sizeof(ts));

        if (s_sensor_ok) {
                float t_c = 0.0f;
                float rh  = 0.0f;

                if (aht20_read(&s_aht20, &t_c, &rh) == ESP_OK) {
                        s_last_temp = t_c;
                        s_last_hum  = rh;
                        ESP_LOGI(TAG, "STATUS | %.1f F | %.0f%% RH | %s",
                                 c_to_f(t_c), rh, ts);
                        return;
                }
        }

        ESP_LOGI(TAG, "STATUS | no temp/humidity | %s", ts);
}

esp_err_t sensors_init(void)
{
        // PIR pins
        pir_init_pin((gpio_num_t)PIR1_GPIO, true);
        pir_init_pin((gpio_num_t)PIR2_GPIO, true);
        pir_init_pin((gpio_num_t)PIR3_GPIO, false);

        // FPGA + camera enable pins
        enable_outputs_init();

        // Stepper configuration
        s_motor = (motor_stepper_t){
            .in1_gpio = STEP_IN1_GPIO,
            .in2_gpio = STEP_IN2_GPIO,
            .in3_gpio = STEP_IN3_GPIO,
            .in4_gpio = STEP_IN4_GPIO,
            .wire_map = {1, 0, 3, 2},
            .phase = 0,
        };

        esp_err_t err = motor_stepper_init(&s_motor);
        if (err != ESP_OK) {
                ESP_LOGE(TAG, "motor_stepper_init failed: %s",
                         esp_err_to_name(err));
                return err;
        }

        motor_stepper_set_phase(&s_motor, 0);
        motor_stepper_release(&s_motor);

        // I2C bus
        if (!i2c_bus_is_init()) {
                const i2c_bus_config_t bus_cfg = {
                    .port = I2C_NUM_0,
                    .sda_gpio = I2C_SDA_GPIO,
                    .scl_gpio = I2C_SCL_GPIO,
                    .enable_internal_pullup = true,
                };

                err = i2c_bus_init(&bus_cfg);
                if (err != ESP_OK) {
                        ESP_LOGE(TAG, "I2C bus init failed: %s",
                                 esp_err_to_name(err));
                        return err;
                }
        }

        // AHT20
        s_sensor_ok = (aht20_init(&s_aht20, AHT20_I2C_ADDR_DEFAULT) == ESP_OK);
        if (!s_sensor_ok) {
                ESP_LOGW(
                    TAG,
                    "AHT20 unavailable - continuing without temp/humidity");
        }

        // DS3231 RTC
        s_rtc_ok = (ds3231_init(I2C_NUM_0) == ESP_OK);
        if (!s_rtc_ok) {
                ESP_LOGW(TAG, "DS3231 unavailable - system time not set");
        } else {
#ifdef RTC_SET_TIME_ONCE
                ds3231_set_time_from_build();
#endif
                bool lost = false;
                if (ds3231_lost_power(&lost) == ESP_OK && lost) {
                        ESP_LOGW(TAG, "RTC lost power - time is NOT valid, "
                                      "enable RTC_SET_TIME_ONCE to set it");
                }
                ds3231_sync_system_time();

                char now[24];
                read_rtc_timestamp(now, sizeof(now));
                ESP_LOGI(TAG, "DS3231 time now: %s", now);
        }

        ESP_LOGI(TAG, "sensor subsystem initialised");
        return ESP_OK;
}

int sensors_get_stepper_phase(void) { return s_motor.phase; }

uint32_t sensors_get_epoch(void)
{
        struct tm t;

        if (!s_rtc_ok || ds3231_get_time(&t) != ESP_OK) return 0;

        return (uint32_t)mktime(&t);
}

void sensors_get_last_readings(float *temp_c, float *humidity_pct)
{
        *temp_c       = s_last_temp;
        *humidity_pct = s_last_hum;
}

bool sensors_get_last_time(struct tm *t)
{
        if (!s_last_time_valid) return false;

        *t = s_last_time;
        return true;
}

void sensors_task(void *pvParameters)
{
        (void)pvParameters;

        ESP_LOGI(TAG, "sensor task started - polling PIRs");

#ifdef DEBUG_PIR_ALWAYS_ON
        enable_outputs_set(true);
        ESP_LOGW(TAG, "DEBUG_PIR_ALWAYS_ON - outputs held high");
        for (;;) vTaskDelay(pdMS_TO_TICKS(10000));
#else
        int idle_ms = 0;

        for (;;) {
                int trig = detect_triggered_pir();

                if (trig == 0) {
                        vTaskDelay(pdMS_TO_TICKS(100));
#if STATUS_LOG_PERIOD_MS > 0
                        idle_ms += 100;
                        if (idle_ms >= STATUS_LOG_PERIOD_MS) {
                                idle_ms = 0;
                                log_status();
                        }
#endif
                        continue;
                }

                char ts[24];
                read_rtc_timestamp(ts, sizeof(ts));

                if (s_sensor_ok) {
                        float t_c = 0.0f;
                        float rh = 0.0f;

                        if (aht20_read(&s_aht20, &t_c, &rh) == ESP_OK) {
                                s_last_temp = t_c;
                                s_last_hum  = rh;
                                ESP_LOGI(TAG, "PIR%d | %.1f F | %.0f%% RH | %s",
                                         trig, c_to_f(t_c), rh, ts);
                        } else {
                                ESP_LOGW(TAG, "PIR%d | AHT20 read failed | %s",
                                         trig, ts);
                        }
                } else {
                        ESP_LOGI(TAG, "PIR%d | sensor unavailable | %s", trig, ts);
                }

                enable_outputs_set(true);
                xSemaphoreGive(g_motion_sem);

                switch (trig) {
                case 1:
                        motor_stepper_swing_cw(&s_motor, STEPS_90, HOLD_MS);
                        break;

                case 2:
                        motor_stepper_swing_ccw(&s_motor, STEPS_90, HOLD_MS);
                        break;

                case 3:
                        vTaskDelay(pdMS_TO_TICKS(PIR3_ACTIVE_MS));
                        break;

                default: break;
                }

                motor_stepper_release(&s_motor);
                enable_outputs_set(false);
                wait_for_all_pirs_low();
        }
#endif
}
