//include($ENV{IDF_PATH}/tools/cmake/project.cmake)
//project(aht20_demo)

// main/aht20_demo.c
#include <stdio.h>
#include <stdbool.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_err.h"
#include "esp_sleep.h"

#include "driver/i2c_master.h"
#include "driver/gpio.h"

#include "aht20.h"
#include "motor.h"

static const char *TAG = "aht20_demo";

/* ====== EDIT THESE IF YOUR WIRING IS DIFFERENT ====== */
#define I2C_PORT_NUM      I2C_NUM_0
#define I2C_SDA_GPIO      21
#define I2C_SCL_GPIO      22
#define I2C_SPEED_HZ      100000
#define AHT20_I2C_ADDR    0x38

#define PIR1_GPIO         35
#define PIR2_GPIO         34
#define PIR3_GPIO         4

#define STEP_IN1_GPIO     13
#define STEP_IN2_GPIO     12
#define STEP_IN3_GPIO     14
#define STEP_IN4_GPIO     27

#define STEPS_90          1024
#define HOLD_MS           3000
#define PIR_SETTLE_TIMEOUT_MS  30000
/* ==================================================== */

static void pir_init_pin(gpio_num_t pin, bool input_only)
{
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << pin),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = input_only ? GPIO_PULLDOWN_DISABLE : GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io));
}

static int detect_triggered_pir_now(void)
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
           gpio_get_level((gpio_num_t)PIR3_GPIO)) {
        if (elapsed >= PIR_SETTLE_TIMEOUT_MS) break;
        vTaskDelay(pdMS_TO_TICKS(100));
        elapsed += 100;
    }
    vTaskDelay(pdMS_TO_TICKS(200));
}

static inline float c_to_f(float c)
{
    return (c * 9.0f / 5.0f) + 32.0f;
}

static esp_err_t i2c_bus_and_aht20_init(i2c_master_bus_handle_t *out_bus,
                                        i2c_master_dev_handle_t *out_dev)
{
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_PORT_NUM,
        .sda_io_num = I2C_SDA_GPIO,
        .scl_io_num = I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    i2c_master_bus_handle_t bus = NULL;
    esp_err_t err = i2c_new_master_bus(&bus_cfg, &bus);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_new_master_bus failed: %s", esp_err_to_name(err));
        return err;
    }

    err = i2c_master_probe(bus, AHT20_I2C_ADDR, 1000);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "AHT20 probe failed: %s", esp_err_to_name(err));
        i2c_del_master_bus(bus);
        return err;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = AHT20_I2C_ADDR,
        .scl_speed_hz = I2C_SPEED_HZ,
    };

    i2c_master_dev_handle_t dev = NULL;
    err = i2c_master_bus_add_device(bus, &dev_cfg, &dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_master_bus_add_device failed: %s", esp_err_to_name(err));
        i2c_del_master_bus(bus);
        return err;
    }

    *out_bus = bus;
    *out_dev = dev;
    return ESP_OK;
}

static void configure_light_sleep(void)
{
    // GPIO wakeup — wake on HIGH for each PIR
    ESP_ERROR_CHECK(gpio_wakeup_enable((gpio_num_t)PIR1_GPIO, GPIO_INTR_HIGH_LEVEL));
    ESP_ERROR_CHECK(gpio_wakeup_enable((gpio_num_t)PIR2_GPIO, GPIO_INTR_HIGH_LEVEL));
    ESP_ERROR_CHECK(gpio_wakeup_enable((gpio_num_t)PIR3_GPIO, GPIO_INTR_HIGH_LEVEL));
    ESP_ERROR_CHECK(esp_sleep_enable_gpio_wakeup());
}

void app_main(void)
{
    esp_log_level_set("*", ESP_LOG_INFO);

    pir_init_pin((gpio_num_t)PIR1_GPIO, true);
    pir_init_pin((gpio_num_t)PIR2_GPIO, true);
    pir_init_pin((gpio_num_t)PIR3_GPIO, false);

    // Motor init once — persists across light sleep
    motor_stepper_t motor = {
        .in1_gpio = STEP_IN1_GPIO,
        .in2_gpio = STEP_IN2_GPIO,
        .in3_gpio = STEP_IN3_GPIO,
        .in4_gpio = STEP_IN4_GPIO,
        .wire_map = {1,0,3,2},
        .phase = 0,
    };
    ESP_ERROR_CHECK(motor_stepper_init(&motor));
    motor_stepper_set_phase(&motor, 0);
    motor_stepper_release(&motor);

    // Sensor init once — persists across light sleep
    i2c_master_bus_handle_t bus = NULL;
    i2c_master_dev_handle_t i2c_dev = NULL;
    aht20_t sensor = {0};
    bool sensor_ok = false;

    esp_err_t ret = i2c_bus_and_aht20_init(&bus, &i2c_dev);
    if (ret == ESP_OK) {
        sensor_ok = (aht20_init(&sensor, i2c_dev) == ESP_OK);
    }

    configure_light_sleep();
    ESP_LOGI(TAG, "ready — entering light sleep loop");

    while (1) {
        // Sleep until a PIR goes HIGH
        esp_light_sleep_start();

        // Woke up — figure out which PIR
        int trig = detect_triggered_pir_now();
        if (trig == 0) continue; // spurious wake, go back to sleep

        // Read sensor
        if (sensor_ok) {
            float t_c = 0.0f, rh = 0.0f;
            esp_err_t r = aht20_read_temperature_humidity(&sensor, &t_c, &rh);
            if (r == ESP_OK) {
                ESP_LOGI(TAG, "PIR%d | %.1f F | %.0f%% RH", trig, c_to_f(t_c), rh);
            } else {
                ESP_LOGW(TAG, "PIR%d | read failed: %s", trig, esp_err_to_name(r));
            }
        } else {
            ESP_LOGI(TAG, "PIR%d | sensor unavailable", trig);
        }

        // Motor action
        switch (trig) {
            case 1: motor_stepper_swing_cw(&motor,  STEPS_90, HOLD_MS); break;
            case 2: motor_stepper_swing_ccw(&motor, STEPS_90, HOLD_MS); break;
            case 3: /* temp/humidity only */                              break;
            default: break;
        }

        motor_stepper_release(&motor);
        wait_for_all_pirs_low();
        // loop back and sleep again
    }
}