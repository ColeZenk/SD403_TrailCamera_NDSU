// ESP32: 3 motion sensors + AHT20 + PWM motor control (ESP-IDF version)
// Sensor 1: 5 first (~0.4 s), wait, then 19 (0.25 s)
// Sensor 2: 19 first (0.25 s), wait, then 5 (~0.4 s)
// Sensor 3: ONLY triggers temp/humidity read (no motor)
// With ~1.5 ms kick and 10% run duty, 3 s wait.
// AHT20 reads at most once every 7 seconds.
//hello
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/i2c.h"

// ---------------- Pins ----------------
#define MOTION1_PIN      GPIO_NUM_4
#define MOTION2_PIN      GPIO_NUM_18
#define MOTION3_PIN      GPIO_NUM_25

#define PWM_A_PIN        GPIO_NUM_5
#define PWM_B_PIN        GPIO_NUM_19

// ---------------- PWM settings ----------------
#define PWM_FREQ_HZ      5000
#define PWM_RES_BITS     LEDC_TIMER_8_BIT   // 0..255
#define PWM_MAX_DUTY     255

static const float dutyRunPercent  = 10.0f;
static const float dutyKickPercent = 100.0f;

// ---------------- Timing ----------------
static const uint32_t RUN_TIME_MS       = 400;
static const uint32_t RUN_TIME_MS_SHORT = 250;
static const uint32_t WAIT_TIME_MS      = 3000;
static const uint32_t KICK_TIME_US      = 1500;

static const uint32_t AHT_MIN_INTERVAL_MS = 7000;

// ---------------- AHT20 (minimal driver via ESP-IDF I2C) ----------------
#define I2C_PORT         I2C_NUM_0
#define I2C_SDA_PIN      GPIO_NUM_21
#define I2C_SCL_PIN      GPIO_NUM_22
#define I2C_FREQ_HZ      100000
#define AHT20_ADDR       0x38

static const char *TAG = "trailcam";

// Time helpers
static inline uint32_t now_ms(void) { return (uint32_t)(esp_timer_get_time() / 1000); }
static inline int64_t  now_us(void) { return esp_timer_get_time(); }

// AHT cooldown
static uint32_t lastAhtReadMs = 0;

// State machine
static bool     sequenceActive = false;
static int      activeSensor   = 0;  // 0 none, 1 or 2
static int      phase          = 0;  // 0 idle, 1 first run, 2 wait, 3 second run
static uint32_t phaseStartMs   = 0;

// Kick tracking
static bool     kickActive     = false;
static int64_t  kickStartUs    = 0;
static uint32_t dutyRun        = 0;
static uint32_t dutyKick       = 0;

// Which PWM channel is active
static ledc_channel_t currentChan = LEDC_CHANNEL_0;

// Rising-edge baselines
static int lastS1 = 0, lastS2 = 0, lastS3 = 0;

// ---- I2C helpers ----
static esp_err_t i2c_write(uint8_t addr, const uint8_t *data, size_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, (uint8_t*)data, len, true);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return err;
}

static esp_err_t i2c_read(uint8_t addr, uint8_t *data, size_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_READ, true);
    if (len > 1) {
        i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return err;
}

// AHT20 init: 0xBE 0x08 0x00
static esp_err_t aht20_init(void)
{
    uint8_t init_cmd[3] = {0xBE, 0x08, 0x00};
    esp_err_t err = i2c_write(AHT20_ADDR, init_cmd, sizeof(init_cmd));
    vTaskDelay(pdMS_TO_TICKS(20));
    return err;
}

// Trigger: 0xAC 0x33 0x00, wait, read 6 bytes
static esp_err_t aht20_read(float *tempC, float *rh)
{
    uint8_t trig[3] = {0xAC, 0x33, 0x00};
    esp_err_t err = i2c_write(AHT20_ADDR, trig, sizeof(trig));
    if (err != ESP_OK) return err;

    vTaskDelay(pdMS_TO_TICKS(85));

    uint8_t data[6] = {0};
    err = i2c_read(AHT20_ADDR, data, sizeof(data));
    if (err != ESP_OK) return err;

    // data[0] status; bit7 busy (optional check)
    uint32_t hum20 = ((uint32_t)data[1] << 12) | ((uint32_t)data[2] << 4) | ((uint32_t)(data[3] >> 4) & 0x0F);
    uint32_t tmp20 = (((uint32_t)data[3] & 0x0F) << 16) | ((uint32_t)data[4] << 8) | (uint32_t)data[5];

    *rh    = ((float)hum20 * 100.0f) / 1048576.0f;        // 2^20
    *tempC = ((float)tmp20 * 200.0f) / 1048576.0f - 50.0f;

    return ESP_OK;
}

// ---- PWM helpers ----
static void pwm_write(ledc_channel_t ch, uint32_t duty)
{
    ledc_set_duty(LEDC_LOW_SPEED_MODE, ch, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, ch);
}

static void pwm_all_off(void)
{
    pwm_write(LEDC_CHANNEL_0, 0); // PWM_A_PIN
    pwm_write(LEDC_CHANNEL_1, 0); // PWM_B_PIN
}

// ---- Logging env, rate limited ----
static void logEnvForSensor(int sensor, uint32_t t_ms)
{
    if (t_ms - lastAhtReadMs < AHT_MIN_INTERVAL_MS) return;
    lastAhtReadMs = t_ms;

    float tC = 0.0f, rh = 0.0f;
    esp_err_t err = aht20_read(&tC, &rh);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "AHT20 read failed: %d", err);
        return;
    }

    float tF = tC * 9.0f / 5.0f + 32.0f;

    ESP_LOGI(TAG, "Sensor %d triggered.", sensor);
    ESP_LOGI(TAG, "Temperature: %.2f F", tF);
    ESP_LOGI(TAG, "Humidity: %.2f %% rH", rh);
}

// ---- Sequence helpers ----
static void startSecondRun(uint32_t t_ms)
{
    (void)t_ms;

    if (activeSensor == 1) {
        // Sensor 1: second is pin 19 -> channel 1
        currentChan = LEDC_CHANNEL_1;
    } else if (activeSensor == 2) {
        // Sensor 2: second is pin 5 -> channel 0
        currentChan = LEDC_CHANNEL_0;
    } else {
        return;
    }

    pwm_all_off();
    pwm_write(currentChan, dutyKick);

    kickActive  = true;
    kickStartUs = now_us();
}

static void startSequence(int sensor, uint32_t t_ms)
{
    sequenceActive = true;
    activeSensor   = sensor;
    phase          = 1;
    phaseStartMs   = t_ms;

    // One env reading right at start if allowed
    if (t_ms - lastAhtReadMs >= AHT_MIN_INTERVAL_MS) {
        logEnvForSensor(sensor, t_ms);
    }

    // First direction pin selection
    if (sensor == 1) {
        // Sensor 1: pin 5 first -> channel 0
        currentChan = LEDC_CHANNEL_0;
    } else {
        // Sensor 2: pin 19 first -> channel 1
        currentChan = LEDC_CHANNEL_1;
    }

    pwm_all_off();
    pwm_write(currentChan, dutyKick);

    kickActive  = true;
    kickStartUs = now_us();
}

// ---- Init ----
static void init_gpio_inputs(void)
{
    gpio_config_t io = {
        .pin_bit_mask = (1ULL<<MOTION1_PIN) | (1ULL<<MOTION2_PIN) | (1ULL<<MOTION3_PIN),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,   // change if PIR needs pull-down
        .intr_type    = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&io));

    lastS1 = gpio_get_level(MOTION1_PIN);
    lastS2 = gpio_get_level(MOTION2_PIN);
    lastS3 = gpio_get_level(MOTION3_PIN);
}

static void init_pwm(void)
{
    ledc_timer_config_t tcfg = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .timer_num        = LEDC_TIMER_0,
        .duty_resolution  = PWM_RES_BITS,
        .freq_hz          = PWM_FREQ_HZ,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&tcfg));

    ledc_channel_config_t ch0 = {
        .gpio_num   = PWM_A_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = LEDC_CHANNEL_0,
        .timer_sel  = LEDC_TIMER_0,
        .duty       = 0,
        .hpoint     = 0
    };
    ledc_channel_config_t ch1 = ch0;
    ch1.gpio_num = PWM_B_PIN;
    ch1.channel  = LEDC_CHANNEL_1;

    ESP_ERROR_CHECK(ledc_channel_config(&ch0));
    ESP_ERROR_CHECK(ledc_channel_config(&ch1));

    dutyRun  = (uint32_t)((dutyRunPercent  / 100.0f) * (float)PWM_MAX_DUTY);
    dutyKick = (uint32_t)((dutyKickPercent / 100.0f) * (float)PWM_MAX_DUTY);

    pwm_all_off();
}

static void init_i2c(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA_PIN,
        .scl_io_num = I2C_SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQ_HZ
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_PORT, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_PORT, conf.mode, 0, 0, 0));

    ESP_ERROR_CHECK(aht20_init());
}

// ---- Main task ----
static void trailcam_task(void *arg)
{
    (void)arg;

    ESP_LOGI(TAG, "Starting: 3-sensor PWM + AHT20 (ESP-IDF)");

    init_gpio_inputs();
    init_pwm();
    init_i2c();

    while (1) {
        uint32_t t_ms = now_ms();
        int64_t  t_us = now_us();

        // Kick timing
        if (kickActive && (t_us - kickStartUs >= (int64_t)KICK_TIME_US)) {
            kickActive = false;
            pwm_write(currentChan, dutyRun);
        }

        // Read inputs and detect rising edges
        int s1 = gpio_get_level(MOTION1_PIN);
        int s2 = gpio_get_level(MOTION2_PIN);
        int s3 = gpio_get_level(MOTION3_PIN);

        bool s1_rising = (s1 == 1 && lastS1 == 0);
        bool s2_rising = (s2 == 1 && lastS2 == 0);
        bool s3_rising = (s3 == 1 && lastS3 == 0);

        lastS1 = s1;
        lastS2 = s2;
        lastS3 = s3;

        // Sensor 3: env only
        if (s3_rising) {
            if (t_ms - lastAhtReadMs >= AHT_MIN_INTERVAL_MS) {
                logEnvForSensor(3, t_ms);
            }
        }

        // Start sequence for sensors 1 and 2
        if (!sequenceActive) {
            if (s1_rising) startSequence(1, t_ms);
            else if (s2_rising) startSequence(2, t_ms);
        }

        // State machine
        if (sequenceActive) {
            switch (phase) {
                case 1: { // first run
                    uint32_t runLimit = RUN_TIME_MS;

                    // Sensor 2 first move on pin 19 (channel 1) -> short run
                    if (activeSensor == 2 && currentChan == LEDC_CHANNEL_1) {
                        runLimit = RUN_TIME_MS_SHORT;
                    }

                    if (t_ms - phaseStartMs >= runLimit) {
                        pwm_all_off();
                        phase = 2;
                        phaseStartMs = t_ms;
                        kickActive = false;
                    }
                    break;
                }

                case 2: // wait
                    if (t_ms - phaseStartMs >= WAIT_TIME_MS) {
                        phase = 3;
                        phaseStartMs = t_ms;
                        startSecondRun(t_ms);
                    }
                    break;

                case 3: { // second run
                    uint32_t runLimit = RUN_TIME_MS;

                    // Sensor 1 second move on pin 19 (channel 1) -> short run
                    if (activeSensor == 1 && currentChan == LEDC_CHANNEL_1) {
                        runLimit = RUN_TIME_MS_SHORT;
                    }

                    if (t_ms - phaseStartMs >= runLimit) {
                        pwm_all_off();
                        sequenceActive = false;
                        activeSensor = 0;
                        phase = 0;
                        kickActive = false;
                    }
                    break;
                }

                default:
                    pwm_all_off();
                    sequenceActive = false;
                    activeSensor = 0;
                    phase = 0;
                    kickActive = false;
                    break;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ---- Entry point ----
// Because main/CMakeLists.txt only builds THIS file, app_main lives here.
void app_main(void)
{
    xTaskCreate(trailcam_task, "trailcam_task", 4096, NULL, 5, NULL);
}
