/**
 * motor.c — 28BYJ-48 stepper motor driver (ULN2003)
 *
 * Full-step 2-coil sequence with trapezoidal ramp profile.
 */

#include "peripherals/motor.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

/*******************************************************************************
 * Step Table — 2-coil full-step (AB→BC→CD→DA)
 ******************************************************************************/

static const uint8_t STEP[4][4] = {
    {1,1,0,0}, {0,1,1,0}, {0,0,1,1}, {1,0,0,1},
};

/*******************************************************************************
 * Ramp Defaults
 ******************************************************************************/

#define RAMP_START_MS  14
#define RAMP_MIN_MS     2
#define RAMP_STEPS    600

/*******************************************************************************
 * Internals
 ******************************************************************************/

static void apply_phase(const motor_stepper_t *m)
{
    const uint8_t *s = STEP[m->phase & 3];
    int v[4] = { s[0], s[1], s[2], s[3] };

    gpio_set_level((gpio_num_t)m->in1_gpio, v[m->wire_map[0]]);
    gpio_set_level((gpio_num_t)m->in2_gpio, v[m->wire_map[1]]);
    gpio_set_level((gpio_num_t)m->in3_gpio, v[m->wire_map[2]]);
    gpio_set_level((gpio_num_t)m->in4_gpio, v[m->wire_map[3]]);
}

/*******************************************************************************
 * Public Interface
 ******************************************************************************/

esp_err_t motor_stepper_init(motor_stepper_t *m)
{
    if (!m) return ESP_ERR_INVALID_ARG;

    gpio_config_t io = {
        .pin_bit_mask = (1ULL << m->in1_gpio) | (1ULL << m->in2_gpio)
                      | (1ULL << m->in3_gpio) | (1ULL << m->in4_gpio),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };

    esp_err_t err = gpio_config(&io);
    if (err != ESP_OK) return err;

    apply_phase(m);
    return ESP_OK;
}

void motor_stepper_set_phase(motor_stepper_t *m, int phase)
{
    if (!m) return;
    m->phase = phase & 3;
    apply_phase(m);
}

void motor_stepper_release(motor_stepper_t *m)
{
    if (!m) return;
    gpio_set_level((gpio_num_t)m->in1_gpio, 0);
    gpio_set_level((gpio_num_t)m->in2_gpio, 0);
    gpio_set_level((gpio_num_t)m->in3_gpio, 0);
    gpio_set_level((gpio_num_t)m->in4_gpio, 0);
}

void motor_stepper_move_ramped(motor_stepper_t *m, int steps, int dir,
                               int start_ms, int min_ms, int ramp)
{
    if (!m || steps <= 0) return;

    dir = (dir < 0) ? -1 : 1;
    if (min_ms < 1)        min_ms = 1;
    if (start_ms < min_ms) start_ms = min_ms;
    if (ramp < 1)          ramp = 1;
    if (2 * ramp > steps)  ramp = steps / 2;

    for (int i = 0; i < steps; i++) {
        m->phase = (m->phase + dir) & 3;
        apply_phase(m);

        int d;
        if (i < ramp)
            d = start_ms - ((start_ms - min_ms) * i) / ramp;
        else if (i >= steps - ramp)
            d = start_ms - ((start_ms - min_ms) * (steps - 1 - i)) / ramp;
        else
            d = min_ms;

        vTaskDelay(pdMS_TO_TICKS(d));
    }
}

void motor_stepper_swing_and_release(motor_stepper_t *m, int steps,
                                     int dir, int hold_ms)
{
    if (!m || steps <= 0) return;

    motor_stepper_move_ramped(m, steps,  dir, RAMP_START_MS, RAMP_MIN_MS, RAMP_STEPS);
    vTaskDelay(pdMS_TO_TICKS(hold_ms));
    motor_stepper_move_ramped(m, steps, -dir, RAMP_START_MS, RAMP_MIN_MS, RAMP_STEPS);
    motor_stepper_release(m);
}

void motor_stepper_swing_cw(motor_stepper_t *m, int steps, int hold_ms)
{
    motor_stepper_swing_and_release(m, steps, +1, hold_ms);
}

void motor_stepper_swing_ccw(motor_stepper_t *m, int steps, int hold_ms)
{
    motor_stepper_swing_and_release(m, steps, -1, hold_ms);
}
