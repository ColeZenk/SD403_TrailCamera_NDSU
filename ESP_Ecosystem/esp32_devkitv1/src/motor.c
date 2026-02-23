// main/motor.c
#include "motor.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

static inline void apply_abcd(const motor_stepper_t *m, int a, int b, int c, int d) {
    int v[4] = { a, b, c, d };
    gpio_set_level((gpio_num_t)m->in1_gpio, v[m->wire_map[0]]);
    gpio_set_level((gpio_num_t)m->in2_gpio, v[m->wire_map[1]]);
    gpio_set_level((gpio_num_t)m->in3_gpio, v[m->wire_map[2]]);
    gpio_set_level((gpio_num_t)m->in4_gpio, v[m->wire_map[3]]);
}

// 2-coil FULL-STEP sequence (more torque, less stall / squeak)
// AB -> BC -> CD -> DA
static const uint8_t FS2[4][4] = {
    {1,1,0,0},
    {0,1,1,0},
    {0,0,1,1},
    {1,0,0,1},
};

// Default ramp profile — tune here if needed
#define MOTOR_DEFAULT_START_DELAY_MS  14
#define MOTOR_DEFAULT_MIN_DELAY_MS     2
#define MOTOR_DEFAULT_RAMP_STEPS     600

static inline void set_phase_internal(motor_stepper_t *m, int phase) {
    m->phase = phase & 3;
    apply_abcd(m,
               FS2[m->phase][0],
               FS2[m->phase][1],
               FS2[m->phase][2],
               FS2[m->phase][3]);
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

esp_err_t motor_stepper_init(motor_stepper_t *m) {
    if (!m) return ESP_ERR_INVALID_ARG;

    gpio_config_t io = {
        .pin_bit_mask =
            (1ULL << m->in1_gpio) |
            (1ULL << m->in2_gpio) |
            (1ULL << m->in3_gpio) |
            (1ULL << m->in4_gpio),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    esp_err_t err = gpio_config(&io);
    if (err != ESP_OK) return err;

    set_phase_internal(m, m->phase);
    return ESP_OK;
}

void motor_stepper_set_phase(motor_stepper_t *m, int phase) {
    if (!m) return;
    set_phase_internal(m, phase);
}

void motor_stepper_release(motor_stepper_t *m) {
    if (!m) return;
    // Drive all coil pins LOW so no current flows during sleep
    gpio_set_level((gpio_num_t)m->in1_gpio, 0);
    gpio_set_level((gpio_num_t)m->in2_gpio, 0);
    gpio_set_level((gpio_num_t)m->in3_gpio, 0);
    gpio_set_level((gpio_num_t)m->in4_gpio, 0);
}

void motor_stepper_move_ramped(motor_stepper_t *m,
                               int steps,
                               int direction,
                               int start_delay_ms,
                               int min_delay_ms,
                               int ramp_steps) {
    if (!m || steps <= 0) return;

    direction = (direction < 0) ? -1 : 1;

    if (min_delay_ms < 1)              min_delay_ms = 1;
    if (start_delay_ms < min_delay_ms) start_delay_ms = min_delay_ms;
    if (ramp_steps < 1)                ramp_steps = 1;
    if (2 * ramp_steps > steps)        ramp_steps = steps / 2;

    for (int i = 0; i < steps; i++) {
        // direction=-1: (phase-1)&3 wraps correctly in two's-complement C
        m->phase = (m->phase + direction) & 3;
        set_phase_internal(m, m->phase);

        int d;
        if (i < ramp_steps) {
            d = start_delay_ms - ((start_delay_ms - min_delay_ms) * i) / ramp_steps;
        } else if (i >= (steps - ramp_steps)) {
            int j = steps - 1 - i;
            d = start_delay_ms - ((start_delay_ms - min_delay_ms) * j) / ramp_steps;
        } else {
            d = min_delay_ms;
        }

        vTaskDelay(pdMS_TO_TICKS(d));
    }
    // Coils remain ON after this call for holding torque.
    // Call motor_stepper_release() when holding is no longer needed.
}

void motor_stepper_swing_and_release(motor_stepper_t *m,
                                     int steps,
                                     int direction,
                                     int hold_ms) {
    if (!m || steps <= 0) return;

    // Out
    motor_stepper_move_ramped(m, steps, direction,
                              MOTOR_DEFAULT_START_DELAY_MS,
                              MOTOR_DEFAULT_MIN_DELAY_MS,
                              MOTOR_DEFAULT_RAMP_STEPS);

    // Hold at target position
    vTaskDelay(pdMS_TO_TICKS(hold_ms));

    // Return
    motor_stepper_move_ramped(m, steps, -direction,
                              MOTOR_DEFAULT_START_DELAY_MS,
                              MOTOR_DEFAULT_MIN_DELAY_MS,
                              MOTOR_DEFAULT_RAMP_STEPS);

    // Release coils — no current wasted during sleep
    motor_stepper_release(m);
}

void motor_stepper_swing_cw(motor_stepper_t *m, int steps, int hold_ms) {
    motor_stepper_swing_and_release(m, steps, +1, hold_ms);
}

void motor_stepper_swing_ccw(motor_stepper_t *m, int steps, int hold_ms) {
    motor_stepper_swing_and_release(m, steps, -1, hold_ms);
}