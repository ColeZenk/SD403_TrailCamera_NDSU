// main/motor.h
#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int in1_gpio;
    int in2_gpio;
    int in3_gpio;
    int in4_gpio;

    // Coil order remap (fixes "squeak/vibrate no motion" without rewiring)
    // Default {0,1,2,3}. Common fix: {1,0,3,2}.
    int wire_map[4];

    // Current phase (0..3), managed internally
    int phase;
} motor_stepper_t;

esp_err_t motor_stepper_init(motor_stepper_t *m);

// Force a specific phase (0..3). Useful for known starting position.
void motor_stepper_set_phase(motor_stepper_t *m, int phase);

// De-energise all coils. Call before deep sleep to avoid wasting current.
void motor_stepper_release(motor_stepper_t *m);

// Low-level ramped move. direction = +1 (CW) or -1 (CCW).
void motor_stepper_move_ramped(motor_stepper_t *m,
                               int steps,
                               int direction,
                               int start_delay_ms,
                               int min_delay_ms,
                               int ramp_steps);

// ---------------------------------------------------------------------------
// High-level helpers (wrap the ramped move with sane defaults)
// ---------------------------------------------------------------------------

// Rotate `steps` in `direction`, hold for `hold_ms`, rotate back, then
// release the coils.  Returns after the full sequence is complete.
void motor_stepper_swing_and_release(motor_stepper_t *m,
                                     int steps,
                                     int direction,
                                     int hold_ms);

// Convenience: swing CW then back
void motor_stepper_swing_cw(motor_stepper_t *m, int steps, int hold_ms);

// Convenience: swing CCW then back
void motor_stepper_swing_ccw(motor_stepper_t *m, int steps, int hold_ms);

#ifdef __cplusplus
}
#endif