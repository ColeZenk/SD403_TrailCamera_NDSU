/*
 * ov2640_config.h
 * OV2640 sensor configuration — config-by-struct pattern
 *
 * Wraps the Espressif sensor_t function pointer mess into a clean
 * struct-based interface. Configure once, apply once, never touch
 * the raw API again.
 */

#ifndef OV2640_CONFIG_H
#define OV2640_CONFIG_H

#include "esp_err.h"
#include "esp_camera.h"

typedef struct {
    int aec_enable;        /* 0 = manual exposure,  1 = auto */
    int agc_enable;        /* 0 = manual gain,      1 = auto */
    int awb_enable;        /* 0 = manual WB,        1 = auto */
    int aec2_enable;       /* 0 = DSP AEC off,      1 = on   */
    int gain;              /* manual gain level (0–30)        */
    int exposure;          /* manual exposure (0–1200)        */
    gainceiling_t gain_ceiling;  /* max gain multiplier       */
    int target_mean;       /* auto-tune target brightness (0 = skip, use exposure as-is) */
    int tune_tolerance;    /* auto-tune stops when |mean - target| < tolerance */
} ov2640_config_t;

/* Locked defaults for diff-based compression pipeline.
 * Gain floored, exposure auto-tuned to mean ~110. */
#define OV2640_LOCKED_DEFAULTS {       \
    .aec_enable     = 0,               \
    .agc_enable     = 0,               \
    .awb_enable     = 0,               \
    .aec2_enable    = 0,               \
    .gain           = 0,               \
    .exposure       = 50,              \
    .gain_ceiling   = GAINCEILING_2X,  \
    .target_mean    = 110,             \
    .tune_tolerance = 5,               \
}

/**
 * Apply sensor configuration.
 * Disables auto controls, sets static parameters, and optionally
 * auto-tunes exposure to hit target_mean brightness.
 *
 * Call after esp_camera_init().
 */
esp_err_t ov2640_apply_config(const ov2640_config_t *cfg);

/**
 * Read back current AEC value from sensor.
 */
int ov2640_get_aec_value(void);

#endif /* OV2640_CONFIG_H */
