/*
 * ov2640_config.c
 * OV2640 sensor configuration implementation
 *
 * All interaction with the Espressif sensor_t API lives here.
 * The rest of the codebase uses ov2640_config_t structs.
 */

#include "ov2640_config.h"

#include <stdlib.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "OV2640";

/**************************************************************************************************
  File Scope
**************************************************************************************************/

static sensor_t *get_sensor(void)
{
    sensor_t *s = esp_camera_sensor_get();
    if (!s) {
        ESP_LOGE(TAG, "sensor handle is NULL — was esp_camera_init() called?");
    }
    return s;
}

static int compute_frame_mean(const camera_fb_t *fb)
{
    uint32_t sum = 0;
    for (size_t i = 0; i < fb->len; i++) {
        sum += fb->buf[i];
    }
    return (int)(sum / fb->len);
}

/**
 * Proportional exposure auto-tune.
 * Captures frames, measures mean brightness, adjusts aec_value
 * until within tolerance of target. Converges in 3–5 iterations.
 */
static esp_err_t tune_exposure(sensor_t *s, int target, int tolerance, int initial_aec)
{
    int aec = initial_aec;

    for (int i = 0; i < 20; i++) {
        s->set_aec_value(s, aec);
        vTaskDelay(pdMS_TO_TICKS(100));

        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) continue;

        int mean = compute_frame_mean(fb);
        esp_camera_fb_return(fb);

        ESP_LOGI(TAG, "tune [%2d]: aec=%-4d  mean=%-3d  target=%d", i, aec, mean, target);

        if (abs(mean - target) < tolerance) {
            ESP_LOGI(TAG, "converged: aec=%d  mean=%d", aec, mean);
            return ESP_OK;
        }

        aec = aec * target / (mean > 0 ? mean : 1);
        if (aec < 1)    aec = 1;
        if (aec > 1200) aec = 1200;
    }

    ESP_LOGW(TAG, "auto-tune did not converge (aec=%d)", aec);
    return ESP_OK;  /* still usable, just not perfect */
}

/**************************************************************************************************
  Global Interface
**************************************************************************************************/

esp_err_t ov2640_apply_config(const ov2640_config_t *cfg)
{
    sensor_t *s = get_sensor();
    if (!s) return ESP_FAIL;

    /* Disable auto controls first — order matters on OV2640 */
    s->set_whitebal(s,      cfg->awb_enable);
    s->set_aec2(s,          cfg->aec2_enable);
    s->set_gain_ctrl(s,     cfg->agc_enable);
    s->set_exposure_ctrl(s, cfg->aec_enable);

    /* Static parameters */
    s->set_agc_gain(s,      cfg->gain);
    s->set_gainceiling(s,   cfg->gain_ceiling);

    /* Exposure: either fixed or auto-tuned */
    if (cfg->target_mean > 0) {
        tune_exposure(s, cfg->target_mean, cfg->tune_tolerance, cfg->exposure);
    } else {
        s->set_aec_value(s, cfg->exposure);
    }

    ESP_LOGI(TAG, "config applied: gain=%d  agc=%d  aec=%d  awb=%d",
             cfg->gain, cfg->agc_enable, cfg->aec_enable, cfg->awb_enable);

    return ESP_OK;
}

int ov2640_get_aec_value(void)
{
    sensor_t *s = get_sensor();
    if (!s) return -1;
    return s->status.aec_value;
}
