/**
 * isr.c — Interrupt service routines
 *
 * Central ISR container. Handlers dispatch via semaphores.
 * No business logic here — just signal forwarding.
 */

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "esp_log.h"

static const char *TAG = "ISR";

/*******************************************************************************
 * Shared Semaphores (extern'd via isr_signals.h)
 ******************************************************************************/

SemaphoreHandle_t g_trigger_sem       = NULL;   /* LoRa → capture */
SemaphoreHandle_t g_button_sem        = NULL;   /* button → FPGA test */
SemaphoreHandle_t g_timer_capture_sem = NULL;   /* timer → capture */

/*******************************************************************************
 * Init
 ******************************************************************************/

esp_err_t isr_init(void)
{
    g_trigger_sem       = xSemaphoreCreateBinary();
    g_button_sem        = xSemaphoreCreateBinary();
    g_timer_capture_sem = xSemaphoreCreateBinary();

    if (!g_trigger_sem || !g_button_sem || !g_timer_capture_sem) {
        ESP_LOGE(TAG, "semaphore creation failed");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "initialized");
    return ESP_OK;
}

/*******************************************************************************
 * Handlers
 ******************************************************************************/

/* Task context — called from lora_receive_task */
void ISR_OnLoRaTrigger(void)
{
    xSemaphoreGive(g_trigger_sem);
}

/* True ISR — GPIO falling edge on test button */
void IRAM_ATTR ISR_OnButtonPress(void *arg)
{
    BaseType_t woken = pdFALSE;
    xSemaphoreGiveFromISR(g_button_sem, &woken);
    portYIELD_FROM_ISR(woken);
}

/* True ISR — GPTimer alarm callback */
bool IRAM_ATTR ISR_OnTimerAlarm(gptimer_handle_t timer,
                                 const gptimer_alarm_event_data_t *event_data,
                                 void *user_ctx)
{
    BaseType_t woken = pdFALSE;
    xSemaphoreGiveFromISR(g_timer_capture_sem, &woken);
    return woken == pdTRUE;
}
