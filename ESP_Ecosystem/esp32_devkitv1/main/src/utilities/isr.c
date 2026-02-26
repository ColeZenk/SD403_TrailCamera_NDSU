/**
 * @file isr.c
 * @brief Interrupt Service Routines - ESP32 DevKitV1
 * @author Cole Zenk
 *
 * Central ISR container. All interrupt logic lives here.
 * Handlers dispatch into their respective modules — no
 * business logic lives in this file.
 *
 * @scope
 * LoRa trigger semaphore (crosses module boundary)
 * FPGA test button semaphore
 * Camera timer alarm semaphore
 *
 * @note g_spi_complete_sem removed — fpga_spi now uses the ESP-IDF
 *       queue/get_trans_result pattern which handles DMA sync internally.
 *       ISR_OnSpiTransferComplete is no longer needed.
 */

/*************************************************************************
 INCLUDES
 *************************************************************************/
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "esp_log.h"

static const char *TAG = "ISR";

/*************************************************************************
 SHARED SEMAPHORES
 * Declared here, extern'd by consumers via isr_signals.h
 *************************************************************************/
SemaphoreHandle_t g_trigger_sem        = NULL;  // LoRa → capture pipeline
SemaphoreHandle_t g_button_sem         = NULL;  // Button → FPGA test task
SemaphoreHandle_t g_timer_capture_sem  = NULL;  // Timer → isolated capture task

/*************************************************************************
 INIT
 *************************************************************************/
esp_err_t isr_init(void)
{
    g_trigger_sem       = xSemaphoreCreateBinary();
    g_button_sem        = xSemaphoreCreateBinary();
    g_timer_capture_sem = xSemaphoreCreateBinary();

    if (!g_trigger_sem || !g_button_sem || !g_timer_capture_sem) {
        ESP_LOGE(TAG, "Failed to create semaphores");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "ISR module initialized");
    return ESP_OK;
}

/*************************************************************************
 HANDLERS
 *************************************************************************/

/**
 * @brief LoRa trigger received
 *
 * Called from lora_receive_task (task context, not an ISR).
 * Uses standard semaphore give — NOT the FromISR variant.
 */
void ISR_OnLoRaTrigger(void)
{
    xSemaphoreGive(g_trigger_sem);
}

/**
 * @brief FPGA test button pressed
 *
 * Called from GPIO ISR on button falling edge — true hardware interrupt.
 * Must use FromISR variants.
 */
void IRAM_ATTR ISR_OnButtonPress(void *arg)
{
    BaseType_t higher_priority_woken = pdFALSE;
    xSemaphoreGiveFromISR(g_button_sem, &higher_priority_woken);
    portYIELD_FROM_ISR(higher_priority_woken);
}

/**
 * @brief Camera timer alarm fired
 *
 * Called from GPTimer alarm callback — hardware interrupt context.
 * Must use FromISR variants.
 * Returns true if a context switch is needed (required by GPTimer API).
 */
bool IRAM_ATTR ISR_OnTimerAlarm(gptimer_handle_t timer,
                                 const gptimer_alarm_event_data_t *event_data,
                                 void *user_ctx)
{
    BaseType_t higher_priority_woken = pdFALSE;
    xSemaphoreGiveFromISR(g_timer_capture_sem, &higher_priority_woken);
    return higher_priority_woken == pdTRUE;
}
