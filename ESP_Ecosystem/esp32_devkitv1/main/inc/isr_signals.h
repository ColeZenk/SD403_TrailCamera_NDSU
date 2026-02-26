/**
 * @file isr_signals.h
 * @brief ISR Shared Signals - Include in any file that needs ISR access
 * @author Cole Zenk
 */

#ifndef ISR_SIGNALS_H
#define ISR_SIGNALS_H

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "driver/spi_master.h"
#include "driver/gptimer.h"
#include "esp_err.h"

/*************************************************************************
 SHARED SEMAPHORES
 *************************************************************************/
extern SemaphoreHandle_t g_trigger_sem;
extern SemaphoreHandle_t g_button_sem;
extern SemaphoreHandle_t g_spi_complete_sem;
extern SemaphoreHandle_t g_timer_capture_sem;

/*************************************************************************
 HANDLERS
 *************************************************************************/
esp_err_t isr_init(void);

void     ISR_OnLoRaTrigger(void);
void     ISR_OnButtonPress(void *arg);
void     ISR_OnSpiTransferComplete(spi_transaction_t *trans);
bool     ISR_OnTimerAlarm(gptimer_handle_t timer,
                          const gptimer_alarm_event_data_t *event_data,
                          void *user_ctx);

#endif /* ISR_SIGNALS_H */
