#ifndef LORA_UART_H
#define LORA_UART_H

#include "esp_err.h"
#include "freertos/FreeRTOS.h"

esp_err_t lora_init(void);
void      lora_send_trigger(void);
void      lora_receive_task(void *arg);

#endif /* LORA_UART_H */
