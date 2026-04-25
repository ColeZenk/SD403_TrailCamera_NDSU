#pragma once

#include <stddef.h>
#include <stdint.h>
#include "config.h"
#include "esp_err.h"

esp_err_t lora_init(void);
esp_err_t lora_send_packet(const uint8_t *data, size_t len);

#ifdef TEST_MODE_LORA_BENCH
void lora_bench_task(void *arg);
#endif
