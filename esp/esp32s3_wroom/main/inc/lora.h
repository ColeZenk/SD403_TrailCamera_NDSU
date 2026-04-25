#pragma once

#include "config.h"
#include "esp_err.h"

esp_err_t lora_init(void);
void lora_receive_task(void *arg);

#ifdef TEST_MODE_LORA_BENCH
void lora_bench_task(void *arg);
#endif
