#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include <stdbool.h>

typedef enum {
    PIR1 = 0,
    PIR2 = 1,
    PIR3 = 2
} pir_id_t;

esp_err_t pir_init(int pir1_gpio, int pir2_gpio, int pir3_gpio);

/**
 * Returns 0 or 1 for the given PIR input.
 */
bool pir_read(pir_id_t pir);
