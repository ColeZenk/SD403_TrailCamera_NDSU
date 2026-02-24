#pragma once

#include <stdint.h>
#include "esp_err.h"

typedef enum {
    PIR1 = 1,
    PIR2 = 2,
    PIR3 = 3
} pir_id_t;

esp_err_t pir_init(int pir1_gpio, int pir2_gpio, int pir3_gpio);

/**
 * Returns 0 or 1 for the given PIR input.
 */
int pir_read(pir_id_t pir);