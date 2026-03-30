/**
 * pir.c — PIR motion sensor GPIO interface
 */

#include "peripherals/pir.h"
#include "driver/gpio.h"

static int pins[3] = { -1, -1, -1 };

esp_err_t pir_init(int pir1, int pir2, int pir3)
{
    pins[0] = pir1;
    pins[1] = pir2;
    pins[2] = pir3;

    gpio_config_t io = {
        .pin_bit_mask = (1ULL << pir1) | (1ULL << pir2) | (1ULL << pir3),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };

    return gpio_config(&io);
}

bool pir_read(pir_id_t pir)
{
    int idx = (int)pir;
    if (idx < 0 || idx > 2 || pins[idx] < 0) return 0;
    return gpio_get_level((gpio_num_t)pins[idx]);
}
