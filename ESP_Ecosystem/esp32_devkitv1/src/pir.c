#include "pir.h"

#include "driver/gpio.h"

static int s_pir1 = -1;
static int s_pir2 = -1;
static int s_pir3 = -1;

esp_err_t pir_init(int pir1_gpio, int pir2_gpio, int pir3_gpio)
{
    s_pir1 = pir1_gpio;
    s_pir2 = pir2_gpio;
    s_pir3 = pir3_gpio;

    gpio_config_t io = {
        .pin_bit_mask = (1ULL << s_pir1) | (1ULL << s_pir2) | (1ULL << s_pir3),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    return gpio_config(&io);
}

int pir_read(pir_id_t pir)
{
    int pin = -1;
    if (pir == PIR1) pin = s_pir1;
    else if (pir == PIR2) pin = s_pir2;
    else if (pir == PIR3) pin = s_pir3;

    if (pin < 0) return 0;
    return gpio_get_level((gpio_num_t)pin);
}
