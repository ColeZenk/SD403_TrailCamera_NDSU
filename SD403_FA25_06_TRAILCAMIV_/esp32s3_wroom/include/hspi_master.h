#ifndef HSPI_MASTER_H
#define HSPI_MASTER_H
#include "esp_err.h"
#define HSPI_MOSI_PIN 13
#define HSPI_MISO_PIN 12
#define HSPI_SCLK_PIN 14
#define HSPI_CS_PIN 15
esp_err_t hspi_master_init(void);
#endif
