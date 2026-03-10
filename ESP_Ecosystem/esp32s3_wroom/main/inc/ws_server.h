#pragma once
#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"

esp_err_t ws_server_init(void);
esp_err_t ws_broadcast(uint8_t position, uint32_t seq,
                        const uint8_t *pixels, size_t len);
