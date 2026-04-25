#pragma once

#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

esp_err_t alloc_reference_frames(void);

uint8_t *reconstruct(const uint8_t *packet, size_t pkt_len,
                     uint8_t *position_out);
