#pragma once

#include <stdint.h>
#include <stddef.h>

/* Provided by main.c (production mode only) */
#ifndef TEST_MODE_LORA_BENCH
uint8_t *reconstruct(const uint8_t *packet, size_t pkt_len, uint8_t *position_out);
extern uint32_t frame_seq;
#endif

/* Initialize LoRa UART, configure the RYLR998 module, and launch lora_task
 * pinned to CORE_LORA.  Call once from app_main. */
void lora_start(void);
