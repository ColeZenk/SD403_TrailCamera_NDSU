#pragma once

/**
 * lora_bench.h — LoRa link test bench (TX/master side on DevKitV1)
 *
 * Only compiled when TEST_MODE_LORA_BENCH is defined in config.h.
 * S3 must run with matching TEST_MODE_LORA_BENCH + same SF/BW settings.
 */

void lora_bench_task(void *arg);

