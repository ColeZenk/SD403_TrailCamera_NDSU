# S3 LoRa Bench Fix

## lora_bench.c
- [X] Fix `LORA_ADDRESS_RECEIVERRECEIVER` → `LORA_ADDRESS` in `bench_module_init()`
- [X] Delete `send_and_echo()`, `run_trial()`, `run_burst()`
- [X] Replace `lora_bench_task()` body with echo loop: init → `readline()` → parse `+RCV=` → 15ms delay → `AT+SEND=<src_addr>,<len>,<data>`
- [X] Update file header comment

## main.c
- [X] Fix `#include "lora.h"` (deleted) — needs `lora_bench.h` in bench mode
