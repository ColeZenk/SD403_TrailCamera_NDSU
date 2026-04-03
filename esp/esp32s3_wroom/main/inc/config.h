#pragma once

/*******************************************************************************
 * WiFi AP Configuration
 ******************************************************************************/
#define WIFI_AP_SSID        "TrailCamera"
#define WIFI_AP_PASS        "wildlife1"     /* min 8 chars for WPA2 */
#define WIFI_AP_CHANNEL     6
#define WIFI_AP_MAX_CONN    4

/*******************************************************************************
 * WebSocket Server
 ******************************************************************************/
#define WS_PORT             80
#define WS_PATH             "/stream"
#define WS_MAX_CLIENTS      4

/*******************************************************************************
 * Bench Test Mode
 *
 * Define TEST_MODE_LORA_BENCH on BOTH boards to run the link bench.
 * BENCH_SF and BENCH_BW MUST match the DevKitV1 config.h values.
 * In bench mode: WiFi/WebSocket disabled; LoRa packets are echoed back.
 ******************************************************************************/
#define TEST_MODE_LORA_BENCH

#ifdef TEST_MODE_LORA_BENCH
    #define BENCH_SF            7       /* must match DevKitV1 */
    #define BENCH_BW            9       /* 7=125kHz 8=250kHz 9=500kHz */
    #define BENCH_CR            1
    #define BENCH_PREAMBLE      12
    #warning "LoRa bench mode — S3 is echo slave, WiFi/WebSocket disabled"
#endif

/*******************************************************************************
 * LoRa UART
 ******************************************************************************/
#define LORA_UART_NUM       UART_NUM_1
#define LORA_PIN_TX         GPIO_NUM_17
#define LORA_PIN_RX         GPIO_NUM_18
#define LORA_BAUD_RATE           115200
#define LORA_BUF_SIZE       1024
#define LORA_ADDRESS_RECEIVER        1               /* this receiver */
#define LORA_NETWORK_ID     6
#define LORA_AT_TIMEOUT_MS           500
#define LORA_INIT_DELAY_MS           300
#define LORA_MAX_PAYLOAD             240             /* RYLR998 max payload */

/*******************************************************************************
 * Image / Frame Constants  (must match camera unit)
 ******************************************************************************/
#define FRAME_W             640
#define FRAME_H             480
#define FRAME_BYTES         (FRAME_W * FRAME_H)   /* 307,200 — grayscale */
#define BLOCK_SIZE          8
#define BLOCKS_W            (FRAME_W / BLOCK_SIZE) /* 80 */
#define BLOCKS_H            (FRAME_H / BLOCK_SIZE) /* 60 */
#define NUM_POSITIONS       3

/*******************************************************************************
 * Gaussian Reconstruction
 ******************************************************************************/
#define GAUSS_SIGMA         10.0f           /* ~1.25 block widths, insensitive */
#define GAUSS_RADIUS_BLOCKS 4               /* only consider blocks within 4 blocks */

/*******************************************************************************
 * Task Config
 ******************************************************************************/
#define STACK_LORA          16384
#define STACK_WS            8192
#define PRIO_LORA           5
#define PRIO_WS             4
#define CORE_LORA           0
#define CORE_WS             1

/*******************************************************************************
 * System Configuration - Computed Constants
 ******************************************************************************/

// Memory allocation
#define LW_BYTES                    (10 << 1)              // (10 * 2)
#define IMAGE_MAX_SIZE_BYTES        (1 << LW_BYTES)        // (1024 * 1024)
#define IMAGE_QUEUE_LENGTH          3
#define UART_BUFFER_SIZE            2048

// Computed timeout values (compile-time calculations)
#define MS_TO_TICKS(ms)             pdMS_TO_TICKS(ms)
#define SECONDS_TO_TICKS(s)         pdMS_TO_TICKS((s) * 1000)

// Task priorities
#define TASK_PRIORITY_HIGH          5
#define TASK_PRIORITY_MEDIUM        4
#define TASK_PRIORITY_LOW           3

#define STACK_SIZE_BIT_WIDTH_S      11
#define STACK_SIZE_BIT_WIDTH_M      12
#define STACK_SIZE_BIT_WIDTH_L      13

// Task stack sizes
#define STACK_SIZE_SMALL            (1 << STACK_SIZE_BIT_WIDTH_S)        // 2048
#define STACK_SIZE_MEDIUM           (1 << STACK_SIZE_BIT_WIDTH_M)        // 4096
#define STACK_SIZE_LARGE            (1 << STACK_SIZE_BIT_WIDTH_L)        // 8192
