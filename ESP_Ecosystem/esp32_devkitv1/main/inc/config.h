#ifndef CONFIG_H
#define CONFIG_H

#include "esp_err.h"
#include "driver/spi_common.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"

/*******************************************************************************
 * Build Configuration & Test Modes
 ******************************************************************************/

/**
 * Test mode configuration - similar to WarblingWire pattern
 * Uncomment ONE of these to enable specific test modes
 */
/* #define TEST_MODE_FPGA_PATTERNS */
/* #define TEST_MODE_LORA_LOOPBACK */
/* #define TEST_MODE_CAMERA_INJECT */
/* #define TEST_MODE_FPGA_GPIO */
#define TEST_MODE_LORA_BENCH

// Verify only one test mode is active
#if defined(TEST_MODE_FPGA_PATTERNS) && defined(TEST_MODE_LORA_LOOPBACK)
    #error "Multiple test modes enabled - choose only one"
#endif
#if defined(TEST_MODE_LORA_BENCH) && (defined(TEST_MODE_FPGA_PATTERNS) || defined(TEST_MODE_LORA_LOOPBACK))
    #error "LORA_BENCH is standalone — disable other test modes"
#endif

// Performance optimization level
#define OPTIMIZE_FOR_SPEED      1  // Set to 0 for size optimization

/*******************************************************************************
 * System Configuration - Computed Constants
 ******************************************************************************/

// Memory allocation
#define IMAGE_MAX_SIZE_BYTES        (1024 * 1024)
#define IMAGE_QUEUE_LENGTH          3
#define SPI_BUFFER_SIZE             4096
#define UART_BUFFER_SIZE            2048

// Computed timeout values (compile-time calculations)
#define MS_TO_TICKS(ms)             pdMS_TO_TICKS(ms)
#define SECONDS_TO_TICKS(s)         pdMS_TO_TICKS((s) * 1000)

// Task priorities
#define TASK_PRIORITY_HIGH          5
#define TASK_PRIORITY_MEDIUM        4
#define TASK_PRIORITY_LOW           3

// Task stack sizes
#define STACK_SIZE_SMALL            2048
#define STACK_SIZE_MEDIUM           4096
#define STACK_SIZE_LARGE            8192

/*******************************************************************************
 * Camera SPI Slave Configuration (receiving from ESP32-CAM)
 ******************************************************************************/

#define CAM_SPI_HOST                SPI2_HOST
#define CAM_SPI_DMA_CHAN            SPI_DMA_CH_AUTO

#define CAM_PIN_MOSI                GPIO_NUM_23
#define CAM_PIN_MISO                GPIO_NUM_19
#define CAM_PIN_SCLK                GPIO_NUM_18
#define CAM_PIN_CS                  GPIO_NUM_5

#define CAM_SPI_MODE                0
#define CAM_SPI_QUEUE_SIZE          3
#define CAM_TRANSFER_TIMEOUT_MS     5000
#define CAM_TRANSFER_TIMEOUT_TICKS  MS_TO_TICKS(CAM_TRANSFER_TIMEOUT_MS)

// Camera reception strategy
#define CAM_USE_ZERO_COPY           1  // DMA directly to final buffer when possible

/*******************************************************************************
 * FPGA SPI Master Configuration (sending to Tang Nano 9K)
 ******************************************************************************/

#define FPGA_SPI_HOST               SPI3_HOST
#define FPGA_SPI_DMA_CHAN           SPI_DMA_CH_AUTO

#define FPGA_PIN_MOSI               GPIO_NUM_32
#define FPGA_PIN_MISO               GPIO_NUM_33
#define FPGA_PIN_SCLK               GPIO_NUM_25
#define FPGA_PIN_CS                 GPIO_NUM_26

#define FPGA_SPI_CLOCK_MHZ          9
#define FPGA_SPI_CLOCK_HZ           (FPGA_SPI_CLOCK_MHZ * 1000000)
#define FPGA_SPI_MODE               0
#define FPGA_SPI_QUEUE_SIZE         3
#define FPGA_MAX_TRANSFER_SIZE      32768  /* full BRAM — single CS transaction, no mid-frame resets */

/*******************************************************************************
 * FPGA I2C GPIO Expander Configuration (PCA9534-compatible at 0x27)
 ******************************************************************************/

#define FPGA_I2C_PORT               I2C_NUM_0
#define FPGA_I2C_SDA_PIN            GPIO_NUM_21
#define FPGA_I2C_SCL_PIN            GPIO_NUM_22
#define FPGA_I2C_CLK_HZ             100000      /* 100 kHz — safe for LVCMOS18 */
#define FPGA_I2C_ADDR               0x27
#define FPGA_I2C_TIMEOUT_MS         100

/* Register map (PCA9534-compatible) */
#define FPGA_GPIO_REG_OUT           0x00
#define FPGA_GPIO_REG_IN            0x01
#define FPGA_GPIO_REG_DIR           0x02
#define FPGA_GPIO_REG_INV           0x03

/* Boot config: bits[7:4]=inputs(unused), bits[3:0]=outputs(steppers) */
#define FPGA_GPIO_DIR_BOOT          0xF0
/* Invert bits[2:0] so button reads 1=pressed (active-low with pull-up) */
#define FPGA_GPIO_INV_BOOT          0x07

/* Button bit positions in INPUT_PORT after invert applied */
#define FPGA_BTN_L_BIT              0   /* gpio_in[0] = button_L, pin 84 */
#define FPGA_BTN_R_BIT              1   /* gpio_in[1] = button_R, pin 83 */
#define FPGA_BTN_S_BIT              2   /* gpio_in[2] = button_S, pin 85 */

/*******************************************************************************
 * LoRa UART Configuration
 ******************************************************************************/

#define LORA_UART_NUM               UART_NUM_2
#define LORA_PIN_TX                 GPIO_NUM_17
#define LORA_PIN_RX                 GPIO_NUM_16
#define LORA_BAUD_RATE              115200

#define LORA_ADDRESS_RECEIVER       2
#define LORA_ADDRESS_SENDER         1
#define LORA_NETWORK_ID             6

#define LORA_AT_TIMEOUT_MS          500
#define LORA_AT_TIMEOUT_TICKS       MS_TO_TICKS(LORA_AT_TIMEOUT_MS)
#define LORA_INIT_DELAY_MS          300
#define LORA_INIT_DELAY_TICKS       MS_TO_TICKS(LORA_INIT_DELAY_MS)

// LoRa polling configuration
#define LORA_POLL_INTERVAL_MS       100
#define LORA_POLL_INTERVAL_TICKS    MS_TO_TICKS(LORA_POLL_INTERVAL_MS)
#define LORA_HEARTBEAT_COUNT        100  // Log every N polls

/*******************************************************************************
 * LoRa Bench Test Configuration (TEST_MODE_LORA_BENCH)
 *
 * MUST match the values in ESP_Ecosystem/esp32s3_wroom/main/inc/config.h.
 *
 * RYLR998 AT+PARAMETER: AT+PARAMETER=<SF>,<BW>,<CR>,<Preamble>
 *   BW: 7=125 kHz  8=250 kHz  9=500 kHz
 *   SF: 7-12  (lower = faster, less range)
 *
 * Start with SF=9/BW=7 (defaults) to confirm link, then switch to SF=7/BW=9
 * to test 6-FPS throughput target.  Both boards must be reflashed each time.
 ******************************************************************************/

#ifdef TEST_MODE_LORA_BENCH
    /* RF parameters — set identically on S3 side */
    #define BENCH_SF                7       /* spreading factor (7-12) */
    #define BENCH_BW                9       /* 7=125kHz 8=250kHz 9=500kHz */
    #define BENCH_CR                1       /* coding rate: 1=4/5 */
    #define BENCH_PREAMBLE          12

    /* Destination address (S3 receiver).
     * Use 0 = factory default (AT+ADDRESS set commands fail with ERR=15 until resolved) */
    #define BENCH_DEST_ADDR         1       /* S3 module address */

    /* Packet parameters */
    #define BENCH_PKT_SIZE          115     /* target SDD502 compressed frame */
    #define BENCH_N                 50      /* packets per trial */
    #define BENCH_TIMEOUT_MS        2000    /* per-packet echo timeout */
    #define BENCH_INTER_PACKET_MS   10      /* gap between sends (ms) */

    /* Burst test */
    #define BENCH_BURST_SECS        10      /* duration of sustained burst */
    #define BENCH_BURST_TIMEOUT_MS  500     /* shorter timeout for burst */

    #warning "LoRa bench mode — DevKitV1 is TX master, S3 must be echo slave"
#endif

/*******************************************************************************
 * Test/Debug Configuration
 ******************************************************************************/

#define TEST_BUTTON_PIN             GPIO_NUM_0
#define TEST_LED_PIN                GPIO_NUM_2
#define TEST_IMAGE_SIZE             32768  // FPGA BRAM size (Tang Nano 9K)

// Test pattern configuration
#ifdef TEST_MODE_FPGA_PATTERNS
    #define TEST_PATTERN_COUNT      4
    #define TEST_PATTERN_HOLD_MS    2000
    #warning "FPGA test pattern mode enabled"
#endif

#ifdef TEST_MODE_LORA_LOOPBACK
    #warning "LoRa loopback test mode enabled"
#endif

#ifdef TEST_MODE_CAMERA_INJECT
    #define TEST_INJECT_INTERVAL_MS 5000
    #warning "Camera injection test mode enabled"
#endif

/*******************************************************************************
 * Performance Attributes - Compiler Hints
 ******************************************************************************/

// Hot path optimization (functions called frequently)
#if OPTIMIZE_FOR_SPEED
    #define HOT_FUNCTION    __attribute__((hot))
    #define COLD_FUNCTION   __attribute__((cold))
    #define ALWAYS_INLINE   __attribute__((always_inline)) static inline
    #define LIKELY(x)       __builtin_expect(!!(x), 1)
    #define UNLIKELY(x)     __builtin_expect(!!(x), 0)
#else
    #define HOT_FUNCTION
    #define COLD_FUNCTION
    #define ALWAYS_INLINE   static inline
    #define LIKELY(x)       (x)
    #define UNLIKELY(x)     (x)
#endif

// ISR functions (critical timing)
#define ISR_FUNCTION        __attribute__((interrupt))

// Alignment for DMA buffers
#define DMA_ALIGNED         __attribute__((aligned(4)))

/*******************************************************************************
 * Protocol Definitions
 ******************************************************************************/

#define IMAGE_HEADER_MAGIC          0xCAFEBEEF
#define IMAGE_HEADER_SIZE           28

// Image header structure (packed for wire protocol)
typedef struct {
    uint32_t magic;
    uint32_t size;
    uint32_t checksum;
    uint32_t timestamp;
    uint16_t width;
    uint16_t height;
    uint8_t  format;
    uint8_t  reserved[7];
} __attribute__((packed)) image_header_t;

_Static_assert(sizeof(image_header_t) == IMAGE_HEADER_SIZE,
               "Image header must be 24 bytes");

// Image data container
typedef struct {
    uint8_t *buffer;
    size_t size;
    image_header_t header;
} image_data_t;

/*******************************************************************************
 * Validation Macros - Compile-Time Checks
 ******************************************************************************/

// Ensure buffer sizes are reasonable
_Static_assert(SPI_BUFFER_SIZE >= 1024, "SPI buffer too small");
_Static_assert(SPI_BUFFER_SIZE <= 8192, "SPI buffer too large for DMA");
_Static_assert((SPI_BUFFER_SIZE % 4) == 0, "SPI buffer must be 4-byte aligned");

// Ensure queue depth is reasonable
_Static_assert(IMAGE_QUEUE_LENGTH >= 1, "Need at least 1 queue slot");
_Static_assert(IMAGE_QUEUE_LENGTH <= 10, "Queue too deep - memory waste");

#endif // CONFIG_H
