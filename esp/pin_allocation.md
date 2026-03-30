# GPIO Pin Allocation Report

Generated: $(date)

## ESP32-CAM Pins

#define PIN_NUM_MOSI       13
#define PIN_NUM_MISO       12
#define PIN_NUM_CLK        14
#define PIN_NUM_CS         15

## ESP32-S3 WROOM Pins
    #define LORA_TX_PIN     GPIO_NUM_17
    #define LORA_RX_PIN     GPIO_NUM_18
    #define BOOT_BUTTON     GPIO_NUM_9  // Boot button on S3 DevKit

## ESP32 DevKitV1 Pins
    #define LORA_TX                     GPIO_NUM_17
    #define LORA_RX                     GPIO_NUM_16
    #define LED_PIN                     GPIO_NUM_2
    #define CAM_PIN_MOSI                GPIO_NUM_23
    #define CAM_PIN_MISO                GPIO_NUM_19
    #define CAM_PIN_SCLK                GPIO_NUM_18
    #define CAM_PIN_CS                  GPIO_NUM_5
    #define FPGA_PIN_MOSI               GPIO_NUM_32
    #define FPGA_PIN_MISO               GPIO_NUM_33
    #define FPGA_PIN_SCLK               GPIO_NUM_25
    #define FPGA_PIN_CS                 GPIO_NUM_26
    #define LORA_PIN_TX                 GPIO_NUM_17
    #define LORA_PIN_RX                 GPIO_NUM_16
    #define TEST_BUTTON_PIN             GPIO_NUM_0
    #define TEST_LED_PIN                GPIO_NUM_2

## Pin Summary Table

| Pin     | ESP32-CAM    | ESP32-S3    | DevKitV1        | Notes |
|---------|--------------|-------------|-----------------|-------|
| GPIO 0  |              |             | TEST_BUTTON_PIN |       |
| GPIO 1  |              | LORA_TX_PIN | LORA_TX         |       |
| GPIO 2  |              |             | LED_PIN         |       |
| GPIO 3  |              |             | FPGA_PIN_MOSI   |       |
| GPIO 4  |              |             |                 |       |
| GPIO 5  |              |             | CAM_PIN_CS      |       |
| GPIO 6  |              |             |                 |       |
| GPIO 7  |              |             |                 |       |
| GPIO 8  |              |             |                 |       |
| GPIO 9  |              | BOOT_BUTTON |                 |       |
| GPIO 10 |              |             |                 |       |
| GPIO 11 |              |             |                 |       |
| GPIO 12 | PIN_NUM_MISO |             |                 |       |
| GPIO 13 | PIN_NUM_MOSI |             |                 |       |
| GPIO 14 | PIN_NUM_CLK  |             |                 |       |
| GPIO 15 | PIN_NUM_CS   |             |                 |       |
| GPIO 16 |              |             | LORA_RX         |       |
| GPIO 17 |              | LORA_TX_PIN | LORA_TX         |       |
| GPIO 18 |              | LORA_RX_PIN | CAM_PIN_SCLK    |       |
| GPIO 19 |              |             | CAM_PIN_MISO    |       |
| GPIO 20 |              |             |                 |       |
| GPIO 21 |              |             |                 |       |
| GPIO 22 |              |             |                 |       |
| GPIO 23 |              |             | CAM_PIN_MOSI    |       |
| GPIO 24 |              |             |                 |       |
| GPIO 25 |              |             | FPGA_PIN_SCLK   |       |
| GPIO 26 |              |             | FPGA_PIN_CS     |       |
| GPIO 27 |              |             |                 |       |
| GPIO 28 |              |             |                 |       |
| GPIO 29 |              |             |                 |       |
| GPIO 30 |              |             |                 |       |
| GPIO 31 |              |             |                 |       |
| GPIO 32 |              |             | FPGA_PIN_MOSI   |       |
| GPIO 33 |              |             | FPGA_PIN_MISO   |       |
| GPIO 34 |              |             |                 |       |
| GPIO 35 |              |             |                 |       |
| GPIO 36 |              |             |                 |       |
| GPIO 37 |              |             |                 |       |
| GPIO 38 |              |             |                 |       |
| GPIO 39 |              |             |                 |       |
| GPIO 40 |              |             |                 |       |
| GPIO 41 |              |             |                 |       |
| GPIO 42 |              |             |                 |       |
| GPIO 43 |              |             |                 |       |
| GPIO 44 |              |             |                 |       |
| GPIO 45 |              |             |                 |       |
| GPIO 46 |              |             |                 |       |
| GPIO 47 |              |             |                 |       |
| GPIO 48 |              |             |                 |       |




