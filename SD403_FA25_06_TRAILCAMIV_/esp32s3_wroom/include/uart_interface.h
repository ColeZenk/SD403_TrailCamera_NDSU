/*
 * UART Interface Header
 * ESP32-S3 WROOM Master - UART communication with ESP32-CAM
 */

#ifndef UART_INTERFACE_H
#define UART_INTERFACE_H

#include "driver/uart.h"
#include "driver/gpio.h"
#include <stdint.h>
#include <stddef.h>

// UART Pin Definitions for UART1 (to CAM)
// Note: GPIO 43/44 are used by UART0 (USB console), so we use different pins
#define UART_TX_PIN         GPIO_NUM_17     // TX pin for UART1 to CAM
#define UART_RX_PIN         GPIO_NUM_18     // RX pin for UART1 to CAM

// UART Configuration
#define UART_PORT_NUM       UART_NUM_1      // Using UART1 for CAM communication
#define UART_BAUD_RATE      115200
#define UART_BUF_SIZE       1024

/**
 * Initialize UART for communication with ESP32-CAM
 */
void uart_interface_init(void);

/**
 * Test UART connection with ESP32-CAM
 */
void uart_test_connection(void);

/**
 * Start UART passthrough task
 * Bridges UART0 (USB) and UART1 (CAM) for programming
 */
void uart_start_passthrough_task(void);

/**
 * Write data to ESP32-CAM via UART
 * @param data Pointer to data buffer
 * @param len Length of data to write
 * @return Number of bytes written
 */
int uart_write_to_cam(const char *data, size_t len);

/**
 * Read data from ESP32-CAM via UART
 * @param data Pointer to buffer to store received data
 * @param max_len Maximum length to read
 * @param timeout_ms Timeout in milliseconds
 * @return Number of bytes read
 */
int uart_read_from_cam(uint8_t *data, size_t max_len, uint32_t timeout_ms);

/**
 * Handle user commands from console
 * @param cmd Command character received
 */
void handle_command(uint8_t cmd);

/**
 * Print banner/menu to console
 * @param Tag Logging tag to use
 */
void print_banner(const char *Tag);

#endif // UART_INTERFACE_H