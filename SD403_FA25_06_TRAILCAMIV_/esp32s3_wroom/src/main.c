/*
 * ESP32-S3 WROOM Master - Main Application
 * UART Programmer for ESP32-CAM
 * 
 * Hardware Connections:
 * WROOM (Master)  ->  ESP32-CAM (Slave)
 * 5V              ->  5V
 * GND             ->  GND
 * GPIO1           ->  IO0 (Boot Mode)
 * GPIO2           ->  EN/Reset
 * GPIO43 (TX)     ->  RX (U0RXD)
 * GPIO44 (RX)     ->  TX (U0TXD)
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "uart_interface.h"
#include "esp32cam_control.h"

static const char *TAG = "MAIN";

/**
 * Initialize all subsystems
 */
static void initialize_system(void) {
    esp32cam_control_init();
    uart_interface_init();
    uart_start_passthrough_task();
}

/**
 * Handle user commands
 */
static void handle_command(uint8_t cmd) {
    switch (cmd) {
        case 'p':
        case 'P':
            esp32cam_enter_programming_mode();
            break;
            
        case 'r':
        case 'R':
            esp32cam_reset_to_normal_mode();
            break;
            
        case 't':
        case 'T':
            uart_test_connection();
            break;
            
        case 'h':
        case 'H':
            esp32cam_hard_reset();
            break;
            
        case '\n':
        case '\r':
            // Ignore newline characters
            break;
            
        default:
            // Ignore other characters
            break;
    }
}

/**
 * Print startup banner and command help
 */
static void print_banner(void) {
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "ESP32-S3 UART Programmer - Master");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "=== Available Commands ===");
    ESP_LOGI(TAG, "  'p' - Enter programming mode");
    ESP_LOGI(TAG, "  'r' - Reset to normal mode");
    ESP_LOGI(TAG, "  't' - Test UART connection");
    ESP_LOGI(TAG, "  'h' - Hard reset");
    ESP_LOGI(TAG, "==========================");
    ESP_LOGI(TAG, "");
}

void app_main(void) {
    print_banner();
    initialize_system();
    
    // Main command loop
    uint8_t cmd;
    while (1) {
        int len = uart_read_bytes(UART_NUM_0, &cmd, 1, pdMS_TO_TICKS(100));
        
        if (len > 0) {
            handle_command(cmd);
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}