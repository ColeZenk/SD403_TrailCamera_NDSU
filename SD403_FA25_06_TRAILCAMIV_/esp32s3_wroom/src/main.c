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
#include "esp32cam_control.h"
#include "uart_interface.h"  // Includes print_banner() and handle_command()

static const char *TAG = "MAIN";

/**
 * Initialize all subsystems
 */
static void initialize_system(void) {
    esp32cam_control_init();
    uart_interface_init();
    // Don't auto-start passthrough - wait for 'p' command
}

void app_main(void) {
    // Give USB Serial/JTAG time to enumerate and stabilize
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    printf("\n\n");  // Force output
    fflush(stdout);
    
    print_banner(TAG);
    initialize_system();
    
    ESP_LOGI(TAG, "Ready - press 'h' for help");
    fflush(stdout);
    
    // Main command loop - use getchar which works with the default console
    while (1) {
        int c = getchar();
        if (c != EOF && c != 0xFF) {
            handle_command((uint8_t)c);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}