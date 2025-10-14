/*
 * UART Interface Implementation
 * ESP32-S3 WROOM Master - UART communication with ESP32-CAM via FPGA
 */

#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "uart_interface.h"
#include "esp32cam_control.h"

static const char *TAG = "UART_INTERFACE";

// Command handler function type
typedef void (*command_handler_t)(void);

// Command definition structure
typedef struct {
    char key;                      // Command key (lowercase)
    const char *description;       // Command description
    command_handler_t handler;     // Function to call
} command_entry_t;

// Command handler functions
static void cmd_programming_mode(void) {
    ESP_LOGI(TAG, "Command 'p' - Entering programming mode...");
    esp32cam_enter_programming_mode();
    ESP_LOGI(TAG, "Starting passthrough...");
    uart_start_passthrough_task();
    ESP_LOGI(TAG, "Passthrough active - console commands disabled");
}

static void cmd_reset_normal(void) {
    ESP_LOGI(TAG, "Command 'r' - Resetting to normal mode...");
    esp32cam_reset_to_normal_mode();
}

static void cmd_test_connection(void) {
    ESP_LOGI(TAG, "Command 't' - Testing UART connection...");
    uart_test_connection();
}

static void cmd_hard_reset(void) {
    ESP_LOGI(TAG, "Command 'h' - Performing hard reset...");
    esp32cam_hard_reset();
}

static void cmd_activate_camera(void) {
    ESP_LOGI(TAG, "Command 'c' - Activating camera in normal mode...");
    esp32cam_disable_fpga_passthrough();  // Disable programming passthrough
    vTaskDelay(pdMS_TO_TICKS(100));       // Brief delay
    esp32cam_reset_to_normal_mode();      // Reset to normal boot mode
    ESP_LOGI(TAG, "Camera activated - FPGA passthrough disabled");
}

// Command table - Add new commands here!
static const command_entry_t command_table[] = {
    {'p', "Enter programming mode", cmd_programming_mode},
    {'r', "Reset to normal mode", cmd_reset_normal},
    {'t', "Test UART connection", cmd_test_connection},
    {'h', "Hard reset", cmd_hard_reset},
    {'c', "Activate camera (normal mode)", cmd_activate_camera},
    // Add more commands here...
};

static const size_t command_table_size = sizeof(command_table) / sizeof(command_table[0]);

void print_banner(const char *Tag) {
    ESP_LOGI(Tag, "========================================");
    ESP_LOGI(Tag, "ESP32-S3 UART Programmer - Master");
    ESP_LOGI(Tag, "========================================");
    ESP_LOGI(Tag, "");
    ESP_LOGI(Tag, "=== Available Commands ===");
    
    // Dynamically generate command list from table
    for (size_t i = 0; i < command_table_size; i++) {
        ESP_LOGI(Tag, "  '%c' - %s", command_table[i].key, command_table[i].description);
    }
    
    ESP_LOGI(Tag, "==========================");
    ESP_LOGI(Tag, "");
}

void handle_command(uint8_t cmd) {
    // Ignore newlines and carriage returns
    if (cmd == '\n' || cmd == '\r') {
        return;
    }
    
    // Convert to lowercase for case-insensitive matching
    char cmd_lower = (cmd >= 'A' && cmd <= 'Z') ? (cmd + 32) : cmd;
    
    // Search command table
    for (size_t i = 0; i < command_table_size; i++) {
        if (command_table[i].key == cmd_lower) {
            // Found matching command - execute it
            command_table[i].handler();
            return;
        }
    }
    
    // Unknown command - silently ignore
    // (Could add ESP_LOGW here if you want to log unknown commands)
}

void uart_interface_init(void) {
    // Configure UART1 (to FPGA, which routes to CAM when in programming mode)
    uart_config_t cam_config = {
        .baud_rate  = UART_BAUD_RATE,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &cam_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, UART_BUF_SIZE * 2, UART_BUF_SIZE * 2, 0, NULL, 0));

    ESP_LOGI(TAG, "UART1 initialized for FPGA (TX=%d RX=%d) Baud=%d", UART_TX_PIN, UART_RX_PIN, UART_BAUD_RATE);
    ESP_LOGI(TAG, "FPGA will route UART1 to ESP32-CAM during programming mode");
}

void uart_test_connection(void) {
    ESP_LOGI(TAG, "Testing UART connection to CAM...");
    const char *test_msg = "AT\r\n"; // Some firmwares/bootloaders may reply
    uart_write_bytes(UART_PORT_NUM, test_msg, strlen(test_msg));

    uint8_t data[128];
    int len = uart_read_bytes(UART_PORT_NUM, data, sizeof(data) - 1, pdMS_TO_TICKS(500));
    if (len > 0) {
        data[len] = '\0';
        ESP_LOGI(TAG, "Received from CAM: %s", (char *)data);
    } else {
        ESP_LOGW(TAG, "No response from CAM on UART1");
    }
}

void uart_start_passthrough_task(void) {
    ESP_LOGI(TAG, "FPGA-based passthrough mode");
    ESP_LOGI(TAG, "Enabling FPGA UART multiplexer...");
    
    // Tell FPGA to route UART1 to ESP32-CAM
    esp32cam_enable_fpga_passthrough();
    
    ESP_LOGI(TAG, "FPGA passthrough ACTIVE!");
    ESP_LOGI(TAG, "UART1 (GPIO %d/%d) now connected to ESP32-CAM via FPGA", 
             UART_TX_PIN, UART_RX_PIN);
    ESP_LOGI(TAG, "Your console remains active - no need to exit monitor");
    ESP_LOGI(TAG, "You can now flash the ESP32-CAM in another terminal");
}

int uart_write_to_cam(const char *data, size_t len) {
    return uart_write_bytes(UART_PORT_NUM, data, len);
}

int uart_read_from_cam(uint8_t *data, size_t max_len, uint32_t timeout_ms) {
    return uart_read_bytes(UART_PORT_NUM, data, max_len, pdMS_TO_TICKS(timeout_ms));
}