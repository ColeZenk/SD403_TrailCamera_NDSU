/**
 * main.c — ESP32 DevKitV1 entry point
 *
 * Initializes all subsystems, creates tasks, monitors heap.
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"

#include "config.h"
#include "isr_signals.h"
#include "cam_spi.h"
#include "fpga_spi.h"
#include "lora_uart.h"
#include "image_processor.h"
#include "peripherals/sensors_temp_humidity.h"
#include "peripherals/fpga_gpio.h"
#include "peripherals/i2c_bus.h"

static const char *TAG = "MAIN";

static esp_err_t init_system(void)
{
    ESP_LOGI(TAG, "=== Trail Camera IV — ESP32 DevKitV1 ===");
    ESP_LOGI(TAG, "IDF %s | heap %lu bytes", esp_get_idf_version(),
             esp_get_free_heap_size());

    esp_err_t ret;

    /* ISR semaphores must exist before any tasks */
    ret = isr_init();           if (ret != ESP_OK) return ret;
    ret = cam_spi_init();       if (ret != ESP_OK) return ret;
    ret = fpga_spi_init();      if (ret != ESP_OK) return ret;
    ret = lora_init();          if (ret != ESP_OK) return ret;
    ret = image_processor_init(); if (ret != ESP_OK) return ret;

    /* I2C bus must be up before any I2C peripheral (AHT20, FPGA GPIO) */
    static const i2c_bus_config_t i2c_cfg = {
        .port                 = FPGA_I2C_PORT,
        .sda_gpio             = FPGA_I2C_SDA_PIN,
        .scl_gpio             = FPGA_I2C_SCL_PIN,
        .clk_speed_hz         = FPGA_I2C_CLK_HZ,
        .enable_internal_pullup = false,  /* external 4.7kΩ pull-ups fitted */
    };
    ret = i2c_bus_init(&i2c_cfg);
    if (ret != ESP_OK) return ret;

    ret = sensors_init();       if (ret != ESP_OK) return ret;
    ret = fpga_gpio_init();
    if (ret != ESP_OK)
        ESP_LOGW(TAG, "FPGA GPIO expander unavailable (%s) — I2C buttons disabled",
                 esp_err_to_name(ret));

    ESP_LOGI(TAG, "all subsystems ready");
    return ESP_OK;
}

static void create_tasks(void)
{
    xTaskCreate(cam_spi_receive_task, "cam_rx",   STACK_SIZE_MEDIUM, NULL, TASK_PRIORITY_HIGH,   NULL);
    xTaskCreate(image_processor_task, "img_proc",  STACK_SIZE_LARGE,  NULL, TASK_PRIORITY_MEDIUM, NULL);
    xTaskCreate(lora_receive_task,    "lora_rx",   STACK_SIZE_MEDIUM, NULL, TASK_PRIORITY_MEDIUM, NULL);
    xTaskCreate(sensors_task,         "sensors",   STACK_SIZE_MEDIUM, NULL, TASK_PRIORITY_LOW,    NULL);

#ifdef TEST_MODE_FPGA_PATTERNS
    xTaskCreate(fpga_test_task,      "fpga_test",     STACK_SIZE_MEDIUM, NULL, TASK_PRIORITY_MEDIUM, NULL);
#endif
#ifdef TEST_MODE_FPGA_GPIO
    xTaskCreate(fpga_gpio_test_task, "fpga_gpio_test", STACK_SIZE_MEDIUM, NULL, TASK_PRIORITY_LOW,    NULL);
#endif
}

static void monitor_heap(void)
{
    uint32_t prev = esp_get_free_heap_size();

    for (;;) {
        vTaskDelay(SECONDS_TO_TICKS(10));
        uint32_t now = esp_get_free_heap_size();
        ESP_LOGI(TAG, "heap: %lu (%+ld)", now, (int32_t)(now - prev));
        prev = now;
    }
}

void app_main(void)
{
    if (init_system() != ESP_OK) {
        ESP_LOGE(TAG, "init failed — halted");
        return;
    }

    create_tasks();
    monitor_heap();
}
