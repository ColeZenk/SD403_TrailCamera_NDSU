/*
 * ESP32 DevKitV1 main
 *
 * This controler will act as the main processor
 *
 * With this being the main processor there are a few tasks
 * that it controls.
 *
 * Tasks:
 *   - Pass image data from the cam to the Tang nano 
 *   - Receive compressed data from the FPGA and transmit
 *     it via LoRa
 *
 *   - General periphal control and management:
 *     + IO functionality with the stepper motor and sensors
 *     + Ability to turn on from IO stimulation
 *     + Bluetooth regulation
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "Cam_DMA_interface.h"
#include "Image_buffer.h"

static const char *TAG = "MAIN";

void app_main(void)
{
    ESP_LOGI(TAG, "ESP32-WROOM SPI DMA Receiver");
    
    if (spi_slave_dma_init() != ESP_OK) {
        ESP_LOGE(TAG, "SPI initialization failed!");
        return;
    }

    image_buffer_init();

    xTaskCreate(spi_receive_task, "spi_rx_task", 4096, NULL, 5, NULL);
    xTaskCreate(image_process_task, "img_proc_task", 8192, NULL, 4, NULL);
    
    ESP_LOGI(TAG, "SPI DMA highway ready!");
}
