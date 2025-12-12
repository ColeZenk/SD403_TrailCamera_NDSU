/*
 * FPGA_SPI_interface.c
 * SPI DMA Master implementation for Tang Nano 9K
 */

#include "FPGA_SPI_interface.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "FPGA_SPI";

#define SPI_CLOCK_SPEED    (SPI_CLOCK_MHZ * 1000000)
#define DMA_CHANNEL        SPI_DMA_CH_AUTO
#define MAX_TRANSFER_SIZE  4092
#define BUTTON_PIN         GPIO_NUM_0
#define TEST_IMAGE_SIZE    76800  // 320x240

static spi_device_handle_t spi_fpga;

esp_err_t fpga_spi_init(void)
{
    ESP_LOGI(TAG, "Initializing SPI master to FPGA...");
    
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = FPGA_MOSI_PIN,
        .miso_io_num = FPGA_MISO_PIN,
        .sclk_io_num = FPGA_SCLK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = MAX_TRANSFER_SIZE,
        .flags = SPICOMMON_BUSFLAG_MASTER,
    };
    
    esp_err_t ret = spi_bus_initialize(SPI3_HOST, &bus_cfg, DMA_CHANNEL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI bus init failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = SPI_CLOCK_SPEED,
        .mode = 0,
        .spics_io_num = FPGA_CS_PIN,
        .queue_size = 3,
        .flags = 0,
    };
    
    ret = spi_bus_add_device(SPI3_HOST, &dev_cfg, &spi_fpga);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI device add failed: %s", esp_err_to_name(ret));
        spi_bus_free(SPI3_HOST);
        return ret;
    }
    
    ESP_LOGI(TAG, "SPI to FPGA ready:");
    ESP_LOGI(TAG, "  MOSI=%d MISO=%d SCLK=%d CS=%d", 
             FPGA_MOSI_PIN, FPGA_MISO_PIN, FPGA_SCLK_PIN, FPGA_CS_PIN);
    ESP_LOGI(TAG, "  Clock: %d MHz", SPI_CLOCK_MHZ);
    
    return ESP_OK;
}

esp_err_t fpga_spi_transmit_image(const uint8_t *data, size_t total_size)
{
    size_t offset = 0;
    
    ESP_LOGI(TAG, "Transmitting %d bytes...", total_size);
    
    while (offset < total_size) {
        size_t chunk_size = (total_size - offset > MAX_TRANSFER_SIZE) 
                            ? MAX_TRANSFER_SIZE 
                            : (total_size - offset);
        
        spi_transaction_t trans = {
            .length = chunk_size * 8,
            .tx_buffer = data + offset,
            .rx_buffer = NULL,
        };
        
        esp_err_t ret = spi_device_transmit(spi_fpga, &trans);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Transmit failed at offset %d: %s", offset, esp_err_to_name(ret));
            return ret;
        }
        
        offset += chunk_size;
    }
    
    ESP_LOGI(TAG, "Transmission complete");
    return ESP_OK;
}

void fpga_test_task(void *pvParameters)
{
    // Configure button
    gpio_config_t btn_cfg = {
        .pin_bit_mask = (1ULL << BUTTON_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&btn_cfg);
    
    // Allocate DMA-capable buffer
    uint8_t *test_buffer = heap_caps_malloc(TEST_IMAGE_SIZE, MALLOC_CAP_DMA);
    if (test_buffer == NULL) {
        ESP_LOGE(TAG, "DMA buffer allocation failed");
        vTaskDelete(NULL);
        return;
    }
    
    uint8_t pattern = 0xAA;
    int frame_count = 0;
    
    ESP_LOGI(TAG, "Test task ready - press BOOT button to transmit");
   static uint8_t pattern_num = 0;

  while (1) {
      if (gpio_get_level(BUTTON_PIN) == 0) {
          ESP_LOGI(TAG, "Button pressed! Pattern %d", pattern_num);
          
          // Generate different patterns
          switch (pattern_num % 4) {
              case 0:  // Solid white
                  memset(test_buffer, 0xFF, TEST_IMAGE_SIZE);
                  ESP_LOGI(TAG, "Pattern: Solid WHITE");
                  break;
              case 1:  // Solid black
                  memset(test_buffer, 0x00, TEST_IMAGE_SIZE);
                  ESP_LOGI(TAG, "Pattern: Solid BLACK");
                  break;
              case 2:  // Gradient - should show smooth color transition
                  for (int i = 0; i < TEST_IMAGE_SIZE; i++) {
                      test_buffer[i] = (i / 16) & 0xFF;  // Slower gradient, more visible
                  }
                  ESP_LOGI(TAG, "Pattern: Gradient");
                  break;
              case 3:  // Alternating 0xAA/0x55 (checkerboard-ish)
                  for (int i = 0; i < TEST_IMAGE_SIZE; i++) {
                      test_buffer[i] = (i & 1) ? 0xAA : 0x55;
                  }
                  ESP_LOGI(TAG, "Pattern: Alternating");
                  break;
          }
          
          fpga_spi_transmit_image(test_buffer, TEST_IMAGE_SIZE);
          pattern_num++;
      }
      
  }    
    free(test_buffer);
    vTaskDelete(NULL);
}
