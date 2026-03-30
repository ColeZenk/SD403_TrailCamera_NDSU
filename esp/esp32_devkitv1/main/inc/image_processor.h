/**
 * @file image_processor.h
 * @brief Image Processor - Camera SPI to FPGA Pipeline
 * @author Cole Zenk
 */

#ifndef IMAGE_PROCESSOR_H
#define IMAGE_PROCESSOR_H

#include "esp_err.h"
#include "freertos/FreeRTOS.h"

esp_err_t image_processor_init(void);
void      image_processor_task(void *pvParameters);

#endif /* IMAGE_PROCESSOR_H */
