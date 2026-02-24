#ifndef IMAGE_BUFFER_H
#define IMAGE_BUFFER_H

#include <stdint.h>
#include <stddef.h>
#include "Cam_DMA_interface.h"
#include "freertos/FreeRTOS.h"

// Initialize image buffer module
void image_buffer_init(void);

// Get queue handle for receiving images
QueueHandle_t image_buffer_get_queue(void);

// Image processing task
void image_process_task(void *pvParameters);

#endif // IMAGE_BUFFER_Hendif // UART_INTERFACE_H
