#ifndef IMAGE_DATA_H
#define IMAGE_DATA_H

#include <stdint.h>
#include <stddef.h>

// Image data structure for passing between tasks
typedef struct {
  uint8_t *buffer;
  size_t size;
} image_data_t;

#endif // IMAGE_DATA_H
