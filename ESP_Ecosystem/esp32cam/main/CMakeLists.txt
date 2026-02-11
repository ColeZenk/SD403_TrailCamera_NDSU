idf_component_register(
  SRCS "../src/main.c"
       "../src/camera_control.c"
       "../src/DMA_SPI_master.c"
       "../src/image_buffer_pool.c"
       "../src/sd_storage.c"
  INCLUDE_DIRS "../include"
  REQUIRES esp_timer
           driver
           esp32-camera
           fatfs
           sdmmc
           vfs
)
