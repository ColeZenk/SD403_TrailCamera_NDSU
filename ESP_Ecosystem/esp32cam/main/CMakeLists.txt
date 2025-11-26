idf_component_register(
    SRCS "../src/main.c"
         # "../src/DMA_SPI_master.c"
         # "../src/camera_control.c"
    INCLUDE_DIRS "../include"
    REQUIRES esp32-camera fatfs sdmmc  # ← Add sdmmc
)
