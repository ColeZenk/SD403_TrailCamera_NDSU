idf_component_register(
  SRCS "src/main.c"
       "src/cam_ctrl.c"
       "src/ov2640_cfg.c"
       "src/dma_spi.c"
       "src/img_buf.c"
       "src/sd_stg.c"
  INCLUDE_DIRS "inc"
  REQUIRES esp_timer
           driver
           esp32-camera
           fatfs
           sdmmc
           vfs
)
