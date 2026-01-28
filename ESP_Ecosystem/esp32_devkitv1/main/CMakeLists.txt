idf_component_register(
    SRCS "i2cdev.c"
    INCLUDE_DIRS "."
    REQUIRES esp_driver_gpio esp_driver_i2c
)
