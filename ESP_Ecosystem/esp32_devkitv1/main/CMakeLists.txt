idf_component_register(
    SRCS "./src/main.c"
         "./src/utilities/image_processor.c"
         "./src/utilities/isr.c"
         "./src/utilities/utils.c"
         "./src/interprocessor/cam_spi.c"
         "./src/interprocessor/fpga_spi.c"
         "./src/interprocessor/lora_uart.c"
         "./src/interprocessor/i2c_bus.c"
         "./src/peripherals/aht20.c"
         "./src/peripherals/motor.c"
         "./src/peripherals/pir.c"
         "./src/peripherals/sensors_temp_humidity.c"

    INCLUDE_DIRS "./inc"
    PRIV_REQUIRES driver
)