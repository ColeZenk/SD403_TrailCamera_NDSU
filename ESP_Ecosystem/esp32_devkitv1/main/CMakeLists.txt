idf_component_register(
    SRCS "../src/main.c"
         "../src/cam_spi.c"
         "../src/fpga_spi.c"
         "../src/image_processor.c"
         "../src/isr.c"
         "../src/lora_uart.c"
         "../src/utils.c"
         "../src/peripherals/aht20.c"
         "../src/peripherals/i2c_bus.c"
         "../src/peripherals/motor.c"
         "../src/peripherals/pir.c"
         "../src/peripherals/sensors_temp_humidity.c"

    INCLUDE_DIRS "../inc"
    PRIV_REQUIRES driver
)
