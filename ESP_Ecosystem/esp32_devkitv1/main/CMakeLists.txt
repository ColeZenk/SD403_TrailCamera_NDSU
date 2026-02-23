idf_component_register(
    SRCS "../src/main.c"
         "../src/cam_spi.c"
         "../src/fpga_spi.c"
         "../src/image_processor.c"
         "../src/isr.c"
         "../src/lora_uart.c"
         "../src/utils.c"

    INCLUDE_DIRS "../inc"
)
