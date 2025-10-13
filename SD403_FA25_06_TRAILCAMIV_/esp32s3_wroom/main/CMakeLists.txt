idf_component_register(
    SRCS "../src/main.c"
         #"../src/hspi_master.c"
         "../src/uart_interface.c"
         "../src/esp32cam_control.c"
    INCLUDE_DIRS "../include"
)
