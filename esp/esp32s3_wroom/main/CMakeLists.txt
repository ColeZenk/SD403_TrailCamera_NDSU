idf_component_register(
    SRCS
        "src/main.c"
        "src/wifi_ap.c"
        "src/ws_server.c"
        "src/lora_bench.c"
        "src/lora_uart.c"
    INCLUDE_DIRS
        "inc"
    REQUIRES
        esp_wifi
        esp_http_server
        esp_psram
        driver
        nvs_flash
        esp_event
        newlib
        esp_timer
)

target_compile_options(${COMPONENT_LIB} PRIVATE -lm)
