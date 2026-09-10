idf_component_register(
    SRCS "msp_codec.c" "msp_host.c" "msp_uart.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_uart log esp_common
)
