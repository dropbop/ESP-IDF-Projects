idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES esp_driver_gpio esp_driver_rmt esp_driver_usb_serial_jtag
    PRIV_REQUIRES esp_timer
)
