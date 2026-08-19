idf_component_register(
    SRCS
        "gsc_stream_test_main.c"
    INCLUDE_DIRS
        "."
    REQUIRES
        freertos
        esp_timer
        esp_psram
        esp_system
        heap
        esp-sr
        esp_driver_usb_serial_jtag
)

target_compile_options(${COMPONENT_LIB} PRIVATE -O2 -ffast-math)
