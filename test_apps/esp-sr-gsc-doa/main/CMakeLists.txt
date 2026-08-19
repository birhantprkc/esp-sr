idf_component_register(
    SRCS
        "app_main.cpp"
        "test_doa_accuracy.cpp"
        "test_gsc_doa.cpp"
    INCLUDE_DIRS
        "."
    REQUIRES
        unity
        freertos
        esp_timer
        esp_psram
        esp_system
        heap
        esp-sr
    WHOLE_ARCHIVE
)

target_compile_options(${COMPONENT_LIB} PRIVATE -O2 -ffast-math)
