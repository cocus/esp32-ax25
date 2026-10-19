idf_component_register(
    SRCS
        "src/afsk_demod.c"
        "src/afsk_mod.c"
        "src/ax25.c"
        "src/afsk_modem.c"
        "src/debug_server.c"
        "src/kiss_server.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_dac esp_hal_ana_conv freertos
    PRIV_REQUIRES esp_adc esp_driver_gptimer esp_driver_gpio esp_timer log heap lwip esp_wifi
)

# The DSP loops run 48000 times a second; optimise them even when the rest of the app is a debug build.
target_compile_options(${COMPONENT_LIB} PRIVATE -O2)
