idf_component_register(SRCS "bt_speaker.c"
    PRIV_REQUIRES
        bt
        nvs_flash
        esp_driver_i2s
        esp_driver_i2c
        esp_driver_gpio
        esp_codec_dev
)
