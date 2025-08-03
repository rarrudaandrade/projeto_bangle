idf_component_register(
    SRCS "gattClient.c"
    PRIV_REQUIRES bt nvs_flash esp-mqtt
    INCLUDE_DIRS "."
)
