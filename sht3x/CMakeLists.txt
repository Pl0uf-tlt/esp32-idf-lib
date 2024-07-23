idf_component_register(
    SRCS sht3x.c
    INCLUDE_DIRS .
    REQUIRES log esp_idf_lib_helpers esp_timer driver
)
