idf_component_register(SRCS "mpu_6050.c" "dot_matrix.c" "hourglass_timer.c"
                       PRIV_REQUIRES spi_flash
                       REQUIRES esp_timer driver
                       INCLUDE_DIRS "")
