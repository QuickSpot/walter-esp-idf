idf_component_register(SRCS "src/WalterModem.cpp"
                    INCLUDE_DIRS "src"
                    REQUIRES fatfs esp_partition app_update spi_flash driver)
add_compile_options(-Wno-error)
