idf_component_register(SRCS "src/WalterModem.cpp"
                    INCLUDE_DIRS "src"
                    REQUIRES esp_partition app_update spi_flash driver)
