idf_component_register(SRCS "WalterModem.cpp"
                    INCLUDE_DIRS "."
                    REQUIRES esp_partition app_update spi_flash driver)
