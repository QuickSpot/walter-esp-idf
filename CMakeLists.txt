idf_component_register(SRCS "src/WalterModem.cpp"
                    "src/proto/WalterGNSS.cpp"
                    "src/proto/WalterHTTP.cpp"
                    "src/proto/WalterMQTT.cpp"
                    "src/proto/WalterSocket.cpp"
                    INCLUDE_DIRS "src"
                    REQUIRES fatfs esp_partition app_update spi_flash driver vfs)

add_compile_options(-Wno-error)
