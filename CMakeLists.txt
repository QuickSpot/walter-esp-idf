set(REQUIRED_COMPONENTS driver)  # driver is always Required

if(CONFIG_WALTER_MODEM_ENABLE_MOTA OR CONFIG_WALTER_MODEM_ENABLE_OTA)
    list(APPEND REQUIRED_COMPONENTS fatfs esp_partition app_update spi_flash vfs)
endif()

message(STATUS "Required components: ${REQUIRED_COMPONENTS}")

idf_component_register(SRCS "src/WalterModem.cpp"
                    "src/proto/WalterGNSS.cpp"
                    "src/proto/WalterHTTP.cpp"
                    "src/proto/WalterMQTT.cpp"
                    "src/proto/WalterSocket.cpp"
                    INCLUDE_DIRS "src"
                    REQUIRES ${REQUIRED_COMPONENTS})

add_compile_options(-Wno-error)
