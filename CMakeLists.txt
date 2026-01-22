message(STATUS "WalterModem Configuration:")

foreach(feature IN ITEMS BLUECHERRY GNSS MOTA SOCKETS COAP MQTT HTTP)
    if(CONFIG_WALTER_MODEM_ENABLE_${feature} STREQUAL "y")
        message(STATUS "${feature} enabled: true")
    else()
        message(STATUS "${feature} enabled: false")
    endif()
endforeach()

idf_component_register(
    SRCS "src/WalterModem.cpp"
         "src/proto/WalterCoAP.cpp"
         "src/proto/WalterBlueCherry.cpp"
         "src/proto/WalterGNSS.cpp"
         "src/proto/WalterHTTP.cpp"
         "src/proto/WalterMQTT.cpp"
         "src/proto/WalterSocket.cpp"
    INCLUDE_DIRS "src"
    REQUIRES fatfs esp_partition app_update spi_flash driver vfs
    PRIV_REQUIRES esp_timer
)

add_compile_options(-Wno-error)
