set(REQUIRED_COMPONENTS driver)  # driver is always Required

if(CONFIG_WALTER_MODEM_ENABLE_SOCKETS STREQUAL "y")
    message(STATUS "SOCKETS enabled: true")
else()
    message(STATUS "SOCKETS enabled: false")
endif()

if(CONFIG_WALTER_MODEM_ENABLE_COAP STREQUAL "y")
    message(STATUS "COAP enabled: true")
else()
    message(STATUS "COAP enabled: false")
endif()

if(CONFIG_WALTER_MODEM_ENABLE_MQTT STREQUAL "y")
    message(STATUS "MQTT enabled: true")
else()
    message(STATUS "MQTT enabled: false")
endif()

if(CONFIG_WALTER_MODEM_ENABLE_HTTP STREQUAL "y")
    message(STATUS "HTTP enabled: true")
else()
    message(STATUS "HTTP enabled: false")
endif()

if(CONFIG_WALTER_MODEM_ENABLE_MOTA STREQUAL "y")
    message(STATUS "MOTA enabled: true")
else()
    message(STATUS "MOTA enabled: false")
endif()

if(CONFIG_WALTER_MODEM_ENABLE_OTA STREQUAL "y")
    message(STATUS "OTA enabled: true")
else()
    message(STATUS "OTA enabled: false")
endif()


if(CONFIG_WALTER_MODEM_ENABLE_MOTA  STREQUAL "y" OR CONFIG_WALTER_MODEM_ENABLE_OTA  STREQUAL "y")
    list(APPEND REQUIRED_COMPONENTS fatfs esp_partition app_update spi_flash vfs)
endif()

# Join list elements with spaces (explode the list)
string(REPLACE ";" " " EXPLODED_COMPONENTS "${REQUIRED_COMPONENTS}")
message(STATUS "Required components: ${EXPLODED_COMPONENTS}")

idf_component_register(SRCS "src/WalterModem.cpp"
                    "src/proto/WalterBlueCherry.cpp"
                    "src/proto/WalterGNSS.cpp"
                    "src/proto/WalterHTTP.cpp"
                    "src/proto/WalterMQTT.cpp"
                    "src/proto/WalterSocket.cpp"
                    INCLUDE_DIRS "src"
                    REQUIRES ${REQUIRED_COMPONENTS})

add_compile_options(-Wno-error)
