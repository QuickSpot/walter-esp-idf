set(REQUIRED_COMPONENTS driver)  # driver is always required

message(STATUS "WalterModem:")

set(OPTIONAL_COMPONENTS fatfs esp_partition app_update spi_flash vfs)
set(ENABLE_EXTRA_COMPONENTS OFF)

foreach(feature IN ITEMS SOCKETS COAP MQTT HTTP MOTA OTA)
    if(CONFIG_WALTER_MODEM_ENABLE_${feature} STREQUAL "y")
        message(STATUS "${feature} enabled: true")
    else()
        message(STATUS "${feature} enabled: false")
    endif()
endforeach()

if(CONFIG_WALTER_MODEM_ENABLE_MOTA STREQUAL "y" OR CONFIG_WALTER_MODEM_ENABLE_OTA STREQUAL "y")
    list(APPEND REQUIRED_COMPONENTS ${OPTIONAL_COMPONENTS})
    set(ENABLE_EXTRA_COMPONENTS ON)
endif()

# Output the components for debugging purposes
foreach(component IN LISTS REQUIRED_COMPONENTS)
    message(STATUS "Required component: ${component}")
endforeach()

idf_component_register(
    SRCS "src/WalterModem.cpp"
         "src/proto/WalterBlueCherry.cpp"
         "src/proto/WalterGNSS.cpp"
         "src/proto/WalterHTTP.cpp"
         "src/proto/WalterMQTT.cpp"
         "src/proto/WalterSocket.cpp"
    INCLUDE_DIRS "src"
    REQUIRES ${REQUIRED_COMPONENTS}
)

add_compile_options(-Wno-error)
