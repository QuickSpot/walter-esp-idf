idf_component_register(SRCS "bluecherry_example.cpp"
                            "components/Bluecherry_ZTP/BlueCherryZTP.cpp"
                            "components/Bluecherry_ZTP/BlueCherryZTP_CBOR.cpp"
                    INCLUDE_DIRS "." REQUIRES driver vfs mbedtls bootloader_support)
