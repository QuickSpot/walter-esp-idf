idf_component_register(SRCS "./BlueCherryZTP.cpp"
                            "./BlueCherryZTP_CBOR.cpp"
            INCLUDE_DIRS "."
            PRIV_REQUIRES 
        bootloader_support mbedtls walter-modem
            )