idf_component_register(SRCS "bluecherry_example.cpp"
                    INCLUDE_DIRS "." REQUIRES driver vfs mbedtls bootloader_support 
                    PRIV_REQUIRES Bluecherry_ZTP)
