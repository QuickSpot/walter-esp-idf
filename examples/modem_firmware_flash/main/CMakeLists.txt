idf_component_register(SRCS "main.c" "modem.c" "modem_stp.c"
                       PRIV_REQUIRES vfs fatfs
                       INCLUDE_DIRS "include")

set(image ../modem_files)
fatfs_create_rawflash_image(ffat ${image} FLASH_IN_PROJECT PRESERVE_TIME)
