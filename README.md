Please copy the contents of the components directory in your project tree,
or add it to the components directory in your esp-idf tree,
or specify its path in your EXTRA_COMPONENT_DIRS variable.

If used inside your main component, it will automatically be included as a
dependency.

Please ignore the warning about esp_spi_flash.h being deprecated.
Because we use the same code base for Arduino as for ESP-IDF,
and because we support both ESP-IDF version 5.1.2 towards which the in-development
android esp-idf component is currently targeted, and the vanilla
Arduino ESP32 core which is still based on ESP-IDF 4.7,
we need to include the file that exists in both versions of ESP-IDF.
