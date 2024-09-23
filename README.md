# Walter modem library for ESP-IDF

## Introduction

Walter is a board designed by [DPTechnics](https://www.dptechnics.com) which 
combines an [ESP32-S3](https://www.espressif.com/en/products/socs/esp32-s3) and
a [Sequans Monarch 2](https://www.sequans.com/products/monarch-2-gm02sp) in a
small form factor IoT module. This gives Walter a vast amount of wireless
connectivity options such as:
- Bluetooth Low Energy 5.0
- 1T1R WiFi b/g/n
- LTE Cat-M1 (LTE-M)
- LTE Cat-NB1 (NB-IoT rel. 13)
- LTE Cat-NB2 (NB-IoT rel. 14), ready for rel. 15
- GNSS receiver (GPS and Galileo)

Besides these you get all the goodies from the ESP32-S3 chip such as  SPI, I2S,
I2C, PWM, RMT, ADC, UART, SD/MMC host and TWAI. 

We design and manufacture Walter in Belgium and guarantee that the board will be
available for a minimum of 10 years. This makes Walter a solid choice to design
your next LPWAN IoT product with.

The Walter modem library makes it easy to interface with the Sequans Monarch 2
modem on the ESP-IDF platform. The library allows for UDP and TCP communication
over NB-IoT and LTE-M networks and also supports the GNSS functionality. 

This library is designed to consume as little energy as possible by making use
of the FreeRTOS locking mechanisms and the hardware UART. There are no active
wait situations which consume useless CPU cycles. Besides that the library
does not allocate dynamic heap memory. All RAM is determined at compiled time.
This makes debugging easier and mitigates unexpected out-of-memory situations.

All Walter libraries and software are written with the same design strategies:
 - Keep it simple
 - Be as efficient as possible
 - Keep the code documented
 - Do not change underlying frameworks

## Contributions

We welcome all contributions to the software via github pull requests. Please
take the design strategies in mind when contributing. 

## License

The library is published under the 'DPTechnics 5 clause' license. This is 
essentially the same as the `BSD-3-Clause` license with the addition that
binaries of which the source code is not open should run on a Walter board from
DPTechnics bv.