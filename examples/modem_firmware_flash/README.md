# Walter Modem Firmware Flash example

This example project uses the ESP32-S3 on Walter to perform firmware updates to the Sequans modem. This project can be used to:

- Update a Sequans modem to a newer firmware version
- Flash a Sequans modem that is in a recovery boot state

Depending on the current boot state of the modem, the project will perform several steps to transfer necessary the modem firmware images from the ESP32-S3 flash storage to the modem flash storage. The latest firmware version images are included in the `modem_files` directory, and are automatically flashed by the example project onto a FATFS partition.

> [!IMPORTANT]
> The entire update process might require multiple firmware transfers and can take up to 30 minutes to complete, depending on the initial boot state of the modem.

The current latest modem firmware version is: `LR8.2.1.0-61488`

## Build and flash

Build the project and flash it to the board, then run monitor tool to view serial output.

```
idf.py -p PORT build flash monitor
```

(Replace PORT with the name of the serial port to use.)

## Example output

```bash
I (551) main: Initializing FATFS partition
I (561) main: Initializing modem
I (561) uart: queue free spaces: 30
I (571) gpio: GPIO[45]| InputEn: 0| OutputEn: 1| OpenDrain: 1| Pullup: 0| Pulldown: 0| Intr:0
I (2581) main: Boot mode: RECOVERY
I (2581) main: Starting transfer of .dup firmware image
I (3481) modem: Transferring: 0%
I (11401) modem: Transferring: 1%
I (19331) modem: Transferring: 2%
I (27251) modem: Transferring: 3%
...
I (782371) modem: Transferring: 98%
I (790291) modem: Transferring: 99%
I (797651) modem: Transferring: 100%
W (799901) uart_terminal: Rx Break
I (813151) main: Transfer of .dup firmware image complete
I (813151) main: Waiting for new boot mode poll
I (815151) main: Boot mode: FFF
I (815161) main: No upgrade needed, on latest software version: LR8.2.1.0-61488
I (815161) main_task: Returned from app_main()
```
_End of update process_