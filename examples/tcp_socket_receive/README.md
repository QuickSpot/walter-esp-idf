# Walter modem udp socket example

## Purpose

This example will make walter querry [example.com](https://example.com) and print the received data.
## Required hardware

To run this example you will need the following items:

- Walter
- An LTE antenna
- A SIM card
- USB-C cable to flash Walter

## Required software

Please follow the instructions from the [Documentation](https://www.quickspot.io/documentation.html#/) to
[setup](https://www.quickspot.io/documentation.html#/developer-toolchains/esp-idf) ESP-IDF.
No other libraries are required for this example to run.

## Run the example

Make sure to connect the LTE antenna to Walter. Running the example without the
antenna connected could damage the radio frontend of the modem. Also insert the
SIM card before starting the sketch.

You should now be able to see your Walter pop up on the
[Walter Demo](http://walterdemo.quickspot.io/) website. Walter identifies itself
to the demo server using his MAC address. You should see the last sent ping
counter value update every time Walter sends out a message.
