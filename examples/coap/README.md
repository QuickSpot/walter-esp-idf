# Walter CoAP example

## Purpose

This example will send a counter value to the [coap.me](https://coap.me/) test server and receive the response from the demo server.

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

You should now be able to see the incomming message from the [coap.me](coap.me) demo server in the serial monitor
