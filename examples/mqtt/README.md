# Walter MQTT example

## Purpose

This example will make Walter count and send the counter value and mac string to the MQTT broker.
on the `walter-mqtt-test-topic`  topic.

## Required hardware

To run this example you will need the following items:

- Walter
- An LTE antenna
- A SIM card
- USB-C cable to flash Walter

## Required Software

Please follow the instructions from the [Documentation](https://www.quickspot.io/documentation.html#/) to
[setup](https://www.quickspot.io/documentation.html#/developer-toolchains/esp-idf) ESP-IDF.
No other libraries are required for this example to run.

You will also need a tool to view the MQTT messages, we recommend [MQTT explorer](https://mqtt-explorer.com/)

## Run the example

Make sure to connect the LTE antenna to Walter. Running the example without the
antenna connected could damage the radio frontend of the modem. Also insert the
SIM card before starting the sketch.

You should now be able to see your Walters mac pop up under the `walter-mqtt-test-topic`  topic.
