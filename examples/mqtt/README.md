# Walter MQTT example

## Purpose

This example demonstrates how Walter can connect using **MQTT**.
Walter will publish a counter value along with its MAC address to the MQTT broker on the topic.
It will subscribe to the same topic for incoming messages.

## Required hardware

To run this example you will need the following items:

- Walter
- An LTE antenna
- A SIM card
- USB-C cable to flash Walter

## Required software

1. Please follow the instructions in the [documentation](https://www.quickspot.io/documentation.html#/) to:
[setup](https://www.quickspot.io/documentation.html#/developer-toolchains/esp-idf) ESP-IDF.

2. You will also need a tool to view the MQTT messages, we recommend [MQTT explorer](https://mqtt-explorer.com/)

## Configuration

Before flashing the example, configure the default settings:

* In the example sketch, update the following:

  ```cpp
    #define MQTT_PORT 1883
    #define MQTT_HOST "broker.emqx.io"
    #define MQTT_TOPIC "walter-test-topic"
    #define MQTT_CLIENT_ID "walter-client"
  ```

## Running the example

1. Connect the LTE antenna to Walter.
   **Warning:** Running without the antenna connected may damage the radio frontend.

2. Insert the SIM card.

3. Flash the example sketch to Walter.

4. You should see requests being performed, and the responses being logged.