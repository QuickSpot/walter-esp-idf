# Walter TCP example

## Purpose

This example demonstrates how to **set up a bidirectional TCP connection** using Walter.
Walter constructs a data packet and sends it to a remote endpoint every 30 seconds.
For demonstration, we use the [Quickspot demo server](http://walterdemo.quickspot.io/) as the endpoint,
but you can replace this with any compatible TCP server in your own application.

## Required hardware

To run this example you will need the following items:

- Walter
- An LTE antenna
- A SIM card
- USB-C cable to flash Walter

## Required software

1. Please follow the instructions in the [documentation](https://www.quickspot.io/documentation.html#/) to:
[setup](https://www.quickspot.io/documentation.html#/developer-toolchains/esp-idf) ESP-IDF.

## Running the example

1. Connect the LTE antenna to Walter.
   **Warning:** Running without the antenna connected may damage the radio frontend.

2. Insert the SIM card.

3. Flash the example sketch to Walter.

4. You should see requests being performed, and the responses being logged.