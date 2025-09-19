# Walter HTTP example

## Purpose

This example demonstrates how Walter can send and receive requests using **HTTP**.
Walter will make a HTTP GET and POST request to a remote webserver.

## Required Hardware

To run this example you will need the following items:

* Walter
* An LTE antenna
* A SIM card with data plan
* USB-C cable to flash Walter

## Required software

1. Please follow the instructions in the [documentation](https://www.quickspot.io/documentation.html#/) to:
[setup](https://www.quickspot.io/documentation.html#/developer-toolchains/esp-idf) ESP-IDF.

## Configuration

Before flashing the example, configure the routes and credentials:

* In the example sketch, update the following:

  ```cpp
  #define HTTP_PORT 80
  #define HTTP_HOST "quickspot.io"
  #define HTTP_GET_ENDPOINT "/hello/get"
  #define HTTP_POST_ENDPOINT "/hello/post"
  ```

## Running the example

1. Connect the LTE antenna to Walter.
   **Warning:** Running without the antenna connected may damage the radio frontend.

2. Insert the SIM card.

3. Flash the example sketch to Walter.

4. You should see requests being performed, and the responses being logged.
