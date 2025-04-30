# Walter positioning example

## Purpose

Walter has a built-in GNSS receiver which makes use of the GPS and Galileo
constellations. This example effectively is a demo of a tracker device in which
position is transmitted to the demo [server](http://walterdemo.quickspot.io/)
as fast as possible.

Walter's GNSS subsystem is built to be very low power and works
with 'snapshot' technology. To limit power usage the same radio is used for LTE
and GNSS and thus they cannot work concurrently. Altough this lowers power
consumption, it also means that the minimum update interval is limited by how
fast a fix is found and an LTE connection is created. You can test these
parameters using this example.

## Required hardware

To run this example you will need the following items:

- Walter
- An LTE antenna
- A passive GNSS antenna
- A SIM card (For GNSS assistance)
- USB-C cable to flash Walter

## Required software

Please follow the instructions from the [Documentation](https://www.quickspot.io/documentation.html#/) to
[setup](https://www.quickspot.io/documentation.html#/developer-toolchains/esp-idf) ESP-IDF.
No other libraries are required for this example to run.

## Run the example

Make sure to connect the LTE antenna to Walter. Running the example without the
antenna connected could damage the radio frontend of the modem. Also insert the
SIM card before starting the sketch. Lastly you need to connect the GNSS antenna
and make sure that it can receive sattelites. Make sure that the antenna has a clear view to the sky.

You should now be able to see your Walter pop up on the
[Walter Demo](http://walterdemo.quickspot.io/) website. Walter identifies itself
to the demo server using his MAC address. When clicking on your Walter you
should see your device on the map and plot the track.
