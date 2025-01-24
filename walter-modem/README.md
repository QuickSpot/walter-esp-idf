# Walter Modem Driver for ESP-IDF

## Overview

This component contains the WalterModem library which is used to use the 
Sequans GM02SP modem which is found on the Walter module. The Walter module 
is a fully certified modem module (CE, UKCA, FCC, IC, RCM) which is designed to 
be integrated in a cellular IoT product. Walter combines LTE-M, NB-IoT, WiFi,
BLE and GNSS in a single module and runs on  the ESP32-S3 MCU with 2MB of PSRAM
and 16MB of flash memory.

## How to use
This is the easiest way to use Walter Modem component.

```bash
idf.py add-dependency "dptechnics/walter-modem^1.1.2" 
```

To use this component inside your project, you must download the content of this
repository to a convenient folder.

Please ensure that the folder path is not excessively long (especially for
Windows users).

Create the `idf_component.yml` file inside the `main` folder of your project,
or copy and modify `idf_component.yml` from one of the examples,
and change the path to the Walter Modem component (`walter-modem`) folder.

```yml
dependencies:
  walter-modem:
    path: <path_to_the_folder>
```

## Contributing

We welcome all contributions to the software via github pull requests on the 
[walter-esp-idf repository](https://github.com/QuickSpot/walter-esp-idf). Please
take the design strategies in mind when contributing. 

## Licence

The library is published under the 'DPTechnics 5 clause' license. This is 
essentially the same as the `BSD-3-Clause` license with the addition that
binaries of which the source code is not open should run on a Walter board from
DPTechnics bv.