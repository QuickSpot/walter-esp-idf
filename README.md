# Walter Modem Component for ESP-IDF

This is the repository of the Walter Modem for the Sequans GM02SP.

## Installation

## Getting Started

## How to use

### Using via Component Registry

This is the easiest way to use Walter Modem component, however, the registration is not done yet. Please go to the manual installation.

```bash
idf.py add-dependency "QuickSpot/walter-esp-idf^1.0.0" 
```

This will be available soon!

### Manual

To use this component inside your project, you must download the content of this repository to a convinient folder.

    Please ensure that the folder path is not excessively long (especially for Windows users).

Create or copy the `idf_component.yml` file inside the `main` folder of your project and change the path to the Walter Modem component (`walter-modem`) folder.

Inside the `idf_component.yml` file, change the location of the component folder.

```yml
dependencies:
  walter-modem:
    path: <path_to_the_folder>
```

### Note

Please ignore the warning about esp_spi_flash.h being deprecated.

Because we use the same code base for Arduino as for ESP-IDF,
and because we support both ESP-IDF version 5.1.2 towards which the in-development
android esp-idf component is currently targeted, and the vanilla
Arduino ESP32 core which is still based on ESP-IDF 4.4,
we need to include the file that exists in both versions of ESP-IDF.
