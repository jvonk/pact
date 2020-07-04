[![LICENSE](https://img.shields.io/github/license/jvonk/pact)](LICENSE)

# PACT Data Ecosystem with MQTT
This repository provides a complete ecosystem to share data such as RSSI measurements from BLE beacons.

## ESP32
ESP32 is a fairly small and cheap microcontroller made by Espressif Systems which can run both MQTT and Bluetooth at the same time. Code connects and sends all information to your MQTT server. See [ESP32_PACT/README.md](ESP32_PACT/README.md) for more.

## Arduino
While this code was only tested on the Intel Arduino 101, a discontinued board which used the Intel Curie model, it should work on any Arduino with Bluetooth. See [arduino_PACT/README.md](arduino_PACT/README.md) for more.

## Raspberry Pi
Currently only used to interface with the Arduinos and host the MQTT server, in the future, I hope to modify code to advertise and scan for messages and send them over MQTT too. See [pi_PACT/README.md](pi_PACT/README.md) for more.

## Data Processing
Python code in an Jupyter Notebook to subscribe and receive data from MQTT. Works on any computer with Jupyter and Python support. See [main_PACT/README.md](main_PACT/README.md) for more.
