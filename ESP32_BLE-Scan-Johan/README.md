# ESP32 BLE iBeacon scanner and advertising (ctrl/data over MQTT)

Advertises as BLE iBeacon, or scans for iBeacon advertisements and reports them using MQTT.

## Getting started

Requires ESP board with 4 MByte flash memory, e.g. [ESP32-DevKitC-VB](https://www.espressif.com/en/products/devkits/esp32-devkitc/overview)

The software relies on the ESP-IDF SDK version >= 4.1-beta2 and accompanying tools.

### Code configuration

For development environment:(GNU toolchain, ESP-IDF, JTAG/OpenOCD, VSCode) refer to https://github.com/cvonk/vscode-starters/tree/master/ESP32

Use `menuconfig` to configure:
- `BLESCAN_MQTT_URL`, URL of the MQTT broker.  For authentication include the username and password, e.g. `mqtt://user:passwd@host.local:1883`
- `BLESCAN_MQTT_TOPIC`, MQTT topic for iBeacons received over BLE
- `BLESCAN_MQTT_CTRL_TOPIC`, MQTT topic for control messages
- `BLESCAN_RESET_PIN`, RESET input GPIO number on ESP32 that connects to a pull down switch (default 0)
- `BLESCAN_RESET_SECONDS`, Number of seconds that RESET needs to be hold low before erasing WiFi credentials (default 3)
- `BLESCAN_OTA_FIRMWARE_URL`, Optional public URL of server that hosts the firmware image (`.bin`)

Load the "factory.bin" image using UART, and use a the the Espressif BLE Provisioning app from
- [Android](https://play.google.com/store/apps/details?id=com.espressif.provble)
- [iOS](https://apps.apple.com/in/app/esp-ble-provisioning/id1473590141)

This stores the WiFi SSID and password in flash memory and triggers a OTA download of the application itself.  Aternatively, don't supply the OTA path and load the "blescan.bin" application using UART.

To erase the WiFi credentials, pull `GPIO# 0` down for at least 3 seconds.

### Over-the-air Updates (OTA)

If not using OTA updates, just provision and load the application `.bin` over the serial port.

## Usage

### Control

Three modes
  - `adv`, the device advertises iBeacon messages
  - `scan`, the device scans for iBeacon messages and reports the using MQTT
  - `idle`, the device neither advertises or scans

To switch modes, sent a control message to either MQTT topic:
- `blescan/ctrl`, all devices listen to this
- `blescan/ctrl/BLEMAC`, where `BLEMAC` is the device specific BLE MAC hardware, e.g. `30aea4cc324e`

Messages can be sent to a specific device, or the whole group:
```
mosquitto_pub -h mqtt.vonk -u {USERNAME} -P {PASSWORD} -t "blescan/ctrl/esp-1" -m "echo"
mosquitto_pub -h mqtt.vonk -u {USERNAME} -P {PASSWORD} -t "blescan/ctrl" -m "echo"
```

Other control messages are:
- `restart`, to restart the ESP32 (and check for OTA updates)
- `echo`, can be used for device discovery when sent to the group

### Scan results

Scan results are reported on MQTT topic:
- `blescan/ibeacon/DEVNAME`, where `DEVNAME` is either:
   - the device name, such as `esp32-1`, or
   - `esp32_XXXX` where the `XXXX` are the last digits of the BLE hardware address.

To listen to all scan results, use e.g.
```
mosquitto_sub -h mqtt.vonk -u {USERNAME} -P {PASSWORD} -t "blescan/ibeacon/#" -v
```
where `#` is a wildcard character.

## Imporvements

Make resiliant towards WiFi and MQTT outages.

## License

Copyright (c) 2020 Johan Vonk

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE
OR OTHER DEALINGS IN THE SOFTWARE.