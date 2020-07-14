# ESP32 BLE iBeacon scanner and advertising (ctrl/data over MQTT)

Advertises as BLE iBeacon, or scans for iBeacon advertisements and reports them using MQTT.

I used this as a tool to research the behavior of Bluetooth Low-Energy (BLE) signals in relation to contact tracing.

![ESP32 statues scattered around the room](media/photo.png)

## Features:

- Supports boh BLE advertiser and scan modes
- Controlled and data presented through MQTT
- Optional Over-the-air (OTA) updates
- Optional WiFi provisioning using phone app

## Getting started

Parts:
- ESP32 board with 4 MByte flash memory, such as [ESP32-DevKitC-VB](https://www.espressif.com/en/products/devkits/esp32-devkitc/overview), LOLIN32, MELIFE ESP32 or pretty much any ESP32 board.
- 5 Volt, micro USB power adapter

The software relies on the ESP-IDF SDK version >= 4.1-beta2 and accompanying tools.

### Configuration

In the main directory, copy `Kconfig-example.projbuild` to `Kconfig.projbuild`, and delete `sdkconfig` so the build system will recreate it.  Either update the defaults in the `Kconfig.projbuild` files, or use `ctrl-shift-p` >> `ESP-IDF: launch gui configuration tool`.
- `WIFI_SSID`: Name of the WiFi access point to connect to.  Leave blank when provisioning using BLE and a phone app.
- `WIFI_PASSWD: Password of the WiFi access point to connect to.  Leave blank when provisioning using BLE and a phone app.
- `BLESCAN_MQTT_URL`, URL of the MQTT broker.  For authentication include the username and password, e.g. `mqtt://user:passwd@host.local:1883`
- `BLESCAN_MQTT_DATA_TOPIC`, MQTT topic for iBeacons received over BLE
- `BLESCAN_MQTT_CTRL_TOPIC`, MQTT topic for control messages
- `OTA_FIRMWARE_URL`, Optional over-the-air URL that hosts the firmware image (`.bin`)
- `RESET_PIN`, Optonal RESET input GPIO number on ESP32 that connects to a pull down switch (default 0)

### Compile

The ESP32 firmware relies on the ESP-IDF SDK version >= 4.1-beta2 and accompanying tools. For more information on how to setup the development environment in VSCode refer to [ESP32 vsCode Starter](https://github.com/cvonk/vscode-starters/tree/master/ESP32).

The firmware loads in two stages:
  1. `factory.bin` configures the WiFi using phone app (not used when WiFi credentials are set using `Kconfig`)
  2. `blescan.bin`, the main application

Compile `blescan.bin` first, by opening the top level folder in Microsft Visual Code and issuing "Terminal >> Run Task >> Build - Build the application".

When using OTA updates, the resulting `build/blescan.bin` should be copied to the OTA file server (`OTA_FIRMWARE_URL`).


### Provision WiFi credentials

If you set your WiFi SSID and password using `Kconfig` you're all set and can simply flash the application and skip the remainder of this section.

To provision the WiFi credentials using a phone app, we need a `factory` app that advertises itself to the phone app.  Open the folder `factory` in Microsft Visual Code and issue "Terminal >> Run Task >> Monitor - Start the monitor".  This will compile and flash the code.  After that it connects to the serial port to show the debug messages.  The serial rate is 115200 baud.

On your phone run the Espressif BLE Provisioning app.
- [Android](https://play.google.com/store/apps/details?id=com.espressif.provble)
- [iOS](https://apps.apple.com/in/app/esp-ble-provisioning/id1473590141)
Scan and connect to the ESP32.  Then specify the WiFi SSID and password.
(You probably have to change `_ble_device_name_prefix` to `PROV_` in `factory\main.c` and change the `config.service_uuid` in `ble_prov.c` to us the mobile apps.)

This stores the WiFi SSID and password in flash memory and triggers a OTA download of the application itself.  Alternatively, don't supply the OTA path and flash the `blescan.bin` application using the serial port.

(To erase the WiFi credentials, pull `GPIO# 0` down for at least 3 seconds.)

### OTA download

Besides connecting to WiFi, one of the first things the code does is check for OTA updates.  Upon completion, the device resets to activate the downloaded code.

We use the term "updates" loosly, because it can also be used to downgrade the firmware.  To determine if the currently running code is different as the code on the server, it compares the project name, version, date and time.  Note that these may not always updated by the SDK.

The OTA server should support either HTTP or HTTPS.  If you WiFi name and password is provisioned using `Kconfig` it will be part of the binary, so you should use a server on your local network.

## Usage

The device interfaces using the MQTT protocol.
> MQTT stands for MQ Telemetry Transport. It is a publish/subscribe, extremely simple and lightweight messaging protocol, designed for constrained devices and low-bandwidth, high-latency or unreliable networks. [FAQ](https://mqtt.org/faq)

### Control

The device support three modes:
  - `adv`, the device advertises iBeacon messages
  - `scan`, the device scans for iBeacon messages and reports them using MQTT
  - `idle`, the device neither advertises or scans

To switch modes, sent a control message with the mode to either:
- `blescan/ctrl`, group topic that all devices listen to
- `blescan/ctrl/DEVNAME`, only `DEVNAME` listens to this topic.
Here `DEVNAME` is either a programmed device name, such as `esp32-1`, or `esp32_XXXX` where the `XXXX` are the last digits of the MAC address.  Device names are hardcoded in `main/ble_scan_task.c`.

Other control messages are:
- `who`, can be used for device discovery when sent to the group topic
- `restart`, to restart the ESP32 (and check for OTA updates)
- `int N`, to change scan/adv interval to N milliseconds
- `mode`, to report the scan/adv mode and interval

Messages can be sent to a specific device, or the whole group:
```
mosquitto_pub -h {BROKER} -u {USERNAME} -P {PASSWORD} -t "blescan/ctrl/esp-1" -m "who"
mosquitto_pub -h {BROKER} -u {USERNAME} -P {PASSWORD} -t "blescan/ctrl" -m "who"
```

### Scan results and replies to Control msgs

Replies to control messages and scan results are reported using MQTT topic `blescan/data/SUBTOPIC/DEVNAME`.  Subtopics are:
- `scan`, BLE scan results
- `mode`, response to `mode` and `int` control messages
- `who`, response to `who` control messages
- `restart`, response to `restart` control messages
- `dbg`, general debug messages

E.g. to listen to all scan results, use:
```
mosquitto_sub -h {BROKER} -u {USERNAME} -P {PASSWORD} -t "blescan/data/scan/#" -v
```
where `#` is a the MQTT wildcard character.

## License

Copyright (c) 2020 Coert and Johan Vonk

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