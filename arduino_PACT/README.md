# Arduino Setup
To set up the Arduino side of the environment, connect all the Arduinos to your computer by using USB Type B to A cables. Open the Arduino IDE and past the code in a new file.

If you are NOT using the Intel Arduino 101s, replace the first line of code with the commented out version. The Intel Arduino 101s that I am using have Intel Curie chips and don't work with the standard ArduinoBLE library. Any other Arduinos with BLE use this library

Upload the code to all of your Arduinos using the standard process. They will be waiting for messages from the Python code because since they don't have WiFi, they don't have a timestamp and can't generate UUIDs. So, the Python code sends each Arduino that it detects its UUID and name over serial.

Each Arduino is constantly sending iBeacon messages every 150ms but interrupts whenever it recognizes another BLE device to quickly write the data to Serial. This means that each Arduino can act as both a advertiser and scanner.

Make sure to follow the instructions in [pi_PACT/README.md](../pi_PACT/README.md) to connect the Arduinos to the MQTT broker.