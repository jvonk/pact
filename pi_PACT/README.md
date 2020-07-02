# Raspberry Pi

## Set up a MQTT broker.
Install Eclipse Mosquitto, an open-source MQTT broker.
```console
sudo apt install mosquitto mosquitto-clients
sudo systemctl enable mosquitto
```
Get the hostname with
```console
hostname
```
or the ip address using
```console
ifconfig
```
Set up a user with a password. You will need to remember this and put it in the [config.py file](../main_PACT/config.py).
```console
mosquitto_passwd -b passwordfile user password
```

## Connect Arduinos
If you are using Arduinos, connect them to the Raspberry Pi over USB after following the instructions to set them up in [arduino_PACT/README.md](../arduino_PACT/README.md). Next, after make sure you installed Python 3, install the pyserial and paho-mqtt modules.
```console
pip3 install pyserial paho-mqtt
```
Check if the Arduinos show up.
```console
python3 -m serial.tools.list_ports -v
```
If they do, run the [readArduinos.py](readArduinos.py) file
```console
python3 readArduinos.py
```