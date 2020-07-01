# Copyright © 2020, Johan Vonk
# SPDX-License-Identifier: MIT
# pip install serial paho-mqtt
import serial, serial.tools.list_ports, serial.threaded
import time
import datetime
import paho.mqtt.client as mqtt
import json
import multiprocessing
import re

def read_serial(portName, queue):
    ser = serial.Serial(port = portName,
                        baudrate = 115200,
                        bytesize = serial.EIGHTBITS,
                        parity = serial.PARITY_NONE,
                        stopbits = serial.STOPBITS_ONE,
                        xonxoff = False,
                        rtscts = True)
    rx = 0
    rxErr = 0
    t0 = time.time()
    while ser:
        received = ser.readline().decode("ASCII")[:-1]

        #print(received.decode("ASCII"), end='')
        #print("r", ":".join("{:02x}".format(ord(c)) for c in received.decode("ASCII")))

        if re.match(r".* \{.*\}$", received):
            queue.put(received)
        else:
            rxErr += 1      

        rx += 1
        msgBits = (len(received) + 1) * 8 
        bps = round(msgBits * rx / (time.time() - t0))
        if (rx % 25 == 0):
            print(portName, "rx =", rx, "err =", rxErr, "bps = ", bps)

# fire up a process for each Arduino 101 attached

if __name__ == '__main__':

    multiprocessing.freeze_support()

    queue = multiprocessing.Queue()
    ports = serial.tools.list_ports.grep("ARDUINO 101")

    for port in ports:
        portname, desc, build = port
        print("starting thread for", portname)
        p = multiprocessing.Process(target=read_serial, args=(portname,queue,))
        p.daemon = True  # so it will die when main thread exits
        p.start()

    try:
        client = mqtt.Client("arduinos")
        client.connect('mqtt.vonk')
        client.username_pw_set(username='mqtt',password='GjRqHdcZM5UnxJ38')
        client.loop_start()
        while True:
            received = queue.get()            
            mac, json = received.split(" ", 1)
            client.publish("blescan/ibeacon/" + mac, payload = json)
            #pass

    except KeyboardInterrupt:
        exit()
