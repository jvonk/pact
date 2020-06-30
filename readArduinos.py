import multiprocessing
import serial, serial.tools.list_ports, serial.threaded
import time

def read_serial(portName):
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
        received = ser.readline()
        #print(received.decode("ASCII"), end='')
        #print("r", ":".join("{:02x}".format(ord(c)) for c in received.decode("ASCII")))
        ok = received[0] == ord('{') and received[-2] == ord('}')
        if (not ok):
            rxErr += 1      
        rx += 1
        msgBits = (len(received) + 1) * 8 
        bps = round(msgBits * rx / (time.time() - t0))
        if (rx % 25 == 0):
            print(portName, "rx =", rx, "err =", rxErr, "bps = ", bps)

if __name__ == '__main__':
    multiprocessing.freeze_support()
    ports = serial.tools.list_ports.grep("ARDUINO 101")

    for port in ports:
        portname, desc, build = port
        print("starting thread for", portname)
        p = multiprocessing.Process(target=read_serial, args=(portname,))
        p.daemon = True  # so it will die when main thread exits
        p.start()

    try:
        while True:
            pass
    except KeyboardInterrupt:
        exit()
