import serial
import time
import sys


port = "/tmp/stockfish"
baud = 9600

ser = serial.Serial()
ser.port = port
ser.baudrate = baud
ser.bytesize = serial.EIGHTBITS
ser.parity = serial.PARITY_NONE
ser.xonxoff = 0
ser.rtscts = 0
ser.dsrdtr = 0
ser.stopbits = 1
ser.timeout = 1

ser.close()
ser.open()
if ser.isOpen():
    print(ser.name + ' is open...')
    while True:
        cmd = "uci"
        ser.write(str.encode(cmd + '\r\n')) #
        val = ser.readline().decode()
        out1 = val.strip()
        out2 = val
        print('Receiving... ' + str(out1))
        print('Receiving... ' + str(out2))
        break
ser.close()


