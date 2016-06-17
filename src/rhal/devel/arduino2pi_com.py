import serial
import time

ser = serial.Serial('/dev/ttyUSB0', 9600)
while 1:
    ser.write('0x127|y135|')
    if ser.inWaiting() > 0:
        print ser.readline()

    time.sleep(1)

# try delay or buffersize?
