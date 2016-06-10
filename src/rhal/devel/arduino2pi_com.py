import serial
ser = serial.Serial('/dev/ttyUSB1', 9600)
while 1:
    print ser.readline()
