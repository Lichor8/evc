import serial
ser = serial.Serial('/dev/ttyUSB0', 9600)
while 1:
    ser.write('0x127|y135|')
    print ser.readline()
