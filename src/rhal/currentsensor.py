import serial


def currentsensor():

    ser = serial.Serial('/dev/ttyUSB0', 9600)
    while 1:
        print(ser.readline())
