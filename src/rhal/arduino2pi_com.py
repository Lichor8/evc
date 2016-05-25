import serial
ser = serial.Serial('/dev/ttyACM2', 9600)
while 1 :
    print ser.readline()
