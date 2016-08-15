import serial
import time


# In testing the Python script, it seems the problem is that the Arduino resets when you open the serial port
# (at least the Uno does), so you need to wait a few seconds for it to start up.

# initialize serial in main? and send ser? (Serial is also in currentsensor!)
ser = serial.Serial('/dev/ttyUSB0', 9600)
time.sleep(2)   # wait for Arduino

while 1:
    # ser.write(sdata)
    ser.write('0x1127|y135|')
    # ser.write('4d180|')
    # ser.write('5t8|')
    print(ser.readline())

    # if ser.inWaiting() > 0:
    # bytesToRead = ser.inWaiting()
    # print(ser.inWaiting())
    # print(ser.read(bytesToRead))

    # time.sleep(1)

    # rdata = print(ser.readline())
    # print(ser.readline())

    # return rdata
