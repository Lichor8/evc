import serial
import time


def initialize():
    # In testing the Python script, it seems the problem is that the Arduino resets when you open the serial port
    # (at least the Uno does), so you need to wait a few seconds for it to start up.

    # initialize serial in main? and send ser? (Serial is also in currentsensor!)
    ser = serial.Serial('/dev/ttyUSB0', 9600)
    time.sleep(2)  # wait for Arduino

    return ser


# def ok():

    # return 1
