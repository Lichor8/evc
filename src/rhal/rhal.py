# import OpenCV functions
# import cv2

# import Numpy functions
# import numpy as np

# import libraries
# from libs import defs

# import computational entities (functions)
# from rhal import currentsensor
from rhal import rpiarduinocom as rpia
from rhal import io


class Rhal:
    # initialize class object for the first time
    def __init__(self):

        # initialize io
        self.ser = []

        # rpi_arduino_com (inputs/outputs)
        self.rdata = []
        self.sdata = []

    # monitor
    # def monitor(self):
        # return self.status

    # configurator
    # def configurator(self, something):
        # self.linecl.setsomething(something)

    def io(self):
        self.ser = io.initialize()

    # setters
    def setsdata(self, sdata):
        self.sdata = sdata

    # getters
    def get_rdata(self):
        return self.rdata

    # coordinator/composer
    def rhal(self):
        # currentsensor.currentsensor()

        # rpi arduino communication
        rpia.rpi2arduino(self.ser, self.sdata)    # setter?
        self.rdata = rpia.arduino2rpi(self.ser)
