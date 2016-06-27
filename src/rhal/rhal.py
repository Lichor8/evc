# import OpenCV functions
# import cv2

# import Numpy functions
# import numpy as np

# import libraries
# from libs import defs

# import computational entities (functions)
from rhal import currentsensor
from rhal import rpiarduinocom as rpia


class Rhal:
    # initialize class object for the first time
    def __init__(self):

        # rpi_arduino_com (inputs/outputs)
        self.rdata = []
        self.sdata = []

    # monitor
    # def monitor(self):
        # return self.status

    # configurator
    # def configurator(self, something):
        # self.linecl.setsomething(something)

    # setters
    # def setsomethinglincl(self, something):
        # self.linecl.setsomething(something)

    # getters
    def get_rdata(self):
        return self.rdata

    # coordinator/composer
    def rhal(self):
        placeholder = self.rdata
        # currentsensor.currentsensor()

        # rpi arduino communication
        # rpia.rpi2arduino(self.sdata)    # setter?
        # self.rdata = rpia.arduino2rpi()
