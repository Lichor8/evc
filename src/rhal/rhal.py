# import OpenCV functions
# import cv2

# import Numpy functions
# import numpy as np

# import libraries
# from libs import defs

# import computational entities (functions)
from rhal import currentsensor

class Rhal:

    # initialize class object for the first time
    def __init__(self):

        # gapdetection (inputs/outputs)
        self.gappos = []

        # lineclustering (inputs/outputs)

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
    # def getgappos(self):
        # return self.gappos

    # coordinator/composer
    def rhal(self):
        currentsensor.currentsensor()
