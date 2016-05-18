import cv2
import numpy as np

# import computational entities functions
from detection import lineclustering
from detection import gapdetection


class Detection:

    # initialize class object for the first time
    def __init__(self):
        self.status = 1
        self.line = 0

        # concentrationcheck
        self.foundcorner = 0
        self.foundgap = 0
        self.location_x = 0
        self.location_y = 0

        # create object for each computational entity class
        self.linecl = lineclustering.LineClustering()
        # self.concheck = concentrationcheck.concentrationcheck()

    # monitor
    def monitor(self):
        return self.status

    # configurator
    def configurator(self, something):
        self.linecl.setsomething(something)

    # setters
    def setsomethinglincl(self, something):
        self.linecl.setsomething(something)

    # getters
    def getline(self):
        return self.line

    # coordinator/composer
    def detection(self):
        # get camera images


        # run computational entities and retrieve outputs
        self.linecl.lineclustering()
        self.line = self.linecl.getline()
        gapdetection.gapdetection()


