# import OpenCV functions
import cv2

# import Numpy functions
import numpy as np

# import libraries
# from libs import defs

# import computational entities (functions)
from detection import lanedetection
from detection import detectshapes
from detection import gapdetection
# from detection import lineclustering

class Detection:

    # initialize class object for the first time
    def __init__(self):
        self.status = 1
        self.line = 0
        # from lanedetection
        self.target = []
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
    def getgappos(self):
        return self.gappos

    def gettarget(self):
        return self.target

    # coordinator/composer
    # def detection(self):
        # self.linecl.lineclustering()
        # self.line = self.linecl.getline()

        # get camera images

        # run computational entities and retrieve outputs
        # receivedpos_x = [1200, 600, 200, 100]
        # receivedpos_y = [230, 230, 100, 80]
        # receivedpos = [receivedpos_x, receivedpos_y]

        # self.gappos = gapdetection.gapdetection(img, receivedpos)

        # detection
        # detectshapes.detectshapes()
        # cap = cv2.VideoCapture('../roadtests/video3_hd_lines.mp4')
        # while cap.isOpened():
        #     ret, frame = cap.read()
        #     target = lanedetection.lanedetection(frame)
        #     print(target)
        #     if cv2.waitKey(1) & 0xFF == ord('q'):
        #         break
        # cap.release()
        # cv2.destroyAllWindows()
        # print(self.target)
