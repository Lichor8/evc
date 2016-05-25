# import the necessary packages
import cv2
import numpy as np

class ShapeDetector:
    def __init__(self):
        pass

    def detect(self, c, channel):
        # initialize the shape name and approximate the contour
        area = cv2.contourArea(c)
        areamin = 1500
        areamax = 10000
        peri = cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, 0.04 * peri, False)
        shape = "none"
        # if the shape is a triangle, it will have 3 vertices

        # if the shape has 4 vertices, it is either a square or
        # a rectangle
        if len(approx) == 4 and channel == 1 and areamin < area < areamax:
            shape = "uturn"

        elif len(approx) >= 5 and channel == 2 and areamin < area < areamax:
            shape = "stop"

        # otherwise, we assume the shape is a circle
        elif channel == 0 and areamin < area < areamax:
            shape = "dir-candidate"

        # return the name of the shape
        return shape
