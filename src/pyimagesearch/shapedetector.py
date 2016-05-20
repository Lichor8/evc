# import the necessary packages
import cv2


class ShapeDetector:
    def __init__(self):
        pass

    def detect(self, c, channel):
        # initialize the shape name and approximate the contour
        peri = cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, 0.04 * peri, True)
        shape = "none"
        # if the shape is a triangle, it will have 3 vertices

        # if the shape has 4 vertices, it is either a square or
        # a rectangle
        if len(approx) == 4 and channel == 1:
            # compute the bounding box of the contour and use the
            # bounding box to compute the aspect ratio
            (x, y, w, h) = cv2.boundingRect(approx)
            ar = w / float(h)

            # a square will have an aspect ratio that is approximately
            # equal to one, otherwise, the shape is a rectangle

            if ar >= 0.95 and ar <= 1.05:
                shape = "u-turn"

        elif len(approx) == 8 and channel == 2:
            shape = "stop"

        # otherwise, we assume the shape is a circle
        elif channel == 0:
            shape = "direction"

        # return the name of the shape
        return shape
