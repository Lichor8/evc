########################################################################################################################
# RoadDetection()
# detects the boundaries of a road and creates the middle line for this road
#
# input:    []
# output:   [, exitflag]
########################################################################################################################

# import necessary libraries
import cv2
import numpy as np

# define classes
class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y
# Point = namedtuple('Point', 'y, x')   # point in cartesian coordinates

# create instance of namedtuples and initialize
imgSize = Point(0, 0)

# import test image
img = cv2.imread('../roadtests/cornerrightinsane.jpg')

imgShape = img.shape
imgSize.x = imgShape[1]     # save the size of the image in pixels
imgSize.y = imgShape[0]

# blur image using a gaussian blur
imgBlurred = cv2.GaussianBlur(img, (19, 19), 0)
gray_image = cv2.cvtColor(imgBlurred, cv2.COLOR_BGR2GRAY)

hist, bins = np.histogram(img.ravel(), 256, [0, 256])

grayval = np.argmax(hist)

print(grayval)

thresh = grayval - 60
th, dst = cv2.threshold(gray_image, thresh, 255, cv2.THRESH_BINARY)

cv2.imshow('ORIGINAL', img)
cv2.imshow('TRESHOLD', dst)

# press any key to terminate process
cv2.waitKey(0)
cv2.destroyAllWindows()
