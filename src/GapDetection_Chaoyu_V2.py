########################################################################################################################
# RoadDetection()
# detects the boundaries of a road and creates the middle line for this road
#
# input:    []
# output:   [, exitflag]
########################################################################################################################

# import necessary libraries
from __future__ import division
import cv2
import numpy as np
# from collections import namedtuple

# define classes
class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y
# Point = namedtuple('Point', 'y, x')   # point in cartesian coordinates

def line(p1, p2):
    A = (p1[1] - p2[1])
    B = (p2[0] - p1[0])
    C = (p1[0]*p2[1] - p2[0]*p1[1])
    return A, B, -C

def intersection(L1, L2):
    D  = L1[0] * L2[1] - L1[1] * L2[0]
    Dx = L1[2] * L2[1] - L1[1] * L2[2]
    Dy = L1[0] * L2[2] - L1[2] * L2[0]
    if D != 0:
        x = Dx / D
        y = Dy / D
        return x,y
    else:
        return False
# create instance of namedtuples and initialize
imgSize = Point(0, 0)
roiSize = Point(0, 0)
roiGap = Point(0, 0)

# import test image
img = cv2.imread('../roadtests/cornerrightinsane.jpg')
intersectimg = img

# blur image using a gaussian blur
imgBlurred = cv2.GaussianBlur(img, (15, 15), 0)
gray_image = cv2.cvtColor(imgBlurred, cv2.COLOR_BGR2GRAY)

hist, bins = np.histogram(img.ravel(), 256, [0, 256])

grayval = np.argmax(hist)

thresh = grayval - 60
th, dst = cv2.threshold(gray_image, thresh, 255, cv2.THRESH_BINARY)

# detect edges in image using canny
imgCanny = cv2.Canny(dst, 50, 150, apertureSize=3)
################################################################################################################
# Calculates the concentration at a corner with roi and based on the concentration, a gap is given.
################################################################################################################

# region of interest (roi) at corners
imgShape = img.shape        # show the resolution of the image
imgSize.x = imgShape[1]     # save the size of the image in pixels, this is used to create a black image
imgSize.y = imgShape[0]

receivedpos_x= 1200          # x_position of the intersection
receivedpos_y = 300
roirangex = int(0.05*imgSize.x)                # the window range of roi, it is currently choose as 100 pixel width
roirangey = int(0.08*imgSize.y)
dstinv=np.invert(dst)
roi = dstinv[(receivedpos_y-roirangey):(receivedpos_y+roirangey),(receivedpos_x-roirangex):(receivedpos_x+roirangex) ]       # get roi from image
imgRoi = np.zeros((imgSize.y, imgSize.x), np.uint8)                 # create new binary black image
imgRoi[(receivedpos_y-roirangey):(receivedpos_y+roirangey),(receivedpos_x-roirangex):(receivedpos_x+roirangex)] = roi     # add roi to black image

roiShape = roi.shape        # show the resolution of the roi
roiSize_x = roiShape[1]     # save the size of the image in pixels, this is used to count the concentration in the roi
roiSize_y = roiShape[0]
# print("roishape \n",roiShape,"roix roiy \n", roiSize_x,roiSize_y)

concentration_roi = 0       # initial concentration,
for i in range(0,roiSize_y):        # loop over all entries of roi
    for j in range(0,roiSize_x):
        if roi[i][j]==255:          # if a white pixel is found thus value 255, then the concentration will become 1 higher
            concentration_roi=concentration_roi+1
# print("totalvalue imgcanny \n",concentration_roi)
FoundGap = 0
if concentration_roi < 0.001*(roiSize_x * roiSize_y):  # the concentration of white should be at least 50% of the roi windowsize
        FoundGap=1                                     # This means that the white value is lower than the limit thus the solid line is less than 0.1% in the roi window, thus there is probably no solid line hence a gap
print("Foundgap \n",FoundGap)
print("Concentration \n",concentration_roi)

# the upperborder
xu1 = receivedpos_x - roirangex
yu1 = receivedpos_y - roirangey
xu2 = receivedpos_x + roirangex
yu2 = receivedpos_y - roirangey
cv2.line(img, (xu1, yu1), (xu2, yu2), (0, 255, 0), 2)
# the leftborder
xl1 = receivedpos_x - roirangex
yl1 = receivedpos_y - roirangey
xl2 = receivedpos_x - roirangex
yl2 = receivedpos_y + roirangey
cv2.line(img, (xl1, yl1), (xl2, yl2), (0, 255, 0), 2)
# the  downborder
xd1 = receivedpos_x - roirangex
yd1 = receivedpos_y + roirangey
xd2 = receivedpos_x + roirangex
yd2 = receivedpos_y + roirangey
cv2.line(img, (xd1, yd1), (xd2, yd2), (0, 255, 0), 2)
# the rightborder
xr1 = receivedpos_x + roirangex
yr1 = receivedpos_y - roirangey
xr2 = receivedpos_x + roirangex
yr2 = receivedpos_y + roirangey
cv2.line(img, (xr1, yr1), (xr2, yr2), (0, 255, 0), 2)
cv2.imshow('border', img)
##################################################################################################


# show all steps as images
# cv2.imshow('grayscaled', gray_image)
# cv2.imshow('imgblurred', imgBlurred)
# cv2.imshow('imgcanny', imgCanny)
cv2.imshow('ROI', imgRoi)
# cv2.imshow('ORIGINAL', img)
cv2.imshow('TRESHOLD', dst)
cv2.imshow('TRESHOLD', dstinv)
# cv2.imshow('Intersections', img)

# press any key to terminate process
cv2.waitKey(0)
cv2.destroyAllWindows()