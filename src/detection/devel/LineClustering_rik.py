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

# blur image using a gaussian blur
imgBlurred = cv2.GaussianBlur(img, (15, 15), 0)
gray_image = cv2.cvtColor(imgBlurred, cv2.COLOR_BGR2GRAY)

hist, bins = np.histogram(img.ravel(), 256, [0, 256])

grayval = np.argmax(hist)

thresh = grayval - 60
th, dst = cv2.threshold(gray_image, thresh, 255, cv2.THRESH_BINARY)

# detect edges in image using canny
imgCanny = cv2.Canny(dst, 50, 150, apertureSize=3)

# region of interest (roi) directly in front of vehicle
imgShape = img.shape
imgSize.x = imgShape[1]     # save the size of the image in pixels
imgSize.y = imgShape[0]
roiSize.x = 50              # define size of roi window in percentage of total image size
roiSize.y = 75
roiGap.x = int(0.5*(imgSize.x - roiSize.x*imgSize.x/100))   # gap calculated as even spaces from image borders
roiGap.y = int(imgSize.y - roiSize.y*imgSize.y/100)         # gap calculated as from the top of the image

roi = imgCanny[roiGap.y:imgSize.y, roiGap.x:imgSize.x - roiGap.x]       # get roi from image
imgRoi = np.zeros((imgSize.y, imgSize.x), np.uint8)                 # create new binary black image
imgRoi[roiGap.y:imgSize.y, roiGap.x:imgSize.x - roiGap.x] = roi     # add roi to black image

# show all steps as images
# cv2.imshow('grayscaled', gray_image)
# cv2.imshow('imgblurred', imgBlurred)
# cv2.imshow('TRESHOLD', dst)
# cv2.imshow('imgcanny', imgCanny)
cv2.imshow('ROI', imgRoi)
# cv2.imshow('ORIGINAL', img)

# set heuristic parameters for the (probabilistic) houghlines function
rho = 1                 # accuracy [pixel]
acc = 1                 # accuracy [deg]
theta = acc*np.pi/360
threshold = 50          # minimum points on a line

# find lines in image using the (probabilistic) houghlines function
# outputP:  x1, y1, x2, y2
# output:   rho, theta
lines = cv2.HoughLines(imgRoi, rho, theta, threshold)

# check if houghlines function found any lines, if not print error
if lines is not None:
    exitflag = 1  # houghlines function found at least one line

    # plot red(0, 0, 255) houghlines in image
    for x in range(0, len(lines)):
        for rho, theta in lines[x]:
            a = np.cos(theta)
            b = np.sin(theta)
            x0 = a*rho
            y0 = b*rho
            x1 = int(x0 + 2000*(-b))
            y1 = int(y0 + 2000*(a))
            x2 = int(x0 - 2000*(-b))
            y2 = int(y0 - 2000*(a))
            cv2.line(img, (x1, y1), (x2, y2), (0, 0, 255), 2)
else:
    exitflag = 0  # houghlines function found no lines (error)
    print("error: no lines detected\n")

# plot lines in original image
cv2.imshow('houghlines', img)

# LineClustering()
# get values rho and theta from houghlines
rho = [line[0][0] for line in lines]
theta = [line[0][1] for line in lines]

# create histogram with n bins (possible values) from 0 to pi (range of theta)
bins = 60       # 6 graden
thhist, thbins = np.histogram(theta, bins, [0, np.pi])
# thvalue = np.arg(thhist[0])
print(thhist)
print(thbins)

# cluster lines based on similar theta using histogram data
robustlines = []
cluster_th = []                 # clustered theta
srange = 10                     # search range for finding peaks in histogram
for i in range(0, len(thhist)):
    if thbins[i-1] >= 0:
        if thhist[i-1] > thhist[i]:
            angle = thbins[i-1]*np.pi/bins
            cluster_th.append(angle)


print("exitflag\n", exitflag)

# press any key to terminate process
cv2.waitKey(0)
cv2.destroyAllWindows()
