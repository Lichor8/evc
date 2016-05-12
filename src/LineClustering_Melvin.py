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
# cv2.imshow('imgcanny', imgCanny)
# cv2.imshow('ROI', imgRoi)
# cv2.imshow('ORIGINAL', img)
# cv2.imshow('TRESHOLD', dst)

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

    xvalues = []
    intersects = []

    # Get values rho and theta from houghlines
    rho = [line[0][0] for line in lines]
    theta = [line[0][1] for line in lines]

    # Create histogram with n bins (possible values) from 0 to pi (range of theta)
    bins = 60
    thhist, thbins = np.histogram(theta, bins, [0, np.pi])

    # Find angles that occur more than the treshold and save that angle
    treshold = 3
    histangles = []
    for i in range(0, len(thhist)):
        if np.amax(thhist) > treshold:
            anglepos = np.argmax(thhist)
            angle = anglepos * np.pi / bins
            thhist[anglepos] = 0
            if anglepos - 1 >= 0:
                thhist[anglepos - 1] = 0
            if anglepos + 1 <= len(thhist):
                thhist[anglepos + 1] = 0
                histangles.append(angle)

    all_angles = [[] for _ in range(len(histangles))]
    tres = np.pi/bins

    #plot red(0, 0, 255) houghlines in image
    for x in range(0, len(lines)):
        for rho, theta in lines[x]:
            a = np.cos(theta)
            b = np.sin(theta)
            x0 = a*rho
            y0 = b*rho
            x1 = int(x0 + 1000*(-b))
            y1 = int(y0 + 1000*(a))
            x2 = int(x0 - 1000*(-b))
            y2 = int(y0 - 1000*(a))
            cv2.line(img, (x1, y1), (x2, y2), (0, 0, 255), 2)
            #Compare theta with the histogram angle, if within certain interval, store in all_angles
            for i in range(0, len(histangles)):
                if histangles[i]-tres < theta and theta < histangles[i]+tres:
                    all_angles[i].append([theta, rho])

    rhovalues = [[] for _ in range(len(all_angles))]

    for i in range (0, len(all_angles)):
        rho = [x[1] for x in all_angles[i]]
        bins = 12
        rhohist, rhobins = np.histogram(rho, bins, [np.amin(rho)-100, np.amax(rho)+100])

        treshold = 3

        for j in range(0, len(rhohist)):
            if np.amax(rhohist) > treshold:
                rhopos = np.argmax(rhohist)
                rho = rhobins[rhopos]
                rhohist[rhopos] = 0
                if rhopos - 1 >= 0:
                    rhohist[rhopos - 1] = 0
                if rhopos + 1 <= len(rhohist):
                    rhohist[rhopos + 1] = 0
                    rhovalues[i].append(rho)

    line_amount = 0
    for k in range (0, len(rhovalues)):
        line_amount = line_amount + len(rhovalues[k])

    all_values = [[] for _ in range(line_amount)]

    tres = 50
    extra_line = 0

    for x in range(0, len(lines)):
        for rho, theta in lines[x]:
            for i in range(0, len(rhovalues)):
                if len(rhovalues[i]) > 1:                       #Split array into two lines if multiple rho values
                    for j in range(0, len(rhovalues[i])):
                        if  rhovalues[i][j]-tres < rho and rho < rhovalues[i][j]+tres:
                            all_values[j].append([theta, rho])
                    extra_line = 1

    for i in range(0, len(rhovalues)):
        if len(rhovalues[i]) == 1:
            all_values[i+extra_line] = all_angles[i]

    print(all_values[0])
    print(all_values[1])
    print(all_values[2])
    print(all_values[3])

    #TODO: make sure the script works for splitting into 3 lines as well and that all values are inserted into all_values with the right index

else:
    exitflag = 0  # houghlines function found no lines (error)
    print("error: no lines detected\n")

#plot lines in original image
# cv2.imshow('houghlines', img)
# cv2.imshow('Intersections', intersectimg)

print("exitflag\n", exitflag)

# press any key to terminate process
cv2.waitKey(0)
cv2.destroyAllWindows()

# L1 = line([x1, y1], [x2, y2])
# L2 = line([x1, y1], [x2, y2])
# R = intersection(L1, L2)
# if R:
#     cv2.circle(intersectimg, (int(R[0]), int(R[1])), 1, (0, 0, 255), 2)
