########################################################################################################################
# lanedetection_vid()
# detects the boundaries of a road and creates the wanted trajectory
#
# input:    []
# output:   [, exitflag]
########################################################################################################################

# import necessary libraries
from __future__ import division
import cv2
import numpy as np
import imutils
import math
# from collections import namedtuple

# define classes
class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y
# Point = namedtuple('Point', 'y, x')   # point in cartesian coordinates

# create instance of namedtuples and initialize
imgSize = Point(0, 0)
roiSize = Point(0, 0)
roiGap = Point(0, 0)

cap = cv2.VideoCapture('/home/pi/evc/roadtests/video3_hd_lines.mp4')

while cap.isOpened():
    ret, frame = cap.read()

    imgShape = frame.shape
    imgSize.x = imgShape[1]  # save the size of the image in pixels
    imgSize.y = imgShape[0]

    x_off = 0.37*imgSize.x
    x1 = x_off
    y1 = 0.75*imgSize.y
    x2 = imgSize.x - x_off
    y2 = y1
    x3 = 0
    y3 = imgSize.y
    x4 = imgSize.x
    y4 = imgSize.y

    # rows, cols, ch = img.shape
    W = 720
    H = 480

    ratio = imgSize.x/W

    pts1 = np.float32([[x1, y1], [x2, y2], [x3, y3], [x4, y4]])  # TL, TR, BL, BR
    pts2 = np.float32(
        [[int(0.4 * W), int(0.5 * H)], [int(0.6 * W), int(0.5 * H)], [int(0.4 * W), H], [int(0.6 * W), H]])

    M = cv2.getPerspectiveTransform(pts1, pts2)
    dst = cv2.warpPerspective(frame, M, (W, H))

    avghough = dst.copy()

    imgBlurred = cv2.GaussianBlur(dst, (5, 5), 0)
    gray_image = cv2.cvtColor(imgBlurred, cv2.COLOR_BGR2GRAY)

    # thresh = 160
    imgCanny = cv2.Canny(gray_image, 150, 250, apertureSize=3)

    cv2.imshow('Canny', imgCanny)

    imgShape = imgCanny.shape
    imgSize.x = imgShape[1]  # save the size of the image in pixels
    imgSize.y = imgShape[0]
    roiSize.x = 100  # define size of roi window in percentage of total image size
    roiSize.y = 40
    roiGap.x = int(0.5 * (imgSize.x - roiSize.x * imgSize.x / 100))  # gap calculated as even spaces from image borders
    roiGap.y = int(imgSize.y - roiSize.y * imgSize.y / 100)  # gap calculated as from the top of the image

    roi = imgCanny[roiGap.y:imgSize.y, roiGap.x:imgSize.x - roiGap.x]  # get roi from image
    imgRoi = np.zeros((imgSize.y, imgSize.x), np.uint8)  # create new binary black image
    imgRoi[roiGap.y:imgSize.y, roiGap.x:imgSize.x - roiGap.x] = roi  # add roi to black image

    cv2.imshow('Canny ROI', imgRoi)

    # set heuristic parameters for the (probabilistic) houghlines function
    rho = 1     # accuracy [pixel]
    acc = 0.3   # accuracy [deg]
    theta = acc * np.pi / 360
    threshold = int(200 / ratio)  # minimum points on a line

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
        treshold = 0
        histangles = []  # Array for storing all angles from the histogram
        for i in range(0, len(thhist)):
            if np.amax(thhist) > treshold:  # Found feasible maximum
                anglepos = np.argmax(thhist)  # Get position of maximum
                angle = anglepos * np.pi / bins  # Calculate value of maximum
                thhist[anglepos] = 0  # Set histogram value to 0, so we won't find this maximum next iteration
                if anglepos - 1 >= 0:  # Check if the position is in bounds of the histogram
                    thhist[anglepos - 1] = 0  # Set the neighbour values to zero as well
                if anglepos + 2 <= len(thhist):
                    thhist[anglepos + 1] = 0
                if angle < 0.2*np.pi or angle > 0.8*np.pi:
                    histangles.append(angle)  # Append theta value of the peak to angle

        all_angles = [[] for _ in range(len(histangles))]  # Make an array with n arrays, for all found angles from hist
        tres = np.pi / bins  # Define a certain treshold

        # plot red(0, 0, 255) houghlines in image
        for x in range(0, len(lines)):
            for rho, theta in lines[x]:
                a = np.cos(theta)
                b = np.sin(theta)
                x0 = a * rho
                y0 = b * rho
                x1 = int(x0 + 1000 * (-b))
                y1 = int(y0 + 1000 * a)
                x2 = int(x0 - 1000 * (-b))
                y2 = int(y0 - 1000 * a)
                cv2.line(dst, (x1, y1), (x2, y2), (0, 0, 255), 2)
                # Compare theta with the histogram angle, if within certain interval, store in all_angles
                for i in range(0, len(histangles)):
                    if histangles[i] - tres < theta < histangles[i] + tres:
                        all_angles[i].append([theta, rho])

        rhovalues = [[] for _ in range(len(all_angles))]  # [[rho 1, ~rho 1, ...], [rho2, ~rho2, ...], ...]

    for i in range(0, len(all_angles)):
        rho = [x[1] for x in all_angles[i]]
        bins = 24  # Number of bins for histogram of rho
        # Histogram of rho (grouped per theta), from the minimum to the maximum of the rho set plus an offset of 200px
        # This is done so we only see one 'peak' if there is one line instead of a roughly evenly distributed histogram
        rhohist, rhobins = np.histogram(rho, bins, [np.amin(rho) - 100, np.amax(rho) + 100])

        treshold = 0

        for j in range(0, len(rhohist)):
            if np.amax(rhohist) > treshold:  # Value must be found more than n times to be a feasible line
                rhopos = np.argmax(rhohist)  # Find position of the maximum rho value in the histogram
                rho = rhobins[rhopos]  # Find maximum value of rho
                rhohist[rhopos] = 0  # Set histogram value to 0, so we won't find this maximum next iteration
                if rhopos - 1 >= 0:  # Check if the position is in bounds of the histogram
                    rhohist[rhopos - 1] = 0  # Set the neighbour values to zero as well
                if rhopos + 1 <= len(rhohist):
                    rhohist[rhopos + 1] = 0
                rhovalues[i].append(rho)  # Append rho value of the peak to rhovalues

    line_amount = 0
    for k in range(0, len(rhovalues)):  # Count how many different peaks of the rho histogram there are
        line_amount += len(rhovalues[k])  # This is equal to the amount of lines

    all_values = [[] for _ in range(line_amount)]

    print(line_amount)

    tres = int(40 / ratio)
    extra_line = 0
    line_counter = -1

    for i in range(0, len(rhovalues)):  # Go through rhovalues e.g. [[218.5, 379.3], [595.5], [-627.0]]
        if len(rhovalues[i]) > 1:  # Split array into multiple lines if multiple rho values
            for j in range(0, len(rhovalues[i])):  # Go through all rhovalues[i] e.g. [218.5, 379.3]
                line_counter += 1  # Counter of current line we are filling
                for x in range(0, len(lines)):  # Go through all found houghlines
                    for rho, theta in lines[x]:  # Set rho and theta
                        if rhovalues[i][j] - tres < rho < rhovalues[i][j] + tres:  # If rho is within treshold
                            all_values[line_counter].append([theta, rho])  # Append to all_values
        else:
            line_counter += 1
            all_values[line_counter] = all_angles[i]

    avg_line_values = [[] for _ in range(line_amount)]

    for i in range(0, line_amount):
        avg_line_values[i] = [(sum([item[0] for item in all_values[i]]) / len(all_values[i])),
                              (sum([item[1] for item in all_values[i]]) / len(all_values[i]))]

    for x in range(0, len(avg_line_values)):
        rho = avg_line_values[x][1]
        theta = avg_line_values[x][0]
        a = np.cos(theta)
        b = np.sin(theta)
        x0 = a * rho
        y0 = b * rho
        x1 = int(x0 + 2000 * (-b))
        y1 = int(y0 + 2000 * (a))
        x2 = int(x0 - 2000 * (-b))
        y2 = int(y0 - 2000 * (a))
        # x3 = int(imgSize.x / 2)
        # y3 = imgSize.y
        # x4 = int(x3 + 2 * math.tan(theta))
        # y4 = imgSize.y - 20 - int(1000 / math.tan(theta))
        cv2.line(avghough, (x1, y1), (x2, y2), (0, 0, 255), 2)
        # cv2.line(avghough, (x3, y3), (x4, y4), (255, 0, 0), 2)

    cv2.imshow('DST', dst)
    cv2.imshow('avghough', avghough)
    print(avg_line_values)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
