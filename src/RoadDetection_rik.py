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

# import test image
imgP = cv2.imread('../roadtests/road0.jpg')
img = cv2.imread('../roadtests/road0.jpg')

# blur image using a gaussian blur
imgBlurred = cv2.GaussianBlur(img, (5, 5), 0)

# define range of grey color in RGB
lower_gray = np.array([220, 220, 220])
upper_gray = np.array([250, 250, 250])

# create grayscale image
imgGrayscale = cv2.inRange(imgBlurred, lower_gray, upper_gray)

# detect edges in image using canny
# imgCanny = cv2.Canny(imgGrayscale, 50, 150, apertureSize=3)

# show all steps as images
cv2.imshow('grayscaled', imgGrayscale)
cv2.imshow('imgblurred', imgBlurred)
# cv2.imshow('imgcanny', imgCanny)

# set heuristic parameters for the (probabilistic) houghlines function
rho = 1                 # accuracy [pixel]
acc = 1                 # accuracy [deg]
theta = acc*np.pi/360
threshold = 10          #
minLineLength = 10      # line segments shorter than this are rejected
maxLineGap = 20         # maximum allowed gap between line segments to treat them as single line

# find lines in image using the (probabilistic) houghlines function
# outputP:  x1, y1, x2, y2
# output:   rho, theta
linesP = cv2.HoughLinesP(imgGrayscale, rho, theta, threshold, minLineLength, maxLineGap)
lines = cv2.HoughLines(imgGrayscale, rho, theta, threshold)

# check if houghlinesP function found any lines, if not print error
if linesP is not None:
    exitflag = 1   # houghlines function found at least one line

    # plot red(0, 0, 255) houghlines in image
    for x in range(0, len(linesP)):
        for x1, y1, x2, y2 in linesP[x]:
            cv2.line(imgP, (x1, y1), (x2, y2), (0, 0, 255), 2)
else:
    exitflag = 0    # houghlinesP function found no lines (error)
    print("error: no lines detected\n")

# plot lines in original image
cv2.imshow('houghlinesP', imgP)

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
            x1 = int(x0 + 1000*(-b))
            y1 = int(y0 + 1000*(a))
            x2 = int(x0 - 1000*(-b))
            y2 = int(y0 - 1000*(a))

            cv2.line(img, (x1, y1), (x2, y2), (0, 0, 255), 2)
else:
    exitflag = 0  # houghlines function found no lines (error)
    print("error: no lines detected\n")

# plot lines in original image
cv2.imshow('houghlines', img)

print("exitflag\n", exitflag)

# press any key to terminate process
cv2.waitKey(0)
cv2.destroyAllWindows()
