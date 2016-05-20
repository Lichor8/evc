# import the necessary packages
from pyimagesearch.shapedetector import ShapeDetector
import imutils
import cv2
import numpy as np

# load the image and resize it to a smaller factor so that
# the shapes can be approximated better
image = cv2.imread("../roadtests/trafficsigns.jpg")


resized = imutils.resize(image, width=400)
ratio = image.shape[0] / float(resized.shape[0])

# define range of colors in HSV
lower_blue = np.array([100, 0, 0])
upper_blue = np.array([255, 100, 100])

lower_yellow = np.array([0, 100, 100])
upper_yellow = np.array([50, 255, 255])

lower_red = np.array([0, 0, 100])
upper_red = np.array([50, 50, 255])

# Threshold the HSV image to get only certain colors
maskblue = cv2.inRange(resized, lower_blue, upper_blue)
maskyellow = cv2.inRange(resized, lower_yellow, upper_yellow)
maskred = cv2.inRange(resized, lower_red, upper_red)

# Bitwise-AND mask and original image
blue = cv2.bitwise_and(resized, resized, mask=maskblue)
yellow = cv2.bitwise_and(resized, resized, mask=maskyellow)
red = cv2.bitwise_and(resized, resized, mask=maskred)

channels = [blue, yellow, red]
# convert the resized image to grayscale, blur it slightly,
# and threshold it

for i in range(0, len(channels)):
    gray = cv2.cvtColor(channels[i], cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (7, 7), 0)
    thresh = cv2.threshold(blurred, 20, 255, cv2.THRESH_BINARY)[1]

    # cv2.imshow("Tresh", thresh)

    # find contours in the thresholded image and initialize the
    # shape detector
    cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = cnts[0] if imutils.is_cv2() else cnts[1]
    sd = ShapeDetector()

    # loop over the contours
    for c in cnts:
        # compute the center of the contour, then detect the name of the
        # shape using only the contour
        M = cv2.moments(c)
        cX = int((M["m10"] / M["m00"]) * ratio)
        cY = int((M["m01"] / M["m00"]) * ratio)
        shape = sd.detect(c, i)

        # multiply the contour (x, y)-coordinates by the resize ratio,
        # then draw the contours and the name of the shape on the image
        if shape is not "none":
            c = c.astype("float")
            c *= ratio
            c = c.astype("int")
            cv2.drawContours(image, [c], -1, (0, 255, 0), 2)
            cv2.putText(image, shape, (cX, cY), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

            # show the output image
            cv2.imshow("Image", image)

cv2.waitKey(0)
