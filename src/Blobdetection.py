# Standard imports
import cv2
import numpy as np

im = cv2.imread("../roadtests/trafficsigns.jpg")
im = cv2.resize(im, (0,0), fx=0.5, fy=0.5)

# define range of colors in HSV
lower_blue = np.array([100, 0, 0])
upper_blue = np.array([255, 100, 100])

lower_yellow = np.array([0, 100, 100])
upper_yellow = np.array([50, 255, 255])

lower_red = np.array([0, 0, 100])
upper_red = np.array([50, 50, 255])

# Threshold the HSV image to get only certain colors
maskblue = cv2.inRange(im, lower_blue, upper_blue)
maskyellow = cv2.inRange(im, lower_yellow, upper_yellow)
maskred = cv2.inRange(im, lower_red, upper_red)

# Bitwise-AND mask and original image
blue = cv2.bitwise_and(im, im, mask=maskblue)
yellow = cv2.bitwise_and(im, im, mask=maskyellow)
red = cv2.bitwise_and(im, im, mask=maskred)

thresh = 250
dst = cv2.threshold(maskyellow, thresh, 255, cv2.THRESH_BINARY)[1]

c = cv2.findContours(dst.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

peri = cv2.arcLength(c, True)
approx = cv2.approxPolyDP(c, 0.04 * peri, True)

# if the shape is a triangle, it will have 3 vertices
if len(approx) == 3:
    shape = "triangle"
    print(shape)

# if the shape has 4 vertices, it is either a square or
# a rectangle
elif len(approx) == 4:
    # compute the bounding box of the contour and use the
    # bounding box to compute the aspect ratio
    (x, y, w, h) = cv2.boundingRect(approx)
    ar = w / float(h)

    # a square will have an aspect ratio that is approximately
    # equal to one, otherwise, the shape is a rectangle
    shape = "square" if ar >= 0.95 and ar <= 1.05 else "rectangle"
    print(shape)

# if the shape is a pentagon, it will have 5 vertices
elif len(approx) == 5:
    print("pentagon")

# if the shape is a pentagon, it will have 5 vertices
elif len(approx) == 6:
    print("hexagon")

# otherwise, we assume the shape is a circle
else:
    print("circle")

# Show blobs
cv2.imshow("Blue channel", maskblue)
cv2.imshow('Yellow channel', maskyellow)
cv2.imshow('Red channel', maskred)
cv2.imshow('Blob detected', im)

cv2.waitKey(0)
