# import the necessary packages
from pyimagesearch.shapedetector import ShapeDetector
import imutils
import cv2
import numpy as np

# load the image and resize it to a smaller factor so that
# the shapes can be approximated better
# image = cv2.imread("../roadtests/uturn.png")
image = cv2.imread("../roadtests/stopndirection.png")

resized = imutils.resize(image, width=500)
ratio = image.shape[0] / float(resized.shape[0])

hsv = cv2.cvtColor(resized, cv2.COLOR_BGR2HSV)

# define range of colors in HSV
lower_blue = np.array([100, 120, 120])
upper_blue = np.array([140, 255, 255])

lower_yellow = np.array([15, 100, 100])
upper_yellow = np.array([25, 255, 255])

lower_red = np.array([0, 120, 120])
upper_red = np.array([10, 255, 255])
lower_red2 = np.array([160, 120, 120])
upper_red2 = np.array([180, 255, 255])

# Threshold the HSV image to get only certain colors
maskblue = cv2.inRange(hsv, lower_blue, upper_blue)
maskyellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
maskred1 = cv2.inRange(hsv, lower_red, upper_red)
maskred2 = cv2.inRange(hsv, lower_red2, upper_red2)
maskred = maskred1 + maskred2

masks = [maskblue, maskred, maskyellow]

for i in range (0, len(masks)):
    # Copy the thresholded image.
    im_floodfill = masks[i].copy()
    # Mask used to flood filling.
    # Notice the size needs to be 2 pixels than the image.
    h, w = masks[i].shape[:2]
    mask = np.zeros((h+2, w+2), np.uint8)
    # Floodfill from point (0, 0)
    cv2.floodFill(im_floodfill, mask, (0, 0), 255)
    # Invert floodfilled image
    im_floodfill_inv = cv2.bitwise_not(im_floodfill)
    # Combine the two images to get the foreground.
    masks[i] = masks[i] | im_floodfill_inv

# Bitwise-AND mask and original image
blue = cv2.bitwise_and(resized, resized, mask=masks[0])
red = cv2.bitwise_and(resized, resized, mask=masks[1])
yellow = cv2.bitwise_and(resized, resized, mask=masks[2])

# cv2.imshow("Red", red)
# cv2.imshow("Blue", blue)
# cv2.imshow("Yellow", yellow)

channels = [blue, yellow, red]
# convert the resized image to grayscale, blur it slightly,
# and threshold it

for i in range(0, len(channels)):
    gray = cv2.cvtColor(channels[0], cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (1, 1), 0)
    thresh = cv2.threshold(blurred, 50, 255, cv2.THRESH_BINARY)[1]

    cv2.imshow("Tresh", thresh)
    # cv2.waitKey(0)

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
        if M["m00"] > 100:
            cX = int((M["m10"] / M["m00"]) * ratio)
            cY = int((M["m01"] / M["m00"]) * ratio)
            shape = sd.detect(c, i)

            # multiply the contour (x, y)-coordinates by the resize ratio,
            # then draw the contours and the name of the shape on the image
            if shape is not "none":
                c = c.astype("float")
                c *= ratio
                print(c)
                c = c.astype("int")
                cv2.drawContours(image, [c], -1, (0, 255, 0), 2)
                cv2.putText(image, shape, (cX, cY), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

# show the output image
cv2.imshow("Image", image)
cv2.waitKey(0)
