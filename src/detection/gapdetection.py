# import OpenCV functions
import cv2

# import Numpy functions
import numpy as np

# import computational entities (functions)
from detection import concentrationcheck


def gapdetection():
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

    imgShape = img.shape  # show the resolution of the image
    imgSize_x = imgShape[1]  # save the size of the image in pixels, this is used to create a black image
    imgSize_y = imgShape[0]
    imgRoi = np.zeros((imgSize_y, imgSize_x), np.uint8)
    receivedpos_x = [1200, 600, 200, 100]
    receivedpos_y = [230, 230, 100, 80]
    Location_xo = []
    Location_yo = []
    imgRoi = np.zeros((imgSize_y, imgSize_x), np.uint8)

    # this for loop takes every valid corner position and save it in Location
    for i in range(0, len(receivedpos_x)):
        [Foundcorner, Foundgap, Location_x, Location_y] = \
            concentrationcheck.concentrationcheck(receivedpos_x[i], receivedpos_y[i], imgSize_x, imgSize_y, imgRoi, dst)

        Location_xo.append(Location_x)
        Location_yo.append(Location_y)

    print("Reallocx \n Reallocy \n", Location_xo, Location_yo)
    # print("Reallocx2 Reallocy2 \n",len(Location_xo),len(Location_yo))

    # This for loop places roi between the intersection
    for i in range(0, len(Location_xo)):
        if i + 1 == len(Location_xo):
            receivedpos_x = int(sum(Location_xo[-1] + Location_xo[0]) / 2)
            receivedpos_y = int(sum(Location_yo[-1] + Location_yo[0]) / 2)

            [Foundcorner, Foundgap, Location_x, Location_y] = \
                concentrationcheck.concentrationcheck(receivedpos_x, receivedpos_y, imgSize_x, imgSize_y, imgRoi, dst)

            # print("receivedpos_x, receivedpos_x-1,receivedpos_x0 \n", receivedpos_x,Location_xo[-1],Location_xo[0])
        elif i + 1 < len(Location_xo):
            receivedpos_x = int(sum(Location_xo[i] + Location_xo[i + 1]) / 2)
            receivedpos_y = int(sum(Location_yo[i] + Location_yo[i + 1]) / 2)
            print("receivedpos_x, receivedpos_x-1,receivedpos_x0 \n", receivedpos_x, Location_xo[i],
                  Location_xo[i + 1])

            [Foundcorner, Foundgap, Location_x, Location_y] = \
                concentrationcheck.concentrationcheck(receivedpos_x, receivedpos_y, imgSize_x, imgSize_y, imgRoi, dst)

    return Foundcorner, Foundgap, Location_x, Location_y