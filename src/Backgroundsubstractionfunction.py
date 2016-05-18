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

def Backgroundsubstration (img):
    # blur image using a gaussian blur
    imgBlurred = cv2.GaussianBlur(img, (19, 19), 0)
    gray_image = cv2.cvtColor(imgBlurred, cv2.COLOR_BGR2GRAY)
    hist, bins = np.histogram(img.ravel(), 256, [0, 256])
    grayval = np.argmax(hist)
    print(grayval)
    thresh = grayval - 60
    [thres, dst] = cv2.threshold(gray_image, thresh, 255, cv2.THRESH_BINARY)
    return thres, dst

img = cv2.imread('../roadtests/cornerrightinsane.jpg')
[thres, dst]=Backgroundsubstration(img)
cv2.imshow('ORIGINAL', img)
cv2.imshow('TRESHOLD', dst)
# press any key to terminate process
cv2.waitKey(0)
cv2.destroyAllWindows()
