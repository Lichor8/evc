# import OpenCV functions
import cv2

# import Numpy functions
import numpy as np


def backgroundsubstration(img):
    # blur image using a gaussian blur
    imgBlurred = cv2.GaussianBlur(img, (19, 19), 0)
    gray_image = cv2.cvtColor(imgBlurred, cv2.COLOR_BGR2GRAY)
    hist, bins = np.histogram(img.ravel(), 256, [0, 256])
    grayval = np.argmax(hist)
    # print(grayval)
    thresh = grayval - 60
    [thres, dst] = cv2.threshold(gray_image, thresh, 255, cv2.THRESH_BINARY)
    return dst
