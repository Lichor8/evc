# import the necessary packages
import math
from detection import shapedetector
import cv2
import imutils
import numpy as np


# define classes
class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y
# Point = namedtuple('Point', 'y, x')   # point in cartesian coordinates

debug = True


def detect_square(frame):
    pi_cam_x_angle = 54
    pi_cam_y_angle = 41
    x_pos = 0
    dist = 0

    size_sign = 0.076

    resized = imutils.resize(frame, width=500)
    imgShape = resized.shape

    W = imgShape[1]  # save the size of the frame in pixels
    H = imgShape[0]

    ratio = frame.shape[0] / float(resized.shape[0])

    hsv = cv2.cvtColor(resized, cv2.COLOR_BGR2HSV)

    # define range of colors in HSV
    lower_red = np.array([0, 120, 120])
    upper_red = np.array([0, 255, 255])
    lower_red2 = np.array([145, 120, 120])
    upper_red2 = np.array([170, 255, 255])

    maskred1 = cv2.inRange(hsv, lower_red, upper_red)
    maskred2 = cv2.inRange(hsv, lower_red2, upper_red2)

    maskred = maskred1 + maskred2

    kernel = np.ones((3, 3), np.uint8)
    maskreddil = cv2.dilate(maskred, kernel, iterations=2)

    # Copy the thresholded frame.
    im_floodfill = maskreddil.copy()
    # Mask used to flood filling.
    # Notice the size needs to be 2 pixels than the frame.
    h, w = maskreddil.shape[:2]
    mask = np.zeros((h + 2, w + 2), np.uint8)
    # Floodfill from point (0, 0)
    cv2.floodFill(im_floodfill, mask, (0, 0), 255)
    # Invert floodfilled frame
    im_floodfill_inv = cv2.bitwise_not(im_floodfill)
    # Combine the two frames to get the foreground.
    maskreddil = maskreddil | im_floodfill_inv

    maskreddil = cv2.erode(maskreddil, kernel, iterations=1)

    red = cv2.bitwise_and(resized, resized, mask=maskreddil)
    red = cv2.cvtColor(red, cv2.COLOR_BGR2GRAY)

    # gray = cv2.cvtColor(channels[i], cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(red, (1, 1), 0)
    thresh = cv2.threshold(blurred, 50, 255, cv2.THRESH_BINARY)[1]

    # cv2.imshow("Tresh", thresh)
    # cv2.waitKey(0)

    # find contours in the thresholded frame and initialize the
    # shape detector
    cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = cnts[0] if imutils.is_cv2() else cnts[1]
    sd = SquareDetector()

    # loop over the contours
    for c in cnts:
        # compute the center of the contour, then detect the name of the
        # shape using only the contour

        M = cv2.moments(c)
        if M["m00"] > 100:
            cX = int((M["m10"] / M["m00"]) * ratio)
            cY = int((M["m01"] / M["m00"]) * ratio)
            # blobarea = cv2.contourArea(c) * ratio
            shape = sd.detect(c)

            if shape == "square":
                x, y, w, h = cv2.boundingRect(c)
                x = int(x * ratio)
                y = int(y * ratio)
                w = int(w * ratio)
                h = int(h * ratio)
                signratio = size_sign / h
                if debug:
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                if y < 0.5 * H:
                    gamma = np.pi / 180 * abs(0.5 * H - y) / H * pi_cam_y_angle
                    distance = signratio * abs(0.5 * H - y) / math.tan(gamma)
                else:
                    gamma = np.pi / 180 * abs(0.5 * H - y - h) / H * pi_cam_y_angle
                    distance = signratio * abs(0.5 * H - y - h) / math.tan(gamma)

                c = c.astype("float")
                c *= ratio
                c = c.astype("int")

                dist = distance
                x_pos = pi_cam_x_angle*(cX - frame.shape[1]/2)/frame.shape[1]

                if debug:
                    cv2.drawContours(frame, [c], -1, (0, 255, 0), 2)
                    cv2.putText(frame, shape, (cX, cY), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
                    disttext = str(round(distance, 3)) + "m"
                    cv2.putText(frame, disttext, (cX, cY + 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)

    if debug:
        cv2.imshow("Red", red)
        # cv2.imshow("Blue", blue)
        # cv2.imshow("Yellow", yellow)
        # cv2.imshow("Treshold", thresh)
        cv2.imshow("Video", frame)

    return [x_pos, dist]


class SquareDetector:
    def __init__(self):
        pass

    def detect(self, c):
        # initialize the shape name and approximate the contour
        area = cv2.contourArea(c)
        areamin = 500
        areamax = 100000
        peri = cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, 0.04 * peri, False)
        shape = "none"

        # print(len(approx))

        # if the shape has 4 vertices, it is either a square or
        # a rectangle
        if 4 <= len(approx) <= 6 and areamin < area < areamax:
            shape = "square"

        # return the name of the shape
        return shape
