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


# define classes
class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y
# Point = namedtuple('Point', 'y, x')   # point in cartesian coordinates

# create instance of namedtuples and initialize
imgSize = Point(0, 0)

cap = cv2.VideoCapture('/home/pi/Downloads/video3.h264')

while cap.isOpened():
    ret, frame = cap.read()
    img = frame

    imgShape = img.shape
    imgSize.x = imgShape[1]     # save the size of the image in pixels
    imgSize.y = imgShape[0]

    x1 = 580
    y1 = 600
    x2 = 990
    y2 = 610
    x3 = 0
    y3 = imgSize.y-100
    x4 = imgSize.x
    y4 = imgSize.y-60

    cv2.circle(img, (x1, y1), 3, (0, 255, 0), 2)
    cv2.circle(img, (x2, y2), 3, (0, 255, 0), 2)
    cv2.circle(img, (x3, y3), 3, (0, 255, 0), 2)
    cv2.circle(img, (x4, y4), 3, (0, 255, 0), 2)

    # rows, cols, ch = img.shape
    W = 1000
    H = 700

    pts1 = np.float32([[x1,y1],[x2,y2],[x3,y3],[x4,y4]])            # TL, TR, BL, BR
    pts2 = np.float32([[int(0.4*W), int(0.5*H)], [int(0.6*W), int(0.5*H)], [int(0.4*W), H], [int(0.6*W), H]])

    M = cv2.getPerspectiveTransform(pts1, pts2)

    dst = cv2.warpPerspective(img, M, (W, H))

    cv2.imshow('Input', img)
    cv2.imshow('Output', dst)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

