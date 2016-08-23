# import OpenCV functions
import cv2

# import Numpy functions
import numpy as np

# import libraries
# from libs import defs

# import computational entities (functions)
from detection import lanedetection
from detection import detectshapes
# from detection import detectshapes
# from detection import gapdetection
# from detection import lineclustering


class Strategy:
    # coordinator/composer
    def strategy(self):

        # initialise
        sign_matrix = np.zeros(shape=(3, 5))    # matrix for storing traffic sign data [distance]
        sign_array = np.zeros(shape=(1, 5))     # array for robustly detected signs

        dist_tres = 0.5     # distance treshold

        # get camera images
        cap = cv2.VideoCapture('../roadtests/video3short.mp4')
        while cap.isOpened():
            ret, frame = cap.read()

            # Matrix to store which signs have been detected in the last n frames
            # [uturn, stop, left, right, straight]
            sign_matrix = np.roll(sign_matrix, 1, axis=0)           # Shift matrix 1 row down
            sign_matrix[0, :] = detectshapes.detectshapes(frame)    # Put new values on row 1

            for i in range(0, 5):
                sign_array[0][i] = row_avg(sign_matrix[:, i])

            print(sign_array[0])

            do_lanedetection = 1

            for i in range(0, 5):
                if sign_array[0][i] != 0 and sign_array[0][i] < dist_tres:
                    do_lanedetection = 0

            # if np.argmin(sign_array[0]):
            #     sign_index = np.argmin(sign_array[0])
            #     print(sign_index)
            #     if sign_index == 0:
            #         sign = "uturn"
            #         print("uturn")
            #     elif sign_index == 1:
            #         sign = "stop"
            #         print(sign)
            #     elif sign_index == 2:
            #         sign = "left"
            #         print(sign)
            #     elif sign_index == 3:
            #         sign = "right"
            #         print(sign)
            #     elif sign_index == 4:
            #         sign = "straight"
            #         print(sign)

            if do_lanedetection:
                target = lanedetection.lanedetection(frame)
                print(target)

            # print(sign_matrix)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cap.release()
        cv2.destroyAllWindows()


def row_avg(row):
    """compares values of row and returns average if the values are close"""
    rowavg = 0
    if 0.9*row[0] < row[1] < 1.1*row[0] and 0.9*row[1] < row[2] < 1.1*row[1]:
        rowavg = (row[0]+row[1]+row[2])/3
    return rowavg
