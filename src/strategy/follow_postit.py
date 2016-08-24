# import OpenCV functions
import cv2

# import Numpy functions
import numpy as np

# import libraries
# from libs import defs

# import computational entities (functions)
# from detection import lanedetection
from detection import detect_square
# from detection import detectshapes
# from detection import detectshapes
# from detection import gapdetection
# from detection import lineclustering


class Follow:
    # coordinator/composer
    def follow(self):

        # initialise

        # get camera images
        cap = cv2.VideoCapture('../roadtests/postit.mp4')
        while cap.isOpened():
            ret, frame = cap.read()

            [x_pos, dist] = detect_square.detect_square(frame)

            print(x_pos, dist)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cap.release()
        cv2.destroyAllWindows()
