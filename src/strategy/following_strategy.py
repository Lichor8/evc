# import OpenCV functions
import cv2

# import Numpy functions
import numpy as np

# import libraries
# from libs import defs

# import computational entities (functions)
# from detection import lanedetection
from detection import detection_following
# from detection import detectshapes
# from detection import detectshapes
# from detection import gapdetection
# from detection import lineclustering


class Follow:
    # coordinator/composer
    def follow(self):

        # initialise

        # get camera images
        cap = cv2.VideoCapture('../roadtests/papercircle.mp4')
        while cap.isOpened():
            ret, frame = cap.read()

            follow_dist = 0.3     # follow distance is 0.3 m
            [x_angle, dist] = detection_following.detection_following(frame)
            rel_dist = dist - follow_dist

            print(x_angle, rel_dist)

            cv2.waitKey(0)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cap.release()
        cv2.destroyAllWindows()
