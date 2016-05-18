# import OpenCV functions
import cv2

# import Numpy functions
import numpy as np

# import robot subsystems
import detection.detection

# create robot objects from classes
det = detection.detection.Detection()

# check robot inputs
io_ok = 1
io_read = 1

# main loop (run while robot receives inputs)
while io_ok:

    if io_read:

        # trigger coordinator/composer of detection
        det.detection()
        # line = det.getline()
        # print(line)

        # status = det.monitor()
        # print(status)

        # something = 5
        # det.setsomethinglincl(something)
        # det.configurator(something)

        # stop main loop
        break
