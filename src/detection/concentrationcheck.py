# import OpenCV functions
import cv2

# import Numpy functions
import numpy as np


# a function that determines the concentration in roi is defined
def concentrationcheck(receivedpos_x, receivedpos_y, imgSize_x, imgSize_y, imgRoi, dst):

    roirangex = int(0.05 * imgSize_x)  # the window range of roi
    roirangey = int(0.05 * imgSize_y)

    dstinv = np.invert(dst)
    roi = dstinv[(receivedpos_y - roirangey):(receivedpos_y + roirangey), (receivedpos_x - roirangex):(receivedpos_x + roirangex)]  # get roi from image
    # imgRoi = np.zeros((imgSize_y, imgSize_x), np.uint8)  # create new binary black image
    imgRoi[(receivedpos_y - roirangey):(receivedpos_y + roirangey), (receivedpos_x - roirangex):(receivedpos_x + roirangex)] = roi  # add roi to black image

    roiShape = roi.shape  # show the resolution of the roi
    roiSize_x = roiShape[1]  # save the size of the image in pixels, this is used to count the concentration in the roi
    roiSize_y = roiShape[0]
    # print("roishape \n",roiShape,"roix roiy \n", roiSize_x,roiSize_y)

    concentration_roi = 0  # initial concentration,this for-loop counts the concentration in the roi
    for i in range(0, roiSize_y):  # loop over all entries of roi
        for j in range(0, roiSize_x):
            if roi[i][j] == 255:  # if a white pixel is found thus value 255, then the concentration will become 1 higher
                concentration_roi = concentration_roi + 1
    # print("totalvalue imgcanny \n",concentration_roi)
    Foundcorner = 0
    Foundgap = 0
    gapposfuckrik_x = []  # those corner points are real ones
    gapposfuckrik_y = []
    cornerposfuckrik_x = []
    cornerposfuckrik_y = []

    # this if else loop shows the threshold for finding a corner or a gap
    if concentration_roi > 0.05 * (roiSize_x * roiSize_y):  # the concentration of white should be at least 5% of the roi windowsize to decide wether or not this is a valid corner
        Foundcorner = 1
        cornerposfuckrik_x.append(receivedpos_x)  # after the check this x coordinate is a valide coordinate for a corner
        cornerposfuckrik_y.append(receivedpos_y)
    else:
        Foundgap = 1
        gapposfuckrik_x.append(receivedpos_x)
        gapposfuckrik_y.append(receivedpos_y)

    # debug statements
    # print("Foundgap Foundcorner\n", Foundgap, Foundcorner)
    # print("Concentration \n", concentration_roi)
    # print("cornerpos_x cornerpos_y gappos_x gappos_y\n", cornerposfuckrik_x, cornerposfuckrik_y,gapposfuckrik_x,gapposfuckrik_y)
    # cv2.imshow('ROI', imgRoi)
    # cv2.imshow('TRESHOLD', dstinv)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()

    # return Foundcorner, Foundgap, cornerposfuckrik_x,cornerposfuckrik_y,gapposfuckrik_x, gapposfuckrik_y
    return Foundgap, cornerposfuckrik_x, cornerposfuckrik_y, gapposfuckrik_x, gapposfuckrik_y
