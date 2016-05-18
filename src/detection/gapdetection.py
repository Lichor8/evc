# import OpenCV functions
import cv2

# import Numpy functions
import numpy as np

# import libraries
# from libs import defs

# import computational entities (functions)
from detection import concentrationcheck
from detection import backgroundsubtraction as backsubt


def gapdetection(img, gappos):

    dst = backsubt.backgroundsubstration(img)

    gappos_x = gappos[0]
    gappos_y = gappos[1]

    imgShape = img.shape  # show the resolution of the image
    imgSize_x = imgShape[1]  # save the size of the image in pixels, this is used to create a black image
    imgSize_y = imgShape[0]
    imgRoi = np.zeros((imgSize_y, imgSize_x), np.uint8)
    Location_xo = []
    Location_yo = []
    imgRoi = np.zeros((imgSize_y, imgSize_x), np.uint8)
    gappos = []
    # print("empty gappos \n",gappos)

    # this for loop takes every valid corner position and save it in Location
    for i in range(0, len(gappos_x)):
        [Foundgap, cornerposfuckrik_x,cornerposfuckrik_y, gapposfuckrik_x, gapposfuckrik_y] = \
            concentrationcheck.concentrationcheck(gappos_x[i], gappos_y[i], imgSize_x, imgSize_y, imgRoi, dst)

        Location_xo.append(cornerposfuckrik_x)
        Location_yo.append(cornerposfuckrik_y)

    # debug prints
    # print("Reallocx Reallocy \n", Location_xo, Location_yo)
    # print("Reallocx2 Reallocy2 \n",len(Location_xo),len(Location_yo))

    # This for loop places roi between the intersection (gaps)
    for i in range(0, len(Location_xo)):
        if i + 1 == len(Location_xo):
            gappos_x = int(sum(Location_xo[-1] + Location_xo[0]) / 2)
            gappos_y = int(sum(Location_yo[-1] + Location_yo[0]) / 2)

            [Foundgap, cornerposfuckrik_x,cornerposfuckrik_y, gapposfuckrik_x, gapposfuckrik_y] = \
                concentrationcheck.concentrationcheck(gappos_x, gappos_y, imgSize_x, imgSize_y, imgRoi, dst)
            if Foundgap == 1:
                gappos.append([gapposfuckrik_x, gapposfuckrik_y])

            # print("gappos_x, gappos_x-1,gappos_x0 \n", gappos_x,Location_xo[-1],Location_xo[0])
        elif i + 1 < len(Location_xo):
            gappos_x = int(sum(Location_xo[i] + Location_xo[i + 1]) / 2)
            gappos_y = int(sum(Location_yo[i] + Location_yo[i + 1]) / 2)
            # print("gappos_x, gappos_x-1,gappos_x0 \n", gappos_x, Location_xo[i], Location_xo[i + 1])

            [Foundgap, cornerposfuckrik_x,cornerposfuckrik_y,gapposfuckrik_x, gapposfuckrik_y] = \
                concentrationcheck.concentrationcheck(gappos_x, gappos_y, imgSize_x, imgSize_y, imgRoi, dst)
            if Foundgap == 1:
                gappos.append([gapposfuckrik_x, gapposfuckrik_y])


    return gappos
