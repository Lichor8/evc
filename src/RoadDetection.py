import cv2
import numpy as np
import os

###################################################################################################
def main():
    imgOriginal =cv2.imread("../roadtests/road0.jpg")               # open image

    if imgOriginal is None:                             # if image was not read successfully
        print ("error: image not read from file \n\n" )       # print error message to std out
        os.system("pause")                                  # pause so user can see error message
        return                                              # and exit function (which exits program)
    # end if

    imgGrayscale = cv2.cvtColor(imgOriginal, cv2.COLOR_BGR2GRAY)        # convert to grayscale

    imgBlurred = cv2.GaussianBlur(imgGrayscale, (5, 5), 0)              # blur

    imgCanny = cv2.Canny(imgBlurred, 100, 200)                          # get Canny edges
   # imgHough = cv2.HoughLines(imgCanny, 100, 200)
    minLineLength = 100 #Line segments shorter than this are rejected
    maxLineGap = 10 #Maximum allowed gap between line segments to treat them as single line
    rho=1 #accuracy 1 pixel
    theta=np.pi / 180 # accuracy 1 degree
    threshold=10 #minimum votes needed to be considered as a line
    lines = cv2.HoughLinesP(imgCanny, rho, theta, threshold, minLineLength, maxLineGap)
    #it directly returns the two endpoints of lines.

    for x1, y1, x2, y2 in lines[0]:
        cv2.line(imgOriginal, (x1, y1), (x2, y2), (0, 0, 255), 2)
    cv2.imshow('houghlinesptest', imgOriginal)

    print("my lines \n\n",lines)

   # cv2.namedWindow("imgOriginal", cv2.WINDOW_AUTOSIZE)        # create windows, use WINDOW_AUTOSIZE for a fixed window size
   # cv2.namedWindow("imgCanny", cv2.WINDOW_AUTOSIZE)           # or use WINDOW_NORMAL to allow window resizing

    #cv2.imshow("imgOriginal", imgOriginal)         # show windows
    cv2.imshow("imgCanny", imgCanny)

    cv2.waitKey()                               # hold windows open until user presses a key
    cv2.destroyAllWindows()                     # remove windows from memory

    return

###################################################################################################
if __name__ == "__main__":
    main()