import numpy as np
import cv2

cap = cv2.VideoCapture('/home/pi/Downloads/video3.h264')

class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y
# Point = namedtuple('Point', 'y, x')   # point in cartesian coordinates

def line(p1, p2):
    A = (p1[1] - p2[1])
    B = (p2[0] - p1[0])
    C = (p1[0]*p2[1] - p2[0]*p1[1])
    return A, B, -C

def intersection(L1, L2):
    D  = L1[0] * L2[1] - L1[1] * L2[0]
    Dx = L1[2] * L2[1] - L1[1] * L2[2]
    Dy = L1[0] * L2[2] - L1[2] * L2[0]
    if D != 0:
        x = Dx / D
        y = Dy / D
        return x,y
    else:
        return False

# create instance of namedtuples and initialize
imgSize = Point(0, 0)
roiSize = Point(0, 0)
roiGap = Point(0, 0)

while(cap.isOpened()):
    ret, frame = cap.read()

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # region of interest (roi) directly in front of vehicle
    imgShape = frame.shape
    imgSize.x = imgShape[1]  # save the size of the image in pixels
    imgSize.y = imgShape[0]
    roiSize.x = 80  # define size of roi window in percentage of total image size
    roiSize.y = 40
    roiGap.x = int(0.5 * (imgSize.x - roiSize.x * imgSize.x / 100))  # gap calculated as even spaces from image borders
    roiGap.y = int(imgSize.y - roiSize.y * imgSize.y / 100)  # gap calculated as from the top of the image

    roi = gray[roiGap.y:imgSize.y, roiGap.x:imgSize.x - roiGap.x]  # get roi from image
    imgRoi = np.zeros((imgSize.y, imgSize.x), np.uint8)  # create new binary black image
    imgRoi[roiGap.y:imgSize.y, roiGap.x:imgSize.x - roiGap.x] = roi  # add roi to black image

    imgBlurred = cv2.GaussianBlur(roi, (5, 5), 0)
    # blur image using a gaussian blur
    # imgBlurred = cv2.GaussianBlur(gray, (15, 15), 0)

    hist, bins = np.histogram(imgBlurred.ravel(), 256, [0, 256])

    grayval = np.argmax(hist)

    thresh = grayval - 100
    th, dst = cv2.threshold(imgBlurred, thresh, 255, cv2.THRESH_BINARY)

    # detect edges in image using canny
    imgCanny = cv2.Canny(dst, 50, 150, apertureSize=3)

    # set heuristic parameters for the (probabilistic) houghlines function
    rho = 1  # accuracy [pixel]
    acc = 1  # accuracy [deg]
    theta = acc * np.pi / 360
    threshold = 50  # minimum points on a line

    # find lines in image using the (probabilistic) houghlines function
    # outputP:  x1, y1, x2, y2
    # output:   rho, theta
    # lines = cv2.HoughLines(imgCanny, rho, theta, threshold)
    #
    # # check if houghlines function found any lines, if not print error
    # if lines is not None:
    #     exitflag = 1  # houghlines function found at least one line
    #
    #     xvalues = []
    #     intersects = []
    #
    #     # Get values rho and theta from houghlines
    #     rho = [line[0][0] for line in lines]
    #     theta = [line[0][1] for line in lines]
    #
    #     # Create histogram with n bins (possible values) from 0 to pi (range of theta)
    #     bins = 60
    #     thhist, thbins = np.histogram(theta, bins, [0, np.pi])
    #
    #     # Find angles that occur more than the treshold and save that angle
    #     treshold = 3
    #     histangles = []  # Array for storing all angles from the histogram
    #     for i in range(0, len(thhist)):
    #         if np.amax(thhist) > treshold:  # Found feasible maximum
    #             anglepos = np.argmax(thhist)  # Get position of maximum
    #             angle = anglepos * np.pi / bins  # Calculate value of maximum
    #             thhist[anglepos] = 0  # Set histogram value to 0, so we won't find this maximum next iteration
    #             if anglepos - 1 >= 0:  # Check if the position is in bounds of the histogram
    #                 thhist[anglepos - 1] = 0  # Set the neighbour values to zero as well
    #             if anglepos + 1 <= len(thhist)-1:
    #                 thhist[anglepos + 1] = 0
    #             histangles.append(angle)  # Append theta value of the peak to angle
    #
    #     all_angles = [[] for _ in range(len(histangles))]  # Make an array with n arrays, for all found angles from hist
    #     tres = np.pi / bins  # Define a certain treshold
    #
    #     # plot red(0, 0, 255) houghlines in image
    #     for x in range(0, len(lines)):
    #         for rho, theta in lines[x]:
    #             a = np.cos(theta)
    #             b = np.sin(theta)
    #             x0 = a * rho
    #             y0 = b * rho
    #             x1 = int(x0 + 1000 * (-b))
    #             y1 = int(y0 + 1000 * (a))
    #             x2 = int(x0 - 1000 * (-b))
    #             y2 = int(y0 - 1000 * (a))
    #             cv2.line(frame, (x1, y1), (x2, y2), (0, 0, 255), 1)
                # Compare theta with the histogram angle, if within certain interval, store in all_angles
        #         for i in range(0, len(histangles)):
        #             if histangles[i] - tres < theta and theta < histangles[i] + tres:
        #                 all_angles[i].append([theta, rho])
        #
        # rhovalues = [[] for _ in range(len(all_angles))]  # [[rho 1, ~rho 1, ...], [rho2, ~rho2, ...], ...]
        #
        # for i in range(0, len(all_angles)):
        #     rho = [x[1] for x in all_angles[i]]
        #     bins = 12  # Number of bins for histogram of rho
        #     # Histogram of rho (grouped per theta), from the minimum to the maximum of the rho set plus an offset of 200px
        #     # This is done so we only see one 'peak' if there is one line instead of a roughly evenly distributed histogram
        #     rhohist, rhobins = np.histogram(rho, bins, [np.amin(rho) - 100, np.amax(rho) + 100])
        #
        #     treshold = 3
        #
        #     for j in range(0, len(rhohist)):
        #         if np.amax(rhohist) > treshold:  # Value must be found more than n times to be a feasible line
        #             rhopos = np.argmax(rhohist)  # Find position of the maximum rho value in the histogram
        #             rho = rhobins[rhopos]  # Find maximum value of rho
        #             rhohist[rhopos] = 0  # Set histogram value to 0, so we won't find this maximum next iteration
        #             if rhopos - 1 >= 0:  # Check if the position is in bounds of the histogram
        #                 rhohist[rhopos - 1] = 0  # Set the neighbour values to zero as well
        #             if rhopos + 1 <= len(rhohist):
        #                 rhohist[rhopos + 1] = 0
        #             rhovalues[i].append(rho)  # Append rho value of the peak to rhovalues
        #
        # line_amount = 0
        # for k in range(0, len(rhovalues)):  # Count how many different peaks of the rho histogram there are
        #     line_amount += len(rhovalues[k])  # This is equal to the amount of lines
        #
        # all_values = [[] for _ in range(line_amount)]
        #
        # tres = 50
        # extra_line = 0
        # line_counter = -1
        #
        # for i in range(0, len(rhovalues)):  # Go through rhovalues e.g. [[218.5, 379.3], [595.5], [-627.0]]
        #     if len(rhovalues[i]) > 1:  # Split array into multiple lines if multiple rho values
        #         for j in range(0, len(rhovalues[i])):  # Go through all rhovalues[i] e.g. [218.5, 379.3]
        #             line_counter += 1  # Counter of current line we are filling
        #             for x in range(0, len(lines)):  # Go through all found houghlines
        #                 for rho, theta in lines[x]:  # Set rho and theta
        #                     if rhovalues[i][j] - tres < rho and rho < rhovalues[i][j] + tres:  # If rho is within tres
        #                         all_values[line_counter].append([theta, rho])  # Append to all_values
        #     else:
        #         line_counter += 1
        #         print("line:", line_counter, "line amount:", line_amount)
        #         all_values[line_counter] = all_angles[i]
        #
        # avg_line_values = [[] for _ in range(line_amount)]
        #
        # for i in range(0, line_amount):
        #     avg_line_values[i] = [(sum([item[0] for item in all_values[i]]) / len(all_values[i])),
        #                           (sum([item[1] for item in all_values[i]]) / len(all_values[i]))]
        #
        # all_inter = []
        #
        # for x in range(0, len(avg_line_values)):
        #     rho = avg_line_values[x][1]
        #     theta = avg_line_values[x][0]
        #     a = np.cos(theta)
        #     b = np.sin(theta)
        #     x0 = a * rho
        #     y0 = b * rho
        #     x1 = int(x0 + 2000 * (-b))
        #     y1 = int(y0 + 2000 * (a))
        #     x2 = int(x0 - 2000 * (-b))
        #     y2 = int(y0 - 2000 * (a))
        #     cv2.line(imgCanny, (x1, y1), (x2, y2), (0, 0, 255), 2)

    cv2.imshow('frame', imgCanny)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()