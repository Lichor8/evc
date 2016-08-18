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


def detectshapes():
    # create instance of namedtuples and initialize
    imgSize = Point(0, 0)

    # pi_cam_x_angle = 54
    pi_cam_y_angle = 41

    detected_signs = np.zeros(shape=(3, 5))

    # load the image and resize it to a smaller factor so that
    # the shapes can be approximated better
    cap = cv2.VideoCapture('../roadtests/video3short.mp4')

    while cap.isOpened():
        ret, frame = cap.read()
        image = frame
        resized = imutils.resize(image, width=500)

        imgShape = resized.shape
        imgSize.x = imgShape[1]  # save the size of the image in pixels
        imgSize.y = imgShape[0]

        # resized = cv2.GaussianBlur(resized, (5, 5), 0)

        size_sign = 0.15     # absolute size of a traffic sign in meters

        ratio = image.shape[0] / float(resized.shape[0])
        th = 100/ratio  # treshold for center of direction sign

        hsv = cv2.cvtColor(resized, cv2.COLOR_BGR2HSV)

        # define range of colors in HSV
        lower_blue = np.array([100, 60, 60])
        upper_blue = np.array([120, 255, 255])

        lower_yellow = np.array([15, 125, 125])
        upper_yellow = np.array([21, 255, 255])

        lower_red = np.array([0, 120, 120])
        upper_red = np.array([5, 255, 255])
        lower_red2 = np.array([165, 120, 120])
        upper_red2 = np.array([180, 255, 255])

        # Threshold the HSV image to get only certain colors
        maskblue = cv2.inRange(hsv, lower_blue, upper_blue)
        maskyellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
        maskred1 = cv2.inRange(hsv, lower_red, upper_red)
        maskred2 = cv2.inRange(hsv, lower_red2, upper_red2)
        maskred = maskred1 + maskred2

        kernel = np.ones((3, 3), np.uint8)
        maskbluedil = cv2.dilate(maskblue, kernel, iterations=2)
        maskreddil = cv2.dilate(maskred, kernel, iterations=2)
        maskyellowdil = cv2.dilate(maskyellow, kernel, iterations=2)

        masks = [maskbluedil, maskreddil, maskyellowdil]

        for i in range(0, len(masks)):
            # Copy the thresholded image.
            im_floodfill = masks[i].copy()
            # Mask used to flood filling.
            # Notice the size needs to be 2 pixels than the image.
            h, w = masks[i].shape[:2]
            mask = np.zeros((h + 2, w + 2), np.uint8)
            # Floodfill from point (0, 0)
            cv2.floodFill(im_floodfill, mask, (0, 0), 255)
            # Invert floodfilled image
            im_floodfill_inv = cv2.bitwise_not(im_floodfill)
            # Combine the two images to get the foreground.
            masks[i] = masks[i] | im_floodfill_inv

        blue_forcircles = cv2.bitwise_and(resized, resized, mask=masks[0])

        masks[0] = cv2.erode(masks[0], kernel, iterations=1)
        masks[1] = cv2.erode(masks[1], kernel, iterations=1)
        masks[2] = cv2.erode(masks[2], kernel, iterations=1)

        # Bitwise-AND mask and original image
        blue = cv2.bitwise_and(resized, resized, mask=masks[0])
        blue = cv2.cvtColor(blue, cv2.COLOR_BGR2GRAY)
        red = cv2.bitwise_and(resized, resized, mask=masks[1])
        red = cv2.cvtColor(red, cv2.COLOR_BGR2GRAY)
        yellow = cv2.bitwise_and(resized, resized, mask=masks[2])
        yellow = cv2.cvtColor(yellow, cv2.COLOR_BGR2GRAY)

        # bluegray = cv2.cvtColor(blue, cv2.COLOR_BGR2GRAY)

        # detect circles in the image
        circles = cv2.HoughCircles(blue, cv2.HOUGH_GRADIENT, 1, 200, param1=50, param2=30,
                                   minRadius=int(15 / ratio), maxRadius=int(1500 / ratio))
        output = resized.copy()
        circlecenters = []

        # ensure at least some circles were found
        if (circles is not None) and (len(circles[0]) < 3):
            # convert the (x, y) coordinates and radius of the circles to integers
            circles = np.round(circles[0, :]).astype("int")

            # loop over the (x, y) coordinates and radius of the circles
            for (x, y, r) in circles:
                # draw the circle in the output image, then draw a rectangle
                # corresponding to the center of the circle
                cv2.circle(output, (x, y), r, (0, 255, 0), 2)
                cv2.rectangle(output, (x - 2, y - 2), (x + 2, y + 2), (0, 128, 255), -1)
                circlecenters.append([x * ratio, y * ratio, r*ratio])
                # show the output image
                # cv2.imshow("canny", bluecanny)
                cv2.imshow("Houghcircles", output)
        else:
            circles = []

        # print(circlecenters)
        channels = [blue, yellow, red]
        roi_dir = []
        # convert the resized image to grayscale, blur it slightly, and threshold it

        detected_signs = np.roll(detected_signs, 1, axis=0)
        detected_signs[1,:]

        for i in range(0, len(channels)):
            # gray = cv2.cvtColor(channels[i], cv2.COLOR_BGR2GRAY)
            blurred = cv2.GaussianBlur(channels[i], (1, 1), 0)
            thresh = cv2.threshold(blurred, 50, 255, cv2.THRESH_BINARY)[1]

            # cv2.imshow("Tresh", thresh)
            # cv2.waitKey(0)

            # find contours in the thresholded image and initialize the
            # shape detector
            cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cnts = cnts[0] if imutils.is_cv2() else cnts[1]
            sd = shapedetector.ShapeDetector()

            # loop over the contours
            for c in cnts:
                # compute the center of the contour, then detect the name of the
                # shape using only the contour

                M = cv2.moments(c)
                if M["m00"] > 100:
                    cX = int((M["m10"] / M["m00"]) * ratio)
                    cY = int((M["m01"] / M["m00"]) * ratio)
                    blobarea = cv2.contourArea(c)*ratio
                    shape = sd.detect(c, i)

                    if shape == "dir-candidate":
                        for k in range(0, len(circlecenters)):
                            # print([0.9*blobarea, circlecenters[k][2]**2*np.pi, 1.6*blobarea])
                            # print([cX, circlecenters[k][0], cY, circlecenters[k][1]])
                            if cX - th < circlecenters[k][0] < cX + th and cY - th < circlecenters[k][1] < cY + th \
                                    and 0.9*blobarea < circlecenters[k][2]**2*np.pi < 1.6*blobarea:

                                xlow = int((min([item[0][0] for item in c])))
                                xhigh = int((max([item[0][0] for item in c])))
                                ylow = int((min([item[0][1] for item in c])))
                                yhigh = int((max([item[0][1] for item in c])))
                                xhalf = xlow + int((xhigh - xlow) / 2)
                                yhalf = ylow + int((yhigh - ylow) / 2)

                                roi = maskblue[ylow:yhigh, xlow:xhigh]

                                roi_tl = maskblue[ylow:yhalf, xlow:xhalf]
                                roi_tr = maskblue[ylow:yhalf, xhalf:xhigh]
                                roi_bl = maskblue[yhalf:yhigh, xlow:xhalf]
                                roi_br = maskblue[yhalf:yhigh, xhalf:xhigh]

                                pxtres = int((cv2.countNonZero(roi) * 0.04))  # 4% is minimum difference between quarters

                                pxtl = cv2.countNonZero(roi_tl)
                                pxtr = cv2.countNonZero(roi_tr)
                                pxbl = cv2.countNonZero(roi_bl)
                                pxbr = cv2.countNonZero(roi_br)

                                if pxtl + pxtres < pxtr and pxbl > pxbr + pxtres:
                                    shape = "dir left"
                                elif pxtl > pxtr + pxtres and pxbl + pxtres < pxbr:
                                    shape = "dir right"
                                else:
                                    shape = "dir straight"

                                roi_dir.append([xlow, xhigh, ylow, yhigh])
                                cv2.imshow("ROI", roi)

                            else:
                                shape = "none"

                    if shape == "uturn" or shape == "stop" or shape == "dir left" or shape == "dir right" \
                            or shape == "dir straight":
                        x, y, w, h = cv2.boundingRect(c)
                        x = int(x * ratio)
                        y = int(y * ratio)
                        w = int(w * ratio)
                        h = int(h * ratio)
                        cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                        signratio = size_sign / h
                        if y < 0.5 * imgSize.y:
                            gamma = np.pi / 180 * abs(0.5 * imgSize.y - y) / imgSize.y * pi_cam_y_angle
                            distance = signratio * abs(0.5 * imgSize.y - y) / math.tan(gamma)
                        else:
                            gamma = np.pi / 180 * abs(0.5 * imgSize.y - y - h) / imgSize.y * pi_cam_y_angle
                            distance = signratio * abs(0.5 * imgSize.y - y - h) / math.tan(gamma)
                        # print(shape, "sign -", "dist:", distance, "y-offset:", abs(0.5 * imgSize.y - y),
                        #       "angle:", gamma)

                    # multiply the contour (x, y)-coordinates by the resize ratio,
                    # then draw the contours and the name of the shape on the image
                    if shape == "uturn" or shape == "stop" or shape == "dir left" or shape == "dir right" \
                            or shape == "dir straight":
                        c = c.astype("float")
                        c *= ratio
                        c = c.astype("int")
                        cv2.drawContours(image, [c], -1, (0, 255, 0), 2)
                        cv2.putText(image, shape, (cX, cY), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
                        disttext = str(round(distance, 3))+"m"
                        cv2.putText(image, disttext, (cX, cY+15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)

                    if shape == "uturn":


        cv2.imshow("Red", red)
        cv2.imshow("Blue", blue)
        cv2.imshow("Yellow", yellow)
        cv2.imshow("Blue gray", bluegray)
        cv2.imshow("Treshold", thresh)
        cv2.imshow("Video", image)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        cv2.waitKey(0)

    cap.release()
    cv2.destroyAllWindows()
