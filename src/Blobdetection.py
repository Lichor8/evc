# Standard imports
import cv2
import numpy as np

im = cv2.imread("../roadtests/trafficsigns.jpg")
im = cv2.resize(im, (0,0), fx=0.5, fy=0.5)

# define range of blue color in HSV
lower_blue = np.array([100, 0, 0])
upper_blue = np.array([255, 100, 100])

# Threshold the HSV image to get only blue colors
mask = cv2.inRange(im, lower_blue, upper_blue)

# Bitwise-AND mask and original image
res = cv2.bitwise_and(im, im, mask=mask)

# Setup SimpleBlobDetector parameters.
params = cv2.SimpleBlobDetector_Params()

# Change thresholds
params.minThreshold = 1
params.maxThreshold = 255

# Filter by Area.
params.filterByArea = True
params.minArea = 1500

# Filter by Circularity
params.filterByCircularity = True
params.minCircularity = 0.1

# Filter by Convexity
params.filterByConvexity = True
params.minConvexity = 0.87

# Filter by Inertia
params.filterByInertia = True
params.minInertiaRatio = 0.01

# Create a detector with the parameters
detector = cv2.SimpleBlobDetector_create(params)

# Detect blobs
keypoints = detector.detect(res)

print(keypoints)

# Draw detected blobs as red circles.
# cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures
# the size of the circle corresponds to the size of blob

im_with_keypoints = cv2.drawKeypoints(im, keypoints, np.array([]), (0, 0, 255),
                                      cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

# Show blobs
cv2.imshow("Keypoints", im_with_keypoints)
cv2.imshow('Mask', mask)
cv2.imshow('Masked image', res)

cv2.waitKey(0)
