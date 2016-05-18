import cv2
import numpy as np
# a function that determines the concentration in roi is defined
def ConcentrationDetection ( receivedpos_x, receivedpos_y, imgSize_x, imgSize_y):

    roirangex = int(0.05 * imgSize_x)  # the window range of roi
    roirangey = int(0.05 * imgSize_y)

    dstinv = np.invert(dst)
    roi = dstinv[(receivedpos_y - roirangey):(receivedpos_y + roirangey),(receivedpos_x - roirangex):(receivedpos_x + roirangex)]  # get roi from image
    imgRoi = np.zeros((imgSize_y, imgSize_x), np.uint8)  # create new binary black image
    imgRoi[(receivedpos_y - roirangey):(receivedpos_y + roirangey),(receivedpos_x - roirangex):(receivedpos_x + roirangex)] = roi  # add roi to black image

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
    Location_x = []  # those corner points are real ones
    Location_y = []

    # this if else loop shows the threshold for finding a corner or a gap
    if concentration_roi > 0.05 * (roiSize_x * roiSize_y):  # the concentration of white should be at least 5% of the roi windowsize to decide wether or not this is a valid corner
        Foundcorner = 1
        Location_x.append(receivedpos_x)  # after the check this x coordinate is a valide coordinate for a corner
        Location_y.append(receivedpos_y)
    else:
        Foundgap = 1
        Location_x.append(receivedpos_x)
        Location_y.append(receivedpos_x)
    print("Foundgap Foundcorner\n", Foundgap, Foundcorner)
    print("Concentration \n", concentration_roi)
    print("Locationx Locationy\n", Location_x, Location_y)
    cv2.imshow('ROI', imgRoi)
    cv2.imshow('TRESHOLD', dstinv)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    return Foundcorner, Foundgap, Location_x, Location_y


img = cv2.imread('../roadtests/cornerrightinsane.jpg')
intersectimg = img

# blur image using a gaussian blur
imgBlurred = cv2.GaussianBlur(img, (15, 15), 0)
gray_image = cv2.cvtColor(imgBlurred, cv2.COLOR_BGR2GRAY)

hist, bins = np.histogram(img.ravel(), 256, [0, 256])

grayval = np.argmax(hist)

thresh = grayval - 60
th, dst = cv2.threshold(gray_image, thresh, 255, cv2.THRESH_BINARY)

# detect edges in image using canny
imgCanny = cv2.Canny(dst, 50, 150, apertureSize=3)

imgShape = img.shape  # show the resolution of the image
imgSize_x = imgShape[1]  # save the size of the image in pixels, this is used to create a black image
imgSize_y = imgShape[0]
receivedpos_x = [1200, 600, 200, 100]
receivedpos_y = [230, 230, 100, 80 ]
Location_xo = []
Location_yo = []
# this for loop takes every valid corner position and save it in Location
for i in range(0,len(receivedpos_x)):
    [Foundcorner,Foundgap,Location_x,Location_y]=ConcentrationDetection(receivedpos_x[i],receivedpos_y[i],imgSize_x,imgSize_y)
    Location_xo.append(Location_x)
    Location_yo.append(Location_y)

print("Reallocx \n Reallocy \n",Location_xo,Location_yo)
# print("Reallocx2 Reallocy2 \n",len(Location_xo),len(Location_yo))

#This for loop places roi between the intersection
for i in range (0,len(Location_xo)):
     if i + 1 == len(Location_xo):
         receivedpos_x = int(sum(Location_xo[-1] + Location_xo[0]) / 2)
         receivedpos_y = int(sum(Location_yo[-1] + Location_yo[0]) / 2)
         ConcentrationDetection(receivedpos_x,receivedpos_y,imgSize_x, imgSize_y)
         # print("receivedpos_x, receivedpos_x-1,receivedpos_x0 \n", receivedpos_x,Location_xo[-1],Location_xo[0])
     elif i+1 < len(Location_xo):
         receivedpos_x = int(sum(Location_xo[i] + Location_xo[i + 1]) / 2)
         receivedpos_y = int(sum(Location_yo[i] + Location_yo[i + 1]) / 2)
         print("receivedpos_x, receivedpos_x-1,receivedpos_x0 \n", receivedpos_x, Location_xo[i], Location_xo[i+1])
         ConcentrationDetection(receivedpos_x, receivedpos_y, imgSize_x, imgSize_y)