import math
import cv2
import numpy as np

# The code does the following: It tries to align the robot with the object, and then the distance inbetween can be calculated.
# first the middle line of the robot is calculated, if the object is either left or right side of the middle line, the robot
# will receive the turning commando. when the robot is well aligned after turning, it will drive to the object if it is
# at certain distance away from the robot. If the object is moving towards the robot, it should drive backwards.


img =cv2.imread("4quadrants.jpg")
imgShape = img.shape  # show the resolution of the image
imgSize_x = imgShape[1]  # save the size of the image in pixels
imgSize_y = imgShape[0]
#test: print("image size x, image size y\n",imgSize_x,imgSize_y)

midpos = int (imgSize_x/2)
#test: xpos_obj = int (imgSize_x/2) +10
xpos_obj = [] # x position of the object in PIXEL
ydist_obj = [] # distance in meter receive from vision

limit_objx =10 # the object is accurate enough aligned if its within this limiet
while (abs(xpos_obj - midpos) > limit_obj): # when the object is more than 20 pixels away from the middle line in robots vision
    if xpos_obj- midpos < 0: # object is at leftside of the robot
        print("give left-turn command to motion") # this should be changed to send commando to motion.
    elif xpos_obj - midpos > 0: # object is at rightside of the robot
        print("give right-turn command to motion\n")
    #test:limiet = limiet +30

limit_objy = 0.2 # the object is at 20 cm away from the robot
while (abs(xpos_obj - midpos) < limit_obj & ydist_obj > limit_objy): # when the object is good aligned and is further than 20 cm away from the robot
    print("give drive straight command to motion")
    if ydist_obj[-1] <= limit_objy # if the object is closer than the limit distance, (-1 indicates the last element in the array)
        print("give stop command to motion")
    elif ydist_obj[-1] - ydist_obj[-2] > 0.1 #if the object is moving towards the robot with a certain speed,
        print("give drive back command to motion")
    #test:if limiet <=40:
     #test:limiet = limiet - 10