import cv2
import numpy as np

import cv2
import numpy as np

im = cv2.imread('../roadtests/road0.jpg')
imgGrayscale2 = cv2.cvtColor(im,cv2.COLOR_BGR2GRAY)
#imgGrayscale=cv2.inRange(im,cv2.cv.Scalar(75,75,75),cv2.cv.Scalar(121,121,121),)
# define range of grey color in RGB
lower_gray = np.array([240,240,240])
upper_gray = np.array([250,250,253])
imgGrayscale = cv2.inRange(im, lower_gray, upper_gray)
imgBlurred = cv2.GaussianBlur(imgGrayscale2, (5, 5), 0)              # blur
imgCanny = cv2.Canny(imgBlurred, 100, 200)

cv2.imshow('grayscaled',imgGrayscale)
cv2.imshow('imgblurred',imgBlurred)
cv2.imshow('imgcanny',imgCanny)
cv2.imshow('imggrayscale2',imgGrayscale2)

minLineLength =10 #Line segments shorter than this are rejected
maxLineGap = 10 #Maximum allowed gap between line segments to treat them as single line
rho=1 #accuracy 1 pixel
theta=np.pi /360 # accuracy 1 degree
threshold=20
lines = cv2.HoughLinesP(imgCanny, rho, theta, threshold, minLineLength, maxLineGap)
###################################################################
# test out how np.delete and for index loop work
#a=np.array([[[1,2,3,4]],[[5,6,7,8]],[[9,10,11,12]]])
#aa=a.shape
#print("array a \n",a, "\n size a \n",aa)
#anew=np.delete(a,[10])
#print("new a\n",anew)
#print("array lines",lines)
#loopme = 'THIS IS A VERY LONG STRING WITH MANY MANY WORDS!'
#for index, w in enumerate(loopme):
#    print ("CURRENT WORD IS", w, "AT CHARACTER", index)
##################################################################
for x in range(0,len(lines)-1):
# create matrix index x which consistes all the element in lines

    for x, (x1,y1,x2,y2) in enumerate(lines[x]):
        #for loop to create a line
        for y, (x3, y3, x4, y4) in enumerate (lines[x+1]):
        #for loop to create a second line so we can use the difference in between to filter out double lines
        #for x, (x3, y3, x4, y4) in enumerate(lines[x+1]):
            if y1 == y2 and y3 == y4:  # Horizontal Lines
                diffy = abs(y1 - y3)
                #calculating the difference between horizontal lines
                print("\n index_x index_y y1 y3 y2\n",x,y,y1,y3,y2,"\n diff in horizontal lines \n",diffy)
            # elif x1 == x2 and x3 == x4:  # Vertical Lines
            #     diffx = abs(x1 - x3)
            #     print("\n x1 x3 x2\n", x1, x3,x2, "\n diff in vertical lines \n", diffx)
            #else:
               #diff = 0
            #if diff < 20 and diff is not 0:
            if diffy < 10: #or diffx < 10 :
                 np.delete(lines[x],x)
                 print("index check again",y)
    #gridsize = (len(lines) - 2) / 2
    #print("gridsize",gridsize)
    cv2.line(im,(x1,y1),(x2,y2),(0,0,255),2)

cv2.imshow('houghlines',im)
cv2.waitKey(0)
cv2.destroyAllWindows()




#
# a=np.array([[[1,2,3,4]],[[5,6,7,8]],[[9,10,11,12]],[[1,2,3,2]],[[1,2.2,3,2.2]]])
# aa=a.shape
# print("array a \n",a, "\n size a \n",aa)
# anew=np.delete(a,[10])
# print("new a\n",anew)
# print("element in array \n",np.array([1]))
#if np.array(1==

# for index, (x3, y3, x4, y4) in enumerate(lines[x]):
#     if y1 == y2 and y3 == y4:  # Horizontal Lines
#         diffy = abs(y1 - y3)
#     elif x1 == x2 and x3 == x4:  # Vertical Lines
#         diffx = abs(x1 - x3)
#         # else:
#         # diff = 0
#
#     # if diff < 20 and diff is not 0:
#     if diffy < 1 or diffx < 20:
#         np.delete(lines[x], index)
#         # print("test")

        # gridsize = (len(lines) - 2) / 2