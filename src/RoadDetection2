import cv2
import numpy as np

im = cv2.imread('../roadtests/road0.jpg')
#imgGrayscale = cv2.cvtColor(im,cv2.COLOR_BGR2GRAY)
#imgGrayscale=cv2.inRange(im,cv2.cv.Scalar(75,75,75),cv2.cv.Scalar(121,121,121),)
# define range of grey color in RGB
lower_gray = np.array([240,240,240])
upper_gray = np.array([250,250,253])
imgGrayscale = cv2.inRange(im, lower_gray, upper_gray)
imgBlurred = cv2.GaussianBlur(imgGrayscale, (5, 5), 0)              # blur
imgCanny = cv2.Canny(imgBlurred, 100, 200)

cv2.imshow('grayscaled',imgGrayscale)
cv2.imshow('imgblurred',imgBlurred)
cv2.imshow('imgcanny',imgCanny)

minLineLength =100 #Line segments shorter than this are rejected
maxLineGap = 10 #Maximum allowed gap between line segments to treat them as single line
rho=1 #accuracy 1 pixel
theta=np.pi /360 # accuracy 1 degree
threshold=20
lines = cv2.HoughLinesP(imgCanny, rho, theta, threshold, minLineLength, maxLineGap)
a=np.array([[[1,2,3,4]],[[5,6,7,8]],[[9,10,11,12]]])
aa=a.shape
print("array a \n",a, "\n size a \n",aa)
anew=np.delete(a,[10])
print("new a\n",anew)
#print("array lines",lines)

for x in range(0,len(lines)):
    #y=lines.shape #383 matrices with each one 1 row, 4 column
    #print("test1",y)

    for x1,y1,x2,y2 in lines[x]:
        #for x3, y3, x4, y4 in lines[x]:
        for index, (x3, y3, x4, y4) in enumerate(lines[x]):
            if y1 == y2 and y3 == y4:  # Horizontal Lines
                diffy = abs(y1 - y3)
            elif x1 == x2 and x3 == x4:  # Vertical Lines
                diffx = abs(x1 - x3)
            #else:
               #diff = 0

            #if diff < 20 and diff is not 0:
            if diffy < 1 or diffx < 20 :
                np.delete(lines[x],index)
                #print("test")

    #gridsize = (len(lines) - 2) / 2
    #print("gridsize",gridsize)
    cv2.line(im,(x1,y1),(x2,y2),(0,0,255),2)

cv2.imshow('houghlines',im)
cv2.waitKey(0)


cv2.destroyAllWindows()
