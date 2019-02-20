
import sys
import rospy
import numpy as np
from group7_final.srv import *
from group7_final.msg import BoardState
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

bridge = CvBridge() 
 
image = cv2.imread('/home/cs4752/ros_ws/src/group7_final/src/dz85_1.jpg')

img2=cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

cv2.imshow("image1",img2)
cv2.waitKey(0)
        
color_boundaries = [([140, 100,   0] , [180, 195, 255]), # pink
                    ([ 57, 132,  61] , [ 65, 201, 114]), # green
                    ([103,  86,  65] , [255, 255, 250]), # blue
                    ([103,  86,  65] , [145, 133, 128])] # yellow
                    
hsv_img = cv2.GaussianBlur(cv2.cvtColor(image,cv2.COLOR_BGR2HSV),(5, 5), 0)
cv2.imshow("image2",hsv_img)
cv2.waitKey(0)

ret,thresh1=cv2.threshold(img2,80,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C)

print ret

cv2.imshow("gray",thresh1)
cv2.waitKey(0)

(r,c)= img2.shape

i=0
j=0
top_x=0
top_y=0

dst = cv2.cornerHarris(thresh1,3,3,0.04)
#dst = cv2.dilate(dst,None)
image[dst>0.01*dst.max()]=[0,0,255]
cv2.imshow('dst',image)
cv2.waitKey(0)

print (dst)

(top_x,top_y)=np.nonzero(dst)

#compare values to coordinates set during initialization?
print top_x[0] #add 3.0625 inches (pixel point is 3.0625 inchs to the left of actual boundary)
print r 
print top_y[0] #minus 1.5 inches (pixel point is 1.5 inches higher than actual boundary)
print c
    
