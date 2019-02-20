import sys
import rospy
import numpy as np
from group7_final.srv import *
from group7_final.msg import BoardState
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import math

bridge = CvBridge() 
 
image = cv2.imread('/home/cs4752/ros_ws/src/group7_final/src/frame0001.jpg')


hand = 'right'

def img2cimg(image,lower,upper):
    hsv_img = cv2.GaussianBlur(cv2.cvtColor(image,cv2.COLOR_BGR2HSV),(5, 5), 0)
    mask   = cv2.inRange(hsv_img, lower, upper)
    output = cv2.bitwise_and(hsv_img, hsv_img, mask = mask)
    img = cv2.medianBlur(output,5)
    return cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

def find_ball(cimg):
    kernel = np.ones((5,5),np.uint8)
    kernel[0,0] = 0
    kernel[0,4] = 0
    kernel[4,0] = 0
    kernel[4,4] = 0
    dilated = cv2.dilate(cimg,kernel,iterations=1)
    closing = cv2.morphologyEx(dilated, cv2.MORPH_CLOSE, kernel)
    circles = cv2.HoughCircles(closing,cv2.cv.CV_HOUGH_GRADIENT,1,20,param1=25,param2=15,minRadius=5,maxRadius=11)
    if not circles==None:
        #circles = np.uint8(np.around(circles))
        for i in circles[0,:]:
            circle_center_x = i[0]
            circle_center_y = i[1]
            circle_radius   = i[2]
            #expected_diameter = rospy.get_param('/true_ball_diameter')/rospy.get_param('pixel_to_m')
            #ball_x = (circle_center_x-rospy.get_param('/board_x'))*rospy.get_param('pixel_to_m')
            #ball_y = (circle_center_x-rospy.get_param('/board_y'))*rospy.get_param('pixel_to_m')
            #ball_on_ground = not circle_radius*2 > expected_diameter
        #print(str(len(circles[0,:]))+" circles found")
        cv2.imshow("circles",circles)
        cv2.waitKey(0)
        return circle_center_x,circle_center_y
    else:
        #print "no golf ball in sight"
        return 0,0
        
#THIS IS WHERE THE PIXEL COORDINATES ARE CALCULATED
#while images[1]==[] #or images[0]==[]: # wait for images to load | ONLY NEED KINECT IMAGE
#    True
    
lower = np.array([140, 100, 0]) 
upper = np.array([180, 195, 255])

# Top right if right arm || Top left if the left arm
input("Put ball in the origin of the board.")
origin = image
cv2.imshow("origin",origin)
cv2.waitKey(0)
#time.sleep(3)


# pixel_coords = []
board_pix_limits = np.zeros( (3,2) )
pixel_per_meter = 0

#CALL find_ball FOR THE origin, yaxis, xaxis IMAGES?
(origin_pix_x,origin_pix_y)=find_ball(img2cimg(origin,lower,upper))

print ("origin point")
print (origin_pix_x,origin_pix_y)

def find_ball(cimg):
    kernel = np.ones((5,5),np.uint8)
    kernel[0,0] = 0
    kernel[0,4] = 0
    kernel[4,0] = 0
    kernel[4,4] = 0
    dilated = cv2.dilate(cimg,kernel,iterations=1)
    closing = cv2.morphologyEx(dilated, cv2.MORPH_CLOSE, kernel)
    circles = cv2.HoughCircles(closing,cv2.cv.CV_HOUGH_GRADIENT,1,20,param1=25,param2=15,minRadius=5,maxRadius=11)
    if not circles==None:
        #circles = np.uint8(np.around(circles))
        for i in circles[0,:]:
            circle_center_x = i[0]
            circle_center_y = i[1]
            circle_radius   = i[2]
            #expected_diameter = rospy.get_param('/true_ball_diameter')/rospy.get_param('pixel_to_m')
            #ball_x = (circle_center_x-rospy.get_param('/board_x'))*rospy.get_param('pixel_to_m')
            #ball_y = (circle_center_x-rospy.get_param('/board_y'))*rospy.get_param('pixel_to_m')
            #ball_on_ground = not circle_radius*2 > expected_diameter
        #print(str(len(circles[0,:]))+" circles found")
        cv2.imshow("circles",circles)
        cv2.waitKey(0)
        return circle_center_x,circle_center_y
    else:
        #print "no golf ball in sight"
        return 0,0
    
print("Place gripper at top block to get ready for phase 2.")
