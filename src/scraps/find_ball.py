
import sys
import rospy
import numpy as np
from group7_final.srv import *
from group7_final.msg import BoardState
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

bridge = CvBridge() 
 
image = cv2.imread('/home/cs4752/ros_ws/src/group7_final/src/frame0001.jpg')

        
color_boundaries = [([140, 100,   0] , [180, 195, 255]), # pink
                    ([ 57, 132,  61] , [ 65, 201, 114]), # green
                    ([103,  86,  65] , [255, 255, 250]), # blue
                    ([103,  86,  65] , [145, 133, 128])] # yellow
                    
hsv_img = cv2.GaussianBlur(cv2.cvtColor(image,cv2.COLOR_BGR2HSV),(5, 5), 0)

for (lower, upper) in color_boundaries:
    # create NumPy arrays from the boundaries
    lower = np.array(lower, dtype = "uint8")
    upper = np.array(upper, dtype = "uint8")
     
    # find the colors within the specified boundaries and apply the mask
    mask   = cv2.inRange(hsv_img, lower, upper)
    output = cv2.bitwise_and(hsv_img, hsv_img, mask = mask)

    #cv2.imshow("images", np.hstack([image, hsv_img, output]))
    #cv2.waitKey(0)

    img = cv2.medianBlur(output,5)
    cimg = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

    if np.array_equal(lower,color_boundaries[0][0]) and np.array_equal(upper,color_boundaries[0][1]): # if pink
        print("pink")
        # for pink, segment images and extract ball center position

        #find circles in image
        kernel = np.ones((5,5),np.uint8)
        kernel[0,0] = 0
        kernel[0,4] = 0
        kernel[4,0] = 0
        kernel[4,4] = 0
        dilated = cv2.dilate(cimg,kernel,iterations=1)
        closing = cv2.morphologyEx(dilated, cv2.MORPH_CLOSE, kernel)
        opening = cv2.morphologyEx(closing, cv2.MORPH_OPEN, kernel)
        cv2.imshow("images", np.hstack([cimg, dilated, closing, opening]))
        cv2.waitKey(0)
        circles = cv2.HoughCircles(closing,cv2.cv.CV_HOUGH_GRADIENT,1,20,param1=25,param2=15,minRadius=5,maxRadius=11)

        ret,thresh1=cv2.threshold(opening,127,255,cv2.THRESH_BINARY)
        cv2.imshow("images",thresh1)
        cv2.waitKey(0)

        (r,c)=thresh1.shape

        
        (top_x,top_y)=np.nonzero(thresh1)
        print top_x[0]
        print r
        print top_y[0]
        print c

        #if not len(circles)==None:
            #circles = np.uint8(np.around(circles))
        for i in circles[0,:]:
            cv2.circle(image,(i[0],i[1]),i[2],(0,255,0),2)
            cv2.circle(image,(i[0],i[1]),2,(0,0,255),3)
            circle_center_x = i[0] #column value
            circle_center_y = i[1] #row value
            circle_radius   = i[2]
            #print i[0]
            #print i[1]
            expected_radius = 1 # rospy.get_param('true_ball_radius')*some transform having to do with the current camera position
            ball_x = 1 # circle_center_x*some transform having to do with the current camera position
            ball_y = 1 # circle_center_y*some transform having to do with the current camera position
            ball_on_ground = not circle_radius > expected_radius
        cv2.imshow('detected circles',np.hstack(([image,hsv_img,output])))
        cv2.waitKey(0)
        #else:
        print(str(len(circles[0,:]))+" circles found")
        
