
import sys
import rospy
import numpy as np
from group7_final.srv import *
from group7_final.msg import BoardState
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

bridge = CvBridge() 
 
image = cv2.imread('/home/cs4752/ros_ws/src/group7_final/src/frame0000.jpg')

        
color_boundaries = [([140, 100,   0] , [180, 195, 255]), # pink
                    ([ 57, 132,  61] , [ 65, 201, 114]), # green
                    ([100,  140, 190] , [140, 200, 255]), # blue
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

    if np.array_equal(lower,color_boundaries[2][0]) and np.array_equal(upper,color_boundaries[2][1]): # if blue
        print("blue")
        # for pink, segment images and extract ball center position
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

        ret,thresh1=cv2.threshold(dilated,140,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C)

        cv2.imshow('gray',thresh1)
        cv2.waitKey(0)

        dst = cv2.cornerHarris(dilated,3,3,0.04)
        #dst = cv2.dilate(dst,None)
        img[dst>0.01*dst.max()]=[0,0,255]
        cv2.imshow('dst',img)
        cv2.waitKey(0)

        #find circles in image
        kernel = np.ones((3,3),np.uint8)

        opening = cv2.morphologyEx(cimg, cv2.MORPH_OPEN, kernel)
        #dilated = cv2.dilate(opening,kernel,iterations=3)
        
        closing = cv2.morphologyEx(opening, cv2.MORPH_CLOSE, kernel)

        
        cv2.imshow("images", np.hstack([cimg,opening,closing]))
        cv2.waitKey(0)
        
        params = cv2.SimpleBlobDetector_Params()

        params.minThreshold = 10;
        params.maxThreshold = 200;
        params.filterByArea = True
        params.minArea = 5

        params.filterByConvexity = True
        params.minConvexity = 0.05


        detector = cv2.SimpleBlobDetector(params)
        keypoints = detector.detect(eroded)
        
        print(img.shape)
        shape = np.array(img.shape)
        for key in keypoints:
            print(key.pt)
        im = cv2.drawKeypoints(image, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        
        cv2.imshow('detected squares',np.hstack(([image,output])))
        cv2.waitKey(0)
        
        #(cnts, _) = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        #print(len(cnts))
        # loop over the contours
        #for c in cnts:
        #    # draw the contour and show it
        #    cv2.drawContours(image, [c], -1, (0, 255, 0), 2)
        #    cv2.imshow("Image", image)
        #    cv2.waitKey(0)
        #    cv2.imshow('detected squares',np.hstack(([image,hsv_img,output])))
        #    cv2.waitKey(0)
            
        #print(str(len(circles[0,:]))+" circles found")
        
