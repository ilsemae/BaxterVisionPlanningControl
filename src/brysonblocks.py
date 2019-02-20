#THINGS TO DO

#CHECK FOR THE BOARD
#IF POINT IS NOT ON BOARD ITS NOT A BLOCK

import cv2
import numpy as np
import math


#New code of getting blocks

#blob detector for 2,3,4,5 blocks then uses feature detection to 

#This now detects balls if code is broken

def findblocks_bb():
    #lower = np.array([103,  97, 100])
    #upper = np.array([120, 255, 255])
    lower = np.array([120, 100, 100])
    upper = np.array([184, 255, 255])
    img = cv2.imread('/home/cs4752/ros_ws/src/group7_final/src/dz85_1.jpg')
    hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    hsv_img = cv2.GaussianBlur(cv2.cvtColor(img,cv2.COLOR_BGR2HSV),(5, 5), 0)
    mask = cv2.inRange(hsv_img, lower, upper)
    
    print (hsv_img[245][370])
    
    res = cv2.bitwise_and(img,img, mask=mask)
    ngray = cv2.cvtColor(res,cv2.COLOR_HSV2BGR)
    ngray = cv2.cvtColor(res,cv2.COLOR_BGR2GRAY)
    kernel = np.ones((3,3),np.uint8)
    opening = cv2.morphologyEx(ngray, cv2.MORPH_OPEN, kernel)
    cv2.imwrite('opening.jpg', opening)
    gray = np.float32(opening)
    kernel = np.ones((6,6),np.uint8)
    closing = cv2.morphologyEx(ngray, cv2.MORPH_CLOSE, kernel)
    #dst = cv2.cornerHarris(gray,2,3,0.04)
    #print(dst)
    #dst = cv2.dilate(dst,None)
    #img[dst>0.01*dst.max()]=[0,0,255]
    cv2.imwrite('harris.jpg',img)
    
    params = cv2.SimpleBlobDetector_Params()
    params.filterByArea = True
    params.minArea = 130
    #300 for singles, 500 for 2 blocks, 1000 three blocks
    params.maxArea = 180
    params.minDistBetweenBlobs = 0.0000000000000000000000000000001
    params.filterByCircularity = True
    params.minCircularity = 0.7
    params.filterByConvexity = True
    params.minConvexity = 0.05
    
    #closing  = cv2.dilate(closing,kernel,iterations=2)
    #I dont use closing because it gets a better image
    inverted = cv2.bitwise_not(closing)
    ret,binary = cv2.threshold(inverted,150,255,cv2.THRESH_BINARY)
    detector = cv2.SimpleBlobDetector(params)
    cv2.imwrite('opening.jpg', closing)
    keypoints = detector.detect(binary)
    print(img.shape)
    shape = np.array(img.shape)
    print()
    #
    # Either call pixelfield() here, or accept ros param (pixelboard)
    #
    #Get positions relative to the plane origin
    block_center_pos = np.zeros( (len(keypoints),3))
    focal_length = shape[0]/(2 * math.tan(57/2))
    #zw = distnace from kinect to the playing board
    zw = 1.5494
    
    i = 0
    for key in keypoints:
        print(key.pt)
        block_center_pos[i,0] = (key.pt[0] * zw) / focal_length
        block_center_pos[i,1] = (key.pt[1] * zw) / focal_length
        i = i + 1
    im = cv2.drawKeypoints(img, keypoints, np.array([]), (0,0,0), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    cv2.imwrite('blobdetect.jpg',im)
    print()
    
  



findblocks_bb()
