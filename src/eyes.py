#!/usr/bin/env python

import sys
import rospy
import numpy as np
from group7_final.srv import *
from group7_final.msg import BoardState
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import math
import time

import baxter_interface as bi

bridge = CvBridge()
images = [[],[]] # 0 = hand, 1 = kinect

def callback_hand(msg):
    try:
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
        images[0] = cv2_img
    except CvBridgeError, e:
        print(e)    


def callback_kinect(msg):
    try:
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
        images[1] = cv2_img
    except CvBridgeError, e:
        print(e)
    

########################################DEFINE MAJOR POINTS##############################################
lower_pink = np.array([140, 100, 150]) 
upper_pink = np.array([184, 255, 255])
lower_bl = np.array([103, 97, 150]) 
upper_bl = np.array([120, 255, 255])


def define_major_points():
    #MEASURE CRITICAL DIMENSIONS OF THE BOARD
    board_real_y=rospy.get_param('true_field_width') #meters
    board_real_x=rospy.get_param('true_field_length') #meters
    
    hand = rospy.get_param('hand_mode')
    limb=bi.Limb(hand)
    rospy.sleep(0.1)

    limb_joints=limb.joint_angles()

    #print("Place arm at our own goal center. ")
    #time.sleep(8)
    print("Place arm at our own goal center. ")
    rospy.sleep(3)
    
    pose=limb.endpoint_pose()
    my_gpos=pose['position']
    rospy.set_param('my_goal_center',[my_gpos.x, my_gpos.y, my_gpos.z])
    rospy.set_param('board_z',my_gpos.z)
    if hand == "left":
        offset = rospy.get_param('true_field_length')
        origin = [my_gpos.x,        my_gpos.y-rospy.get_param('true_field_width')/2,    my_gpos.z]
        pos_x = 1
    else:
        offset = -rospy.get_param('true_field_length')
        origin = [my_gpos.x+offset, my_gpos.y-rospy.get_param('true_field_width')/2,    my_gpos.z]
        pos_x = -1
    rospy.set_param('their_goal_center',[my_gpos.x, my_gpos.y+offset, my_gpos.z])
    rospy.set_param('/plane_origin',origin)

    #print("Place arm above top left corner of the board:")
    #time.sleep(8)
    print("Place arm above origin corner of the board:")
    rospy.sleep(3)
    pose=limb.endpoint_pose()
    opos=pose['position']

    #print("Place the arm at the tip of your x unit vector.")
    #time.sleep(8)
    print("Place the ball at the tip of your x unit vector.")
    rospy.sleep(3)
    pose=limb.endpoint_pose()
    xpos=pose['position']

    #print("Place the arm at the tip of your y unit vector.")
    #time.sleep(8)
    print("Place the ball at the tip of your y unit vector.")
    rospy.sleep(3)
    pose=limb.endpoint_pose()
    ypos=pose['position']

    #NEED TO "RAW INPUT" IMAGE TO GET BOARD CORNER AND BLOCK CALIBRATION
    # raw_input doesn't work with the game server
    
    #NEED TO PLACE ARM IN IDEAL THROWING POSTURE

    mag_x=math.sqrt(math.pow(xpos.x-my_gpos.x,2)+math.pow(xpos.y-my_gpos.y,2)+math.pow(xpos.z-my_gpos.z,2))
    mag_y=math.sqrt(math.pow(ypos.x-my_gpos.x,2)+math.pow(ypos.y-my_gpos.y,2)+math.pow(ypos.z-my_gpos.z,2))
    norm_x=[(xpos.x-my_gpos.x)/mag_x, (xpos.y-my_gpos.y)/mag_x, (xpos.z-my_gpos.z)/mag_x]
    norm_y=[(ypos.x-my_gpos.x)/mag_y, (ypos.y-my_gpos.y)/mag_y, (ypos.z-my_gpos.z)/mag_y]
    rospy.set_param('/plane_x_vector',norm_y)
    rospy.set_param('/plane_y_vector',norm_x)
    
    #THIS IS WHERE THE PIXEL COORDINATES ARE CALCULATED
    #while images[1].size==0 #or images[0]==[]: # wait for images to load | ONLY NEED KINECT IMAGE
	#    True
    lower = [140, 100, 0] 
    upper = [180, 195, 255]
	
	# Top right if right arm || Top left if the left arm
    print("Put ball in the origin of the board.")
    time.sleep(3)
    origin = images[1]
    
    print("Put ball on the negative y axis of the board.")
    time.sleep(3)
    yaxis = images[1]

    
    print("Put ball on the x axis of the board.")
    time.sleep(3)
    xaxis = images[1]
    
    # pixel_coords = []
    ball_images = [origin, yaxis, xaxis]
    board_pix_limits = np.zeros( (3,2) )
    pixel_per_meter = 0
    
    #CALL find_ball FOR THE origin, yaxis, xaxis IMAGES?
    (origin_pix_x,origin_pix_y,origin_pix_g)=find_ball(origin)
    (yaxis_pix_x,yaxis_pix_y,yaxis_pix_g)=find_ball(yaxis)
    (xaxis_pix_x,xaxis_pix_y,xaxis_pix_g)=find_ball(xaxis)
    
    origin_pix=np.array([origin_pix_x,origin_pix_y])
    yaxis_pix=np.array([yaxis_pix_x,yaxis_pix_y])
    xaxis_pix=np.array([xaxis_pix_x,xaxis_pix_y])
    
    yvect_pix=origin_pix-y_axis_pix
    if hand == 'left':
        xvect_pix=origin_pix-xaxis_pix
    else:
        xvect_pix=xaxis_pix-origin_pix
        
    pxpm_y=np.linalg.norm(yvect_pix)/board_real_y
    pxpm_x=np.linalg.norm(xvect_pix)/board_real_x
    
    pxpm=(pxpm_x+pxpm_y)/2
    
    rospy.set_param('pix_per_m',pxpm)
        
    print("Place gripper at top block to get ready for phase 2.")
    
def pixel_to_board(positions):
    global images
    global flag
    #returns in place transformation of board positions to a range of 0 to 1 in terms of where it is on the board
    # 1 in the xaxis = the right side
    # 1 in the yaxis = top of the board (farthest from baxter)
    for i in range(len(positions)):
        positions[i][0] = (positions[i][0] - board_pixel_limits[0]) / (board_pixel_limits[1]-board_pixel_limits[0])
        positions[i][1] = (positions[i][1] - board_pixel_limits[2]) / (board_pixel_limits[3]-board_pixel_limits[2])
    return positions
    
    
def define_major_points_test():
    #images[1]=arg1
    #MEASURE CRITICAL DIMENSIONS OF THE BOARD
    board_real_y=rospy.get_param('true_field_width') #meters
    board_real_x=rospy.get_param('true_field_length') #meters
    
    hand = 'right'
    limb=bi.Limb(hand)
    rospy.sleep(0.1)

    limb_joints=limb.joint_angles()
    
    #THIS IS WHERE THE PIXEL COORDINATES ARE CALCULATED
    #while images[1]==[] #or images[0]==[]: # wait for images to load | ONLY NEED KINECT IMAGE
	#    True
	    
    lower = np.array([140, 100, 150]) 
    upper = np.array([184, 255, 255])
    
    #lower = np.array([140, 100, 150]) 
    #upper = np.array([184, 225, 255])
    
    print("Place gripper at home")
    time.sleep(4)
    limb_end=limb.endpoint_pose()
    limb_endxyz=limb_end['position']
    limb_joints=limb.joint_angles()
    rospy.set_param('def_angles',[limb_joints[hand+'_s0'],limb_joints[hand+'_s1'],limb_joints[hand+'_e0'],limb_joints[hand+'_e1'],limb_joints[hand+'_w0'],limb_joints[hand+'_w1'],limb_joints[hand+'_w2']])
    home=[limb_endxyz.x,limb_endxyz.y,limb_endxyz.z]
    rospy.set_param('board_home',home)
    rospy.set_param('board_z',limb_endxyz.z)
    
    print("Place arm at our own goal center.")
    time.sleep(4)
    pose=limb.endpoint_pose()
    my_gpos=pose['position']
    rospy.set_param('my_goal_center',[my_gpos.x, my_gpos.y, my_gpos.z])
    
    print("Place gripper at clearance height")
    time.sleep(4)
    limb_end=limb.endpoint_pose()
    limb_endxyz=limb_end['position']
    rospy.set_param('max_operating_height',limb_endxyz.z)
    
    print("Place gripper at board origin")
    time.sleep(4)
    limb_end=limb.endpoint_pose()
    limb_endxyz=limb_end['position']
    rospy.set_param('board_x',limb_endxyz.x)
    rospy.set_param('board_y',limb_endxyz.y)
	
	# Top right if right arm || Top left if the left arm
    print("Put ball in the origin of the board.")
    time.sleep(4)
    origin = images[1]
    #cv2.imshow("origin",origin)
    #cv2.waitKey(1)

    
    print("Put ball on the negative y axis of the board.")
    time.sleep(4)
    yaxis = images[1]
    #cv2.imshow("yaxis",yaxis)
    #cv2.waitKey(1)

    
    print("Put ball on the x axis of the board.")
    time.sleep(4)
    xaxis = images[1]
    #cv2.imshow("xaxis",xaxis)
    #cv2.waitKey(1)

    # pixel_coords = []
    ball_images = [origin, yaxis, xaxis]
    board_pix_limits = np.zeros( (3,2) )
    pixel_per_meter = 0
    
    #CALL find_ball FOR THE origin, yaxis, xaxis IMAGES?
    (origin_pix_y,origin_pix_x)=findball_bb(img2cimg(origin,lower,upper),origin)
    (yaxis_pix_y,yaxis_pix_x)=findball_bb(img2cimg(yaxis,lower_pink,upper_pink),yaxis)
    (xaxis_pix_y,xaxis_pix_x)=findball_bb(img2cimg(xaxis,lower_pink,upper_pink),xaxis)
    
    #(origin_pix_x,origin_pix_y)=find_ball(img2cimg(origin,lower,upper))
    #(yaxis_pix_x,yaxis_pix_y)=find_ball(img2cimg(yaxis,lower,upper))
    #(xaxis_pix_x,xaxis_pix_y)=find_ball(img2cimg(xaxis,lower,upper))
    
    print ("origin point")
    print (origin_pix_x,origin_pix_y)
    
    print ("y point")
    print (yaxis_pix_x,yaxis_pix_y)
    
    print ("x point")
    print (xaxis_pix_x,xaxis_pix_y)
    
    
    origin_pix=np.array([origin_pix_x,origin_pix_y])
    yaxis_pix=np.array([yaxis_pix_x,yaxis_pix_y])
    xaxis_pix=np.array([xaxis_pix_x,xaxis_pix_y])
    
    yvect_pix=origin_pix-yaxis_pix
    
    if hand == 'left':
        xvect_pix=origin_pix-xaxis_pix
    else:
        xvect_pix=xaxis_pix-origin_pix
        
    pxpm_y=np.linalg.norm(yvect_pix)/board_real_y
    pxpm_x=np.linalg.norm(xvect_pix)/(board_real_x/2.)
    
    print ("pxpm_y=")
    print (pxpm_y)
    print ("pxpm_x=")
    print (pxpm_x)
    
    pxpm=(pxpm_x+pxpm_y)/2.
    
    print ("pxpm=")
    print (pxpm)
    
    #pxpm.item()
    a = xvect_pix/np.linalg.norm(xvect_pix).item()
    a = a.tolist()
    b = yvect_pix/np.linalg.norm(yvect_pix).item()
    b = b.tolist()
    rospy.set_param('pix_per_m',pxpm.item())
    rospy.set_param('x_unit',a)
    rospy.set_param('y_unit',b)
        
    print("Place gripper at top block to get ready for phase 2.")
    
    

################################### Image Analysis Here ##########################################

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
        return circle_center_x,circle_center_y
    else:
        #print "no golf ball in sight"
        return 0,0

def do_blah():
    #global images
    #global flag
    origin = images[1]
    #print(ori)
    #ori=k_im
    #cv2.imshow("origin",origin)
    #cv2.waitKey(1)
    findball_bb(img2cimg(origin,lower_pink,upper_pink),origin)
    #find_blocks(img2cimg(origin,lower_bl,upper_bl),origin)

def findball_bb(ngray,img):
    kernel = np.ones((3,3),np.uint8)
    opening = cv2.morphologyEx(ngray, cv2.MORPH_OPEN, kernel)
    #cv2.imwrite('opening.jpg', opening)
    gray = np.float32(opening)
    kernel = np.ones((8,8),np.uint8)
    closing = cv2.morphologyEx(ngray, cv2.MORPH_CLOSE, kernel)
    
    #cv2.imwrite('/home/cs4752/ros_ws/src/group7_final/src/asdf.jpg',img)
    
    params = cv2.SimpleBlobDetector_Params()
    params.filterByArea = True
    params.minArea = 70
    #300 for singles, 500 for 2 blocks, 1000 three blocks
    params.maxArea = 250
    params.minDistBetweenBlobs = 0.0000000000000000000000000000001
    params.filterByCircularity = True
    params.minCircularity = 0.7
    params.filterByConvexity = True
    params.minConvexity = 0.05
    
    #closing  = cv2.dilate(closing,kernel,iterations=2)
    #I dont use closing because it = []
    #ball_images = [origin, yaxis, xaxis] #gets a better image
    inverted = cv2.bitwise_not(closing)
    ret,binary = cv2.threshold(inverted,150,255,cv2.THRESH_BINARY)
    detector = cv2.SimpleBlobDetector(params)
    #cv2.imwrite('opening.jpg', opening)
    keypoints = detector.detect(binary)
    #print(img.shape)
    shape = np.array(img.shape)
    #print()
    #
    # Either call pixelfield() here, or accept ros param (pixelboard)
    #
    #Get positions relative to the plane origin
    block_center_pos = np.zeros( (len(keypoints),3))
    print(block_center_pos)
    focal_length = shape[0]/(2 * math.tan(57/2))
    #zw = distnace from kinect to the playing board
    zw = 1.5494
    
    i = 0
    for key in keypoints:
        #print(key.pt)
        block_center_pos[i,0] = (key.pt[0])# * zw) / focal_length
        block_center_pos[i,1] = (key.pt[1])# * zw) / focal_length
        print (i)
        print((block_center_pos[i,0],block_center_pos[i,1]))
        i = i + 1
    im = cv2.drawKeypoints(img, keypoints, np.array([]), (0,255,0), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    #cv2.imshow('blobdetect',im)
    ##v2.waitKey(0)
    #cv2.imshow('blobdetect_bin',binary)
    #cv2.waitKey(0)
    #cv2.imshow('blobdetect_cl',closing)
    #cv2.waitKey(0)
    print(block_center_pos)
    return (block_center_pos[0,0],block_center_pos[0,1])
        
def find_blocks(cimg, img):
    kernel = np.ones((7,7),np.uint8)
    opening = cv2.morphologyEx(cimg, cv2.MORPH_OPEN, kernel)
    #res = cv2.bitwise_and(img,img, mask=opening)
    
    # HARRIS FUNCTION
    gray = np.float32(opening)
    dst = cv2.cornerHarris(gray,2,3,0.04)
    #dst = cv2.dilate(dst,None)
    #img[dst>0.01*dst.max()]=[0,0,255]
   
    params = cv2.SimpleBlobDetector_Params()
    params.filterByArea = True
    params.minArea = 50
    #300 for singles, 500 for 2 blocks, 1000 three blocks
    params.maxArea = 1000
    params.minDistBetweenBlobs = 0.00000000000000000000000000000000000000000000000000000000000001
    params.filterByCircularity = True
    params.minCircularity = 0.1
    params.filterByConvexity = True
    params.minConvexity = 0.05

    inverted = cv2.bitwise_not(opening)
    ret,binary = cv2.threshold(inverted,150,255,cv2.THRESH_BINARY)
    detector = cv2.SimpleBlobDetector(params)
    keypoints = detector.detect(binary)
    shape = np.array(binary.shape)

    #Get positions relative to the plane origin
    blocks_x = np.zeros( (len(keypoints),1) )
    blocks_y = np.zeros( (len(keypoints),1) )
    focal_length = shape[0]/(2 * math.tan(57/2))
    #zw = distnace from kinect to the playing board
    zw = 1.5494
    i = 0
    #for key in keypoints:
     #   blocks_x[i] = (key.pt[0] * zw) / focal_length
      #  blocks_y[i] = (key.pt[1] * zw) / focal_length
       # i = i + 1
    im = cv2.drawKeypoints(img, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    
    #cv2.imshow('im',im)
    #cv2.waitKey(1)
    
    #cv2.imshow('opening',opening)
    #cv2.waitKey(0)
    
    blocks_theta = []
    return blocks_x, blocks_y, blocks_theta
    
def img2cimg(image,lower,upper):
    hsv_img = cv2.GaussianBlur(cv2.cvtColor(image,cv2.COLOR_BGR2HSV),(5, 5), 0)
    mask   = cv2.inRange(hsv_img, lower, upper)
    output = cv2.bitwise_and(hsv_img, hsv_img, mask = mask)
    ngray = cv2.cvtColor(output,cv2.COLOR_HSV2BGR)
   #img = cv2.medianBlur(output,5)
    return cv2.cvtColor(ngray,cv2.COLOR_BGR2GRAY)


def getBoardState():
    ball_x = None
    ball_y = None
    ball_on_ground = None
    blocks_x = None
    blocks_y = None
    blocks_theta = None
    board_origin = None
    board_x_unit = None
    board_y_unit = None
    on_blocks = None
    goal_center = None
    goal_theta = None
    defense = None
    while images[0]==[] or images[1]==[]: # wait for images to load
	    True

    color_boundaries = [([140, 100,   0] , [180, 195, 255]), # pink
	                    ([ 57, 132,  61] , [ 65, 201, 114]), # green
	                    ([103,  97, 100] , [120, 255, 250]), # blue
	                    ([103,  86,  65] , [145, 133, 128])] # yellow
	                    
    image = images[1] #only look at kinect image
    hsv_img = cv2.GaussianBlur(cv2.cvtColor(image,cv2.COLOR_BGR2HSV),(5, 5), 0)

    for i in range(0,len(color_boundaries)):
        lower = np.array(color_boundaries[i][0], dtype = "uint8")
        upper = np.array(color_boundaries[i][1], dtype = "uint8")
         
        # find the pixels within the specified color boundaries and apply a mask
        mask   = cv2.inRange(hsv_img, lower, upper)
        output = cv2.bitwise_and(hsv_img, hsv_img, mask = mask)

        img = cv2.medianBlur(output,5)
        cimg = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

        if np.array_equal(lower,color_boundaries[0][0]) and np.array_equal(upper,color_boundaries[0][1]):
            #print("pink")
            ball_x, ball_y = findball_bb(cimg,img)
            ball_on_ground = True

        if np.array_equal(lower,color_boundaries[1][0]) and np.array_equal(upper,color_boundaries[1][1]):
            #print("green")
            # for green, determine playing field regions - save corners as bounds? or line segments? also get board unit vectors
            board_origin = []
            board_x_unit = []
            board_y_unit = []
        if np.array_equal(lower,color_boundaries[2][0]) and np.array_equal(upper,color_boundaries[2][1]):
            #print("blue")
            blocks_x, blocks_y, blocks_theta = find_blocks(cimg,img)
            
        if np.array_equal(lower,color_boundaries[3][0]) and np.array_equal(upper,color_boundaries[3][1]):
            #print("yellow")
            # for yellow, determine goal region - save corners as bounds? or line segments maybe
            goal_center = [1,1]
            goal_theta = 0

    # update this if have time
    on_blocks = [0,0,0,0,0,0,0,0,0,0] # blocks you are allowed to pick up - set using block pos and board info    
    #for i,block in blocks_x:
    #    if block < 333333: #our side of board
    #    on_blocks(i) = 1

    if abs(ball_x-rospy.get_param('board_home')[0]) < .001 and abs(ball_y-rospy.get_param('board_home')[1]) < 001:
        defense = False
    else:
        defense = True
    
    return (ball_x,ball_y,ball_on_ground,blocks_x,blocks_y,blocks_theta,board_origin,board_x_unit,board_y_unit,on_blocks,goal_center,goal_theta,defense)

def update_state():
    global images
    global flag
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
		state = getBoardState()
		board = BoardState(state[0],state[1],state[2],state[3],state[4],state[5],state[6],state[7],state[8],state[9],state[10],state[11],state[12])
		pub_board.publish(board)
		rate.sleep()

if __name__ == "__main__":
    while rospy.get_param('hand_mode') == "": # wait for game to give us a hand
        True
    rospy.init_node('eyes',anonymous=True)
    print("eyes seeing")
    pub_board = rospy.Publisher('board_state',BoardState,queue_size=10)
    hand_topic_name   = 'cameras/'+rospy.get_param('hand_mode')+'_hand_camera/image'
    kinect_topic_name = 'camera/rgb/image_color'
    rospy.Subscriber(hand_topic_name, Image, callback_hand)
    rospy.Subscriber(kinect_topic_name, Image, callback_kinect)
    #lower_pink = np.array([140, 100, 150]) 
    #upper_pink = np.array([184, 255, 255])
    rospy.sleep(10)
    define_major_points_test()
    update_state()
    rospy.spin()


