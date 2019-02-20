#!/usr/bin/env python

import sys
import rospy
import numpy as np
from group7_final.srv import *
from group7_final.msg import BoardState,Order
from game_server.msg import GameState
from game_server.srv import Init
import time
import math

import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import baxter_interface as bi

from baxter_pykdl import baxter_kinematics as bk

from baxter_kdl.kdl_parser import kdl_tree_from_urdf_model
from urdf_parser_py.urdf import URDF

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import Header

from baxter_core_msgs.srv import(
    SolvePositionIK,
    SolvePositionIKRequest,
)

######################################### Local Variables ####################################################

bridge = CvBridge()
board_params = {'ball_x': None,
              'ball_y': None,
              'ball_on_ground': '',
              'blocks_x': [],
              'blocks_y': [],
              'blocks_theta': [],
              'board_origin': [],
              'board_x_unit': [],
              'board_y_unit': [],
              'on_blocks': [],
              'goal_center': [],
              'goal_theta': None,
			  'on_defense':  False }
			 
####################################### Useful Functions ######################################################

def collision(point):

    block_l = rospy.get_param('true_block_side_length') #would be better to calculate diagonal
    ball_d  = rospy.get_param('true_ball_diameter')
    epsilon = .01
    for i in range(0,len(board_params['blocks_x'])):
        obstacle = np.array([board_params['blocks_x'][i],board_params['blocks_y'][i]])
        if np.linalg.norm(point-obstacle) < block_l + ball_d + epsilon :
            return True
    return False

def path_collision(start,goal):
    
    path = goal - start
    no_path_collisions = True
    path_length = np.linalg.norm(path)
    increment = 0.03*(path)/path_length
    check_point = start
    i=0
    while i in range(0,int(np.floor(path_length/0.01))) and no_path_collisions:
        check_point = check_point + increment
        no_path_collisions = not collision(check_point)
        i = i+1

    return not no_path_collisions

def decide_action():
    L = rospy.get_param('true_goal_length')
    goal_check_points_x = rospy.get_param('my_goal_center')[0]*np.ones((1,3))
    goal_check_points_y = rospy.get_param('my_goal_center')[1]*(np.ones((1,3))+np.array([L/4.,0,-L/4.]))

    x_limit = rospy.get_param('true_field_length')
    y_limit = rospy.get_param('true_field_width')

    admissisble_x_range = [0,x_limit/2.]
    admissisble_y_range = [0,y_limit]

    samples= [[],[],[]]

    #tries = 0
    for sample in samples:
    #while tries < 50:

        #print("tries = " + str(tries))
        #good_sample = False
        # find a start point in the allowed region:
        #while not good_sample:
        #    sample = np.array([np.random.uniform(admissisble_x_range[0],admissisble_x_range[1],1),
		#		             np.random.uniform(admissisble_y_range[0],admissisble_y_range[1],1)])
        #    good_sample =  not collision(sample)
        #sample = np.reshape(sample,(1,2))[0]

        for i in range(0,len(goal_check_points_x)):
            goal = np.array([goal_check_points_x[i],goal_check_points_y[i]])
            path = goal-sample
            if not path_collision(sample,goal):
                path_found = True
                return "Hold the ball above " +str(sample)+ " and throw it in this direction: "+str(goal-sample)

        print("No straight path found. Checking for possible bank shots.")

        for i in range(0,len(goal_check_points_x)):
            # check for wall1 rebound path:
            a = sample[1]
            a_prime = goal[1]
            d = abs(goal[0] - sample[0])
            b = a*d/(a+a_prime)
            bounce_point = np.array([b,0])
            path = [[bounce_point-sample],[goal-bounce_point]]
            if not path_collision(sample,bounce_point):
                if not path_collision(bounce_point,goal):
                    return "Hold the ball above " +str(sample)+ " and throw it in this direction: "+str(bounce_point-sample)

            # check for wall2 rebound path:
            a = y_limit-sample[1]
            a_prime = y_limit-goal[1]
            d = abs(goal[0] - sample[0])
            b = a*d/(a+a_prime)
            bounce_point = np.array([b,y_limit])
            path = [[bounce_point-sample],[goal-bounce_point]]
            if not path_collision(sample,bounce_point):
                if not path_collision(bounce_point,goal):
                    return "Hold the ball above " +str(sample)+ " and throw it in this direction: "+str(bounce_point-sample)

            #tries = tries + 1

    return "basic throw"


def move_body_client(act,tar):

    #possible inputs: act{open,close,moveto,moveover} tar{[ee_position]}... probably need to add orientation here too
    rospy.wait_for_service('move_body')
    try:
        move_body = rospy.ServiceProxy('move_body', MoveBody)
        resp1 = move_body(act,tar)
        if resp1.Move:
            return "Action successfully performed."
        else:
            return "Action not possible. State of system unchanged."
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def arrange_blocks(desired_block_positions):

    print("Place hand around top block.")	
    hand = rospy.get_param('/hand_mode')
    limb=bi.Limb(hand)
    rospy.sleep(0.1)
    limb_joints=limb.joint_angles()
    time.sleep(2)
    pose=limb.endpoint_pose()
    top_block_pos=pose['position']
    n = rospy.get_param('num_blocks')
    block_order = []
    start_block_positions = []

    for i in range(0,n):
        move_body_client('Open',[])
        move_body_client('MoveTo',[top_block_pos.x,top_block_pos.y,top_block_pos.z-0.0445*i])
        move_body_client('Close',[])
        move_body_client('MoveTo',desired_block_positions[i])
        move_body_client('Open',[])

#################################### Where game actions begin ###################################################

#### PHASE 0 ##########
def phase0(data):

    print("Game has not started yet. Time remaining: "+str(data.time_remaining)+" Score: "+str(data.score)+" Penalty: "+str(data.penalty))
    print("You've been given "+str(rospy.get_param('num_blocks'))+" blocks.")

#### PHASE 1 ##########
def phase1(groupname,logo,data):
    try:
	    resp1 = init_robot(groupname,logo)
	    if resp1.arm=="none":
	        message = "You were not given an arm! :("
	    else:
	        message = "You have been given the "+resp1.arm+" arm!"
            rospy.set_param('/hand_mode',resp1.arm)
	    print(message)
    except rospy.ServiceException, e:
	    print "Service call failed: %s"%e

    print("You've been given "+str(rospy.get_param('num_blocks'))+" blocks.")
    while board_params['ball_x'] == None: # wait for board_params to be updated
        True
    print(board_params)

#### PHASE 2 ##########
def phase2(config):

    n = rospy.get_param('num_blocks')
    hand=rospy.get_param('hand_mode')
    limb=bi.Limb(hand)
    rospy.sleep(.1)
    end_xyz=limb.endpoint_pose()['position']
    
    x_unit=rospy.get_param('x_unit') #really baxter's y axis
    y_unit=rospy.get_param('y_unit') #really baxter's x axis
    
    if hand == 'right':
        y_offset=.3 #meters
    else:
        y_offset=-.3 #meters
    x_s = np.array(rospy.get_param('board_home')[0]) + np.array([0,.1,-.1,.2,-.2])

    desired_block_positions = np.zeros((n,3))
    for i in range(0,n):
        mult = np.power(-1,i)
        desired_block_positions[i,:] = [x_s[i],end_xyz.y+y_offset,rospy.get_param('board_z')]
        
    arrange_blocks(desired_block_positions)


#### PHASE 3 ##########
def phase3():

    c = "defend goal"
    print('phase 3')
    if board_params['on_defense']:
        rospy.set_param('t0_on_offense',0)
        rospy.set_param('t_on_offense',0)
    else:
        # If the ball is in the home position for more than one second, switch to offense mode, otherwise stay on defense
        if rospy.get_param('t0_on_offense') == 0:
            rospy.set_param('t0_on_offense',time.time())
        else:
            rospy.set_param('t_on_offense',time.time() - rospy.get_param('t0_on_offense'))
        if rospy.get_param('t_on_offense') > 1:
            c ="basic throw"
    pub.publish(c)
    center = rospy.get_param('my_goal_center')
    L = rospy.get_param('true_goal_length')
    down_end = np.array(center)+np.array([0, -L/2, .015])
    move_body_client('MoveTo',down_end)

######################################## subscriber callback functions ################################################

def callback_game(data):

    if rospy.get_param('phase') == data.current_phase:
        return
    elif rospy.get_param('phase') < data.current_phase:
        rospy.set_param('phase',data.current_phase)
        phase = data.current_phase
        if phase==0:
            print("in phase 0")
            phase0(data)

        if phase==1:
            cv_img = cv2.imread('/home/cs4752/ros_ws/src/group7_final/src/Sylvester.jpg')
            logo = bridge.cv2_to_imgmsg(cv_img, "bgr8")
            phase1('Group 7',logo,data)

        if phase==2:
            print("Arrange Blocks!")
            phase2(data)

        if phase==3:
            print("Game On!")
            phase3()

def callback_board(data):

    board_params['ball_x']       = data.ball_x
    board_params['ball_y']       = data.ball_y
    board_params['ball_on_ground'] = data.ball_on_ground
    board_params['blocks_x']     = data.blocks_x
    board_params['blocks_y']     = data.blocks_y
    board_params['blocks_theta'] = data.blocks_theta
    board_params['board_origin'] = data.board_origin
    board_params['board_x_unit'] = data.board_x_unit
    board_params['board_y_unit'] = data.board_y_unit
    board_params['on_blocks']    = data.on_blocks
    board_params['goal_center']  = data.goal_center
    board_params['goal_theta']   = data.goal_theta
    board_params['on_defense']   = data.defense

#####################################################################################################################

if __name__ == "__main__":

    pub = rospy.Publisher('order',Order,queue_size=10)
    rospy.init_node('brain',anonymous=True)
    rospy.Subscriber("/game_server/game_state",GameState,callback_game)
    rospy.Subscriber("board_state",BoardState,callback_board)
    print('brain running')
    init_robot = rospy.ServiceProxy('/game_server/init', Init)
    rospy.wait_for_service('/game_server/init')
    rospy.spin()


