#!/usr/bin/env python

import rospy
import numpy as np
import argparse
import struct
import sys
import math
import time

from scipy import spatial
from std_msgs.msg import String

import baxter_interface as bi

from baxter_pykdl import baxter_kinematics as bk

from baxter_kdl.kdl_parser import kdl_tree_from_urdf_model
from urdf_parser_py.urdf import URDF

from group7_final.srv import *
from group7_final.msg import Order,BoardState

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
			  'on_defense':  None };

##############################################################################################################

def stop_moving(hand):

	limb=bi.Limb(hand)
	rospy.sleep(0.1)
	j_vels=limb.joint_velocities()
	j_vels[hand+'_s0']=0
	j_vels[hand+'_s1']=0
	j_vels[hand+'_e0']=0
	j_vels[hand+'_e1']=0
	j_vels[hand+'_w0']=0
	j_vels[hand+'_w1']=0
	j_vels[hand+'_w2']=0
	limb.set_joint_velocities(j_vels)


def get_currents():

	hand = rospy.get_param('/hand_mode')
	limb=bi.Limb(hand)
	rospy.sleep(.1)
	end=limb.endpoint_pose()
	pos = end['position']
	ori = end['orientation']
	return np.array([[pos[0],pos[1],pos[2]],[ori[0],ori[1],ori[2],ori[3]]])

def ik_test(limb,pos,ori):
    print("ik")
    ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
    iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
    ikreq = SolvePositionIKRequest()
    hdr = Header(stamp=rospy.Time.now(), frame_id='base')

    poses = {'left': PoseStamped(header=hdr,pose=Pose(
                                            position=Point(
                                                x=pos[0], y=pos[1], z=pos[2],
                                            ),
                                            orientation=Quaternion(
                                                x=ori[0], y=ori[1], z=ori[2], w=ori[3],
                                            ), ),),
             'right': PoseStamped(header=hdr,pose=Pose(
                                            position=Point(
                                                x=pos[0], y=pos[1], z=pos[2],
                                            ),
                                            orientation=Quaternion(
                                                x=ori[0], y=ori[1], z=ori[2], w=ori[3],
                                            ),),)}
    ikreq.pose_stamp.append(poses[limb])
    try:
        rospy.wait_for_service(ns, 5.0)
        resp = iksvc(ikreq)
    except (rospy.ServiceException, rospy.ROSException), e:
        rospy.logerr("Service call failed: %s" % (e,))
        return 1

    # Check if result valid, and type of seed ultimately used to get solution
    # convert rospy's string representation of uint8[]'s to int's
    resp_seeds = struct.unpack('<%dB' % len(resp.result_type),
                               resp.result_type)
    if (resp_seeds[0] != resp.RESULT_INVALID):
        seed_str = {ikreq.SEED_USER: 'User Provided Seed',ikreq.SEED_CURRENT: 'Current Joint Angles',ikreq.SEED_NS_MAP: 'Nullspace Setpoints',}.get(resp_seeds[0], 'None')
        print("SUCCESS - Valid Joint Solution Found from Seed Type: %s" %
              (seed_str,))
        # Format solution into Limb API-compatible dictionary
        limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
    else:
        print("INVALID POSE - No Valid Joint Solution Found.") 
    joint_solution = dict(zip(resp.joints[0].name, resp.joints[0].position))

    print("IK SOLUTION ANGLES:")
    print(joint_solution)
    return joint_solution
    
def ChangeGripper(action):
    if rospy.get_param('/gripper')==action:
        return True
    else:
        rospy.set_param('/gripper',action)
        gripper = bi.Gripper(rospy.get_param('/hand_mode'))
        rospy.sleep(1)
        if action=="open":
            gripper.open()
            return True
        elif action=="close":
            gripper.close()
            return True
        else:
            print("Please enter a valid gripper action.")
            return False

def MoveTo(target):
    hand=rospy.get_param('/hand_mode')
    current_limb=bi.Limb(hand)
    rospy.sleep(0.1)
    current_baxter_position = current_limb.endpoint_pose()
    epsilon = 0.001

    pos = current_baxter_position['position']
    ori = current_baxter_position['orientation']
    
    if np.linalg.norm(np.array([pos.x,pos.y,pos.z])-np.array(target)) < epsilon:
        return True
    else:
        
        lift_position = [pos.x,pos.y,rospy.get_param('max_operating_height')]
        print(lift_position)
        hover_table_position = [target[0],target[1],rospy.get_param('max_operating_height')]
        i=0

        j_pos_lift=ik_test(hand,lift_position,ori)
        print("jello")
        print(j_pos_lift)
        print("ji")
        
        current_limb.move_to_joint_positions(j_pos_lift,20)
        rospy.sleep(2)

        j_pos_hov=ik_test(hand,hover_table_position,ori)

        current_limb.move_to_joint_positions(j_pos_hov,20)
        rospy.sleep(2)

        j_pos_target=ik_test(hand,target,ori)
        current_limb.move_to_joint_positions(j_pos_target,20)
        rospy.sleep(1.25)
        return True
    
###################################### Used for stacking in phase 2 #########################################

def handle_move_body(req):
    if req.Action=="Open":
        isPossible = ChangeGripper('open')
    elif req.Action=="Close":
        isPossible = ChangeGripper('close')
    elif req.Action=="MoveTo":
        isPossible = MoveTo(req.Target)
    else:
        print("Please enter a valid action: Open, Close, or MoveTo.")
    return isPossible

####################################### setup actions (phase 1) ##############################################

def define_major_points():

    #MEASURE CRITICAL DIMENSIONS OF THE BOARD
    
    hand = rospy.get_param('/hand_mode')
    limb=bi.Limb(hand)
    rospy.sleep(0.1)

    limb_joints=limb.joint_angles()

    print("Place arm at our own goal center.")
    time.sleep(8)
    
    pose=limb.endpoint_pose()
    my_gpos=pose['position']
    rospy.set_param('my_goal_center',[my_gpos.x, my_gpos.y, my_gpos.z])
    rospy.set_param('board_z',my_gpos.z)
    if hand == "left":
        offset = rospy.get_param('true_field_length')
        origin = [my_gpos.x,        my_gpos.y-rospy.get_param('true_field_width')/2,    my_gpos.z]
    else:
        offset = -rospy.get_param('true_field_length')
        origin = [my_gpos.x+offset, my_gpos.y-rospy.get_param('true_field_width')/2,    my_gpos.z]
    rospy.set_param('their_goal_center',[my_gpos.x, my_gpos.y+offset, my_gpos.z])
    rospy.set_param('/plane_origin',origin)

    print("Place arm above top left corner of the board:")
    time.sleep(8)
    pose=limb.endpoint_pose()
    opos=pose['position']

    print("Place the arm at the tip of your x unit vector.")
    time.sleep(8)
    pose=limb.endpoint_pose()
    xpos=pose['position']

    print("Place the arm at the tip of your y unit vector.")
    time.sleep(8)
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
    print("Place gripper at top block to get ready for phase 2.")

########################################## defensive actions #########################################################

# maybe randomly vary the speed to make our movements less predictable?
def DrawLine(dest,v): #dest is a vector relative to current position
    hand = rospy.get_param('hand_mode')
    limb=bi.Limb(hand)
    rospy.sleep(0.1)
    j_vels=limb.joint_velocities()
    start_pose=limb.endpoint_pose()
    start_pos=start_pose['position']

    start_xyz=np.array([start_pos.x, start_pos.y, start_pos.z])
    travel_vector = np.array(dest-start_xyz)
    abs_dist=np.linalg.norm(travel_vector)
    dir_vect =travel_vector/(np.linalg.norm(travel_vector))
    v_0=v
    goal=np.array(start_xyz+travel_vector)

    v=v_0*dir_vect
    w = np.array([0, 0, 0])
    xi=np.concatenate((v, w),axis=0)	

    joint_values = limb.joint_names()
    kin=bk(hand)

    q_dot=np.dot(kin.jacobian_pseudo_inverse(),xi)

    j_vels[hand+'_s0']=q_dot[0,0]
    j_vels[hand+'_s1']=q_dot[0,1]
    j_vels[hand+'_e0']=q_dot[0,2]
    j_vels[hand+'_e1']=q_dot[0,3]
    j_vels[hand+'_w0']=q_dot[0,4]
    j_vels[hand+'_w1']=q_dot[0,5]
    j_vels[hand+'_w2']=q_dot[0,6]

    frq=.05
    limb.set_command_timeout(20)
    limb.set_joint_velocities(j_vels)
    limb.set_joint_velocities(j_vels)
    vel_dict=limb.endpoint_velocity()['linear']
    rospy.sleep(frq)
    time_at_start = rospy.get_time() # in seconds
    isMoving = True
    
    while isMoving:
        print("moving!")
        t = rospy.get_time() - time_at_start
        pos_dict=limb.endpoint_pose()['position']
        or_dict=limb.endpoint_pose()['orientation']
        current_position = np.array([pos_dict.x,pos_dict.y,pos_dict.z,or_dict.x,or_dict.y,or_dict.z])
        expected_position = np.array([start_pos.x, start_pos.y, start_pos.z,0,0,0]) + np.array([xi[0]*t, xi[1]*t, xi[2]*t,0,0,0])
        joint_angles=limb.joint_angles()
        position_error = current_position - expected_position
        vel_dict=limb.endpoint_velocity()['linear']
        vel_ang=limb.endpoint_velocity()['angular']
        current_v = np.array([vel_dict.x,vel_dict.y,vel_dict.z,0,0,0])
        k = .5
        dir_vect = dir_vect-k*np.array([position_error[0],position_error[1],position_error[2]])
        dir_vect = dir_vect/(np.linalg.norm(dir_vect))
        new_vel = v_0*np.array([dir_vect[0],dir_vect[1],dir_vect[2],0,0,0])
        I=np.identity(7)
        b_vect=np.array([0,0,0,0,0,0,0])
        q_dot = np.dot(kin.jacobian_pseudo_inverse(),(new_vel))
        j_vels[hand+'_s0']=q_dot[0,0]
        j_vels[hand+'_s1']=q_dot[0,1]
        j_vels[hand+'_e0']=q_dot[0,2]
        j_vels[hand+'_e1']=q_dot[0,3]
        j_vels[hand+'_w0']=q_dot[0,4]
        j_vels[hand+'_w1']=q_dot[0,5]
        j_vels[hand+'_w2']=q_dot[0,6]
        rospy.sleep(frq)
        if (board_params['on_defense']==False) or (AreWeThereYet(limb,start_xyz,abs_dist)): 
            j_vels=stop_moving(hand)
            isMoving = False
        limb.set_joint_velocities(j_vels)

def AreWeThereYet(limb,start,dist):
	cur_pos=limb.endpoint_pose()['position']
	cur_pos_arr=numpy.array([cur_pos.x, cur_pos.y, cur_pos.z])
	cur_dist_vect=cur_pos_arr-start
	cur_dist=numpy.linalg.norm(cur_dist_vect)
	
	if(cur_dist<dist):
		return False

	return True   

def defend_goal(up_end,down_end):

    #MoveTo(down_end)
    up_from_down =  up_end - down_end
    down_from_up = -up_from_down
    while (board_params['on_defense']==True) and (rospy.get_param('t_on_offense') < 1):
        DrawLine(up_from_down,0.1)
        DrawLine(down_from_up,0.1)

######################################## OFFENCE ##########################################
def neutral_pos(hand):

    limb=bi.Limb(hand)
    rospy.sleep(0.1)
    limb.move_to_neutral()

def set_joint_angles(hand,angles):
    limb=bi.Limb(hand)
    rospy.sleep(0.1)
    temp=limb.joint_angles()

    current_angles=np.array([temp[hand+'_s0'],temp[hand+'_s1'],temp[hand+'_e0'],temp[hand+'_e1'],temp[hand+'_w0'],temp[hand+'_w1'],temp[hand+'_w2']])
    
    desired_angles=np.array([angles[hand+'_s0'],angles[hand+'_s1'],angles[hand+'_e0'],angles[hand+'_e1'],angles[hand+'_w0'],angles[hand+'_w1'],angles[hand+'_w2']])
    
    n=10

    delta_angles=desired_angles-current_angles
    step_angles=delta_angles/n
    
    print(current_angles)
    
    for i in range(n):
        current_angles=current_angles+step_angles

        temp[hand+'_s0']=current_angles[0]
        temp[hand+'_s1']=current_angles[1]
        temp[hand+'_e0']=current_angles[2]
        temp[hand+'_e1']=current_angles[3]
        temp[hand+'_w0']=current_angles[4]
        temp[hand+'_w1']=current_angles[5]
        temp[hand+'_w2']=current_angles[6]

        limb.set_joint_positions(temp)
        rospy.sleep(.25)


def throw_test():
    hand=rospy.get_param('hand_mode')
    limb=bi.Limb(hand)
    board_params['on_defense']= False
    bi.Gripper(hand).calibrate()
    rospy.sleep(1)

    #hard-coded right arm throwing position | should be a param

    if hand=="right":
        throw_angs={'right_s0': 0.8410049659973146,'right_s1': -0.9518350777954102, 'right_w0': 0.9096506061767579, 'right_w1': -1, 'right_w2': -3.046485841259766, 'right_e0': -1.0289176122985841, 'right_e1': 2.235010005395508} #'right_w1': 0.7198204838928223
        wind_angle=-1.5
        rel_angle=3
    else:
        throw_angs={'left_w0': 1.8246701451049805, 'left_w1': -1, 'left_w2': -0.011504855895996095, 'left_e0': 1.581917685699463, 'left_e1': 2.211233303210449, 'left_s0': -1.4166312559936525, 'left_s1': -0.30794664281616213} #'left_w1': -1.397073000970459
        wind_angle=-1.5
        rel_angle=3
    
    limb.move_to_neutral()
    set_joint_angles(hand,throw_angs)
    rospy.sleep(3)
    j_vel=limb.joint_velocities()

    limb.set_command_timeout(20)

    j_vel[hand+'_w1']=2000
    limb.set_joint_velocities(j_vel)
    limb.set_joint_position_speed(1.0)
    print("IT'S A SINKER")

    throw_angs[hand+'_w1']=rel_angle
    limb.set_joint_positions(throw_angs)
    print("IT'S A STRIKE")

    #rospy.sleep(.05) #tune timer for optimal release point
    
    ChangeGripper('open')
    
    print("ball thrown")
    
def angled_throw(direction):
    True

######################################## subscriber callbacks ##########################################

def callback_board(data):
    board_params['ball_x']         = data.ball_x
    board_params['ball_y']         = data.ball_y
    board_params['ball_on_ground'] = data.ball_on_ground
    board_params['blocks_x']       = data.blocks_x
    board_params['blocks_y']       = data.blocks_y
    board_params['blocks_theta']   = data.blocks_theta
    board_params['board_origin']   = data.board_origin
    board_params['board_x_unit']   = data.board_x_unit
    board_params['board_y_unit']   = data.board_y_unit
    board_params['on_blocks']      = data.on_blocks
    board_params['goal_center']    = data.goal_center
    board_params['goal_theta']     = data.goal_theta
    board_params['on_defense']     = data.defense
    
def initialize_home():
    input("Place gripper at home")
    hand = rospy.get_param('hand_mode')
    limb = bi.Limb(hand)
    limb_end=limb.endpoint_pose()
    limb_endxyz=limb_end['position']
    home=[limb_endxyz.x,limb_endxyz.y,limb_endxyz.z]
    rospy.set_param('board_home',home)

    input("Place gripper at clearance height")
    limb_end=limb.endpoint_pose()
    limb_endxyz=limb_end['position']
    rospy.set_param('max_operating_height',limb_endxyz.z)

def doOrder(data):
    print("Robot hears: " + data.order)
    print("Robot says: Your command is my wish.")

    s = data.order.split()
    center = rospy.get_param('my_goal_center')
    L = rospy.get_param('true_goal_length')
    
    if data.order == "defend goal":
    
        up_end   = np.array(center)+np.array([0,  L/2, .015])
        down_end = np.array(center)+np.array([0, -L/2, .015])
        defend_goal(up_end,down_end)
        
    if data.order == "define major points":
    
        define_major_points()
        
    if data.order == "basic throw":
    
        ChangeGripper("open")
        MoveTo(rospy.get_param('board_home'))
        ChangeGripper("close")
        throw_test()
        
    if s[0] == "Hold":
    
        a1 = s.find("[")
        b1 = s.find("]")
        vect_string_1 = s[a1:b1]
        num_list1 = []
        for i in vect_string_1.split(","):
            num_list1.append(int(vect_string_1.split(",")[i]))
        ChangeGripper("open")
        MoveTo(rospy.get_param('board_home'))
        ChangeGripper("close")
        MoveTo(np.array(num_list1)+np.array([0,0,.02]))
        a2 = s.find("[",a1+1)
        b2 = s.find("]",b1+1)
        vect_string_2 = s[a2:b2]
        num_list2 = []
        for i in vect_string_2.split(","):
            num_list2.append(int(vect_string_2.split(",")[i]))
        angled_throw(num_list2)

#######################################################################################################

if __name__ == "__main__":
    while rospy.get_param('board_z') == None:
        True
    rospy.init_node('body')
    print("body running")
    if False:	
        #throw_test()
        data.order="basic throw"
        doOrder(data)
    rospy.Subscriber("order",Order,doOrder)
    rospy.Subscriber("board_state",BoardState,callback_board)
    s = rospy.Service('move_body', MoveBody, handle_move_body)
    rospy.spin()



