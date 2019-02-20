#!/usr/bin/env python

import rospy
import numpy
import argparse
import struct
import sys
import math
import time

board_dims = [7,20]

block_r = .5
ball_r = .5
epsilon = .01
gripper_r = 1

obstacles_x = [15,2,3,4,15,6,7,8,9,10]
obstacles_y = [5,12,2,17,2,18,9,4,2,7]

gripper_pos = numpy.array([3.5,18])

goal_check_points_x = [19,19,19,19,19]
goal_check_points_y = [1.5,2.5,3.5,4.5,5.5]

x_limit = 20
y_limit = 7

admissisble_x_range = [0,10]
admissisble_y_range = [0,y_limit]

def collision(check):

    for i in range(0,len(obstacles_x)):
        obstacle = numpy.array([obstacles_x[i],obstacles_y[i]])
        if numpy.linalg.norm(check-obstacle) < block_r + ball_r + epsilon :
            return True
    return False

def path_collision(start,goal):
    
    path = goal - start
    no_path_collisions = True
    path_length = numpy.linalg.norm(path)
    increment = 0.03*(path)/path_length
    check_point = start
    i=0
    while i in range(0,int(numpy.floor(path_length/0.01))) and no_path_collisions:
        check_point = check_point + increment
        no_path_collisions = not collision(check_point)
        i = i+1

    return not no_path_collisions

####################### will need to adjust paths slightly to account for ball size ###########################

time_at_start = time.time() # in seconds
tries = 0
path_found = False

while tries < 50 and not path_found:

    print("tries = " + str(tries))
    good_sample = False
    # find a start point in the allowed region:
    while not good_sample:
        sample = numpy.array([numpy.random.uniform(admissisble_x_range[0],admissisble_x_range[1],1),
				         numpy.random.uniform(admissisble_y_range[0],admissisble_y_range[1],1)])
        good_sample =  not collision(sample)
    sample = numpy.reshape(sample,(1,2))[0]
    print("sample drawn:")
    print(sample)

    for i in range(0,len(goal_check_points_x)):
        goal = numpy.array([goal_check_points_x[i],goal_check_points_y[i]])
        print("checking goal point " + str(i) + ": " + str(goal))
        
        if not path_collision(sample,goal):
            path_found = True
            print("found an open path! with direction:")
            print(path)
            #return "place the ball at " +str(sample)+ " and hit it in this direction: "+str(goal-sample)
            print "place the ball at " +str(sample)+ " and hit it in this direction: "+str(goal-sample)
            break

    print("No straight path found. Checking for possible bank shots.")

    for i in range(0,len(goal_check_points_x)):
        # check for wall1 rebound path:
        a = sample[1]
        a_prime = goal[1]
        d = abs(goal[0] - sample[0])
        b = a*d/(a+a_prime)
        bounce_point = numpy.array([b,0])
        path = [[bounce_point-sample],[goal-bounce_point]]
        if not path_collision(sample,bounce_point):
            if not path_collision(bounce_point,goal):
                print("found an open path! with direction: "+str(path))
                #return "place the ball at " +str(sample)+ " and hit it in this direction: "+str(bounce_point-sample)
                print "place the ball at " +str(sample)+ " and hit it in this direction: "+str(bounce_point-sample)
                break

        # check for wall2 rebound path:
        a = y_limit-sample[1]
        a_prime = y_limit-goal[1]
        d = abs(goal[0] - sample[0])
        b = a*d/(a+a_prime)
        bounce_point = numpy.array([b,y_limit])
        path = [[bounce_point-sample],[goal-bounce_point]]
        if not path_collision(sample,bounce_point):
            if not path_collision(bounce_point,goal):
                print("found an open path! with direction: "+str(path))
                #return "place the ball at " +str(sample)+ " and hit it in this direction: "+str(bounce_point-sample)
                print "place the ball at " +str(sample)+ " and hit it in this direction: "+str(bounce_point-sample)
                break

        tries = tries + 1

print("No path found. We'll try throwing instead!")

time_at_end = time.time() # in seconds
total_time = time_at_end-time_at_start
print("program took " +str(total_time) + " seconds to run.")






