#!/usr/bin/env python3

# Created on: Jan 30, 2021
# Author: 
# Dhananjay Tiwari, 
# 5th year, Dual Degree
# Mechanical Engineering, 
# Indian Institute of Technology Bombay
# bug_base algorithm implementation: 
# ouputs the final path of the robot, if it exists otherwise failure

from pydoc import cli
from turtle import update
from sc627_helper.msg import MoveXYAction, MoveXYGoal, MoveXYResult
import rospy
import actionlib
import numpy as np
from helper import *
import matplotlib.pyplot as plt

rospy.init_node('test', anonymous= True)

#Initialize client
client = actionlib.SimpleActionClient('move_xy', MoveXYAction)
client.wait_for_server()

# ======================#
#       functions       #
# ======================#

def computePose(vec):
    vec = vec/norm(vec)
    x = vec[0]
    y = vec[1]
    if x > 0 and y >= 0:
        theta = np.arctan(y/x)
    elif x < 0 and y >= 0:
        theta = np.pi - np.arctan(abs(y/x))
    elif x < 0 and y < 0:
        theta = np.pi + np.arctan(abs(y/x))
    elif x > 0 and y < 0:
        theta = 2*np.pi - np.arctan(abs(y/x))
    elif x == 0 and y > 0:
        theta = np.pi/2
    elif x == 0 and y < 0:
        theta = 3*np.pi/2

    return theta # between 0 to 2*pi radians

def moveTurtle(wp, client, position, direction):

    wp.pose_dest.x = position[0]
    wp.pose_dest.y = position[1]
    wp.pose_dest.theta = computePose(direction)
    
    #send waypoint to turtlebot3 via move_xy server
    client.send_goal(wp)
    client.wait_for_result()
    #getting updated robot location
    result = client.get_result()

    return np.array([result.pose_final.x, result.pose_final.y])


# ===================#
#       inputs       #
# ===================#

#Read from input file
data = open('input.txt', 'r')
data = data.read().split('\n\n')

# initialize start, goal positions and step size
robotSetup = data[0].split('\n')
start = np.array([float(robotSetup[0].split(',')[0]), float(robotSetup[0].split(',')[1])])
goal  = np.array([float(robotSetup[1].split(',')[0]), float(robotSetup[1].split(',')[1])])
step_size = float(robotSetup[2])

# read the obstacle data
for k in range(len(data[1:])):
    obstacleData = data[1:][k]
    for i in range(len(obstacleData.split('\n'))):
        points = obstacleData.split('\n')[i].split(',')
        if i == 0:
            P = np.array([[float(points[0]), float(points[1])]])
        else:
            P = np.append(P, np.array([[float(points[0]), float(points[1])]]), axis = 0)
    if k == 0:
        obstacleList = P
    else:
        obstacleList = (obstacleList, P)


# ============================#
#        initialization       #
# ============================#

#setting result as initial location
result = MoveXYResult()
result.pose_final.x = start[0]
result.pose_final.y = start[1]
result.pose_final.theta = 0 #in radians (0 to 2pi)

current_position = np.array([
    result.pose_final.x,
    result.pose_final.y
]) # initialize current position
path = np.array([current_position]) # initialize the path for plotting
numObstacles = len(obstacleList) # number of obstacles in the arena
out_pointToObstacle = np.zeros([numObstacles, 5]) # initialize the output of computeDistancePointToPolygon(P, q)
    
direction = (goal - current_position)/norm(goal - current_position) # initial direction

# ============================#
#         Run the loop        #
# ============================#

wp = MoveXYGoal()

while norm(current_position-goal) > step_size:

    est_position = current_position + direction * step_size
        
    for i in range(numObstacles):
        out_pointToObstacle[i, :] = computeDistancePointToPolygon(obstacleList[i], est_position)

    d_min = min(out_pointToObstacle[:, 0])
    i_min                   = np.argmin(out_pointToObstacle[:, 0])
    hit_position            = out_pointToObstacle[i_min, 3:]

    if d_min < step_size:
        print('Failure! Hit the obstacle at', hit_position, 'and cannot get around it')
        break
    
    current_position = moveTurtle(wp, client, est_position, direction)
    path = np.append(path, np.array([current_position]), axis = 0)   
    
    if norm(current_position - goal) < step_size:
        print('Success! I am at the goal')


output = open('output_base.txt', "w")
for position in path:
    line = str(position[0]) + ',' + str(position[1])
    output.write(line)
    output.write('\n')
output.close()


# =================#
#       Plots      #
# =================#

plt.figure()
# plot obstacles
for P in obstacleList:
    plt.plot(np.append(P[:, 0], P[0,0]), np.append(P[:, 1], P[0, 1]))
# plot the robot path between the start point and the goal
plt.plot(path[:,0], path[:, 1])
plt.grid()
plt.xlabel('x-axis')
plt.ylabel('y-axis')
plt.title('Robot path from bug_base algorithm')
plt.show()