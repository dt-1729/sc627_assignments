#!/usr/bin/env python3

# Created on: Feb 24, 2021
# Author: 
# Dhananjay Tiwari, 
# 5th year, Dual Degree
# Mechanical Engineering, 
# Indian Institute of Technology Bombay
# motion planning using potential functions 
# ouputs the final path of the robot, if it exists otherwise failure

import numpy as np
from helper import *
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d

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

# potential field parameters
d_star = 2
xi = 0.8
eta = 0.8
Q_star = 2

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

'''
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
'''

def potentialField(x, y):
    q = np.array([x,y])
    d_q_goal = np.sqrt((x-goal[0])**2 + (y-goal[1])**2)

    # compute attractive potential
    '''
    if d_q_goal <= d_star:
        U_att = 0.5 * xi * d_q_goal**2
    elif d_q_goal > d_star:
        U_att = d_star * xi * d_q_goal - 0.5 * xi * d_star**2
    '''
    U_att = np.where(d_q_goal <= d_star, 0.5*xi*d_q_goal**2, d_star*xi*d_q_goal-0.5*xi*d_star**2)
    # compute repulsive potential
    U_rep = np.zeros(len(obstacleList))
    
    for i in range(len(obstacleList)):
        out = computeDistancePointToPolygon(obstacleList[i], np.array([x, y]))
        d_min = out[0]
        
        if d_min <= Q_star and d_min > 0:
            U_rep[i] = 0.5 * eta*(1/Q_star - 1/d_min)**2
            if U_rep[i] >= 5:
                U_rep[i] = 5
        else:
            U_rep[i] = 0
        
        #U_rep[i] = np.where(d_min <= Q_star, 0.5*eta*(1/Q_star-1/d_min)**2, 0)
    
    return U_att + np.sum(U_rep, axis = 0)

def computePotentialGradientAtPoint(q):

    # compute attractive gradient
    if norm(q - goal) <= d_star:
        att_grad = xi * (q - goal)
    elif norm(q - goal) > d_star:
        att_grad = d_star * xi * (q - goal) / norm(q - goal)
    
    # compute repulsive gradient
    rep_grad = np.zeros([len(obstacleList),2])

    for i in range(len(obstacleList)):
        out = computeDistancePointToPolygon(obstacleList[i], q)
        d_min = out[0]
        closest_point = out[3:]
        if d_min <= Q_star:
            rep_grad[i, :] = eta*(1/Q_star - 1/d_min)*1/d_min**2 * (q - closest_point)/norm(q - closest_point)
        else:
            rep_grad[i, :] = np.array([0,0])

    gradient = att_grad + np.sum(rep_grad, axis = 0)
    
    return gradient

# ============================#
#        initialization       #
# ============================#
current_position = start    
gradient = computePotentialGradientAtPoint(current_position) # initial gradient
path = np.array([current_position]) # initialize the path for plotting
gradient_data = np.array([norm(gradient)]) # intialize the gradient_data to store gradient variation along the path followed by the bot

# loop control parameters
count = 0
MAX_LIMIT = 5000

# simulation loop
while norm(gradient) > 1e-6:

    print('Distance from goal = ', norm(current_position - goal), end = '\r')

    current_position = current_position - step_size * gradient#/norm(gradient)
    gradient = computePotentialGradientAtPoint(current_position)
    path = np.append(path, np.array([current_position]), axis = 0)
    gradient_data = np.append(gradient_data, np.array([norm(gradient)]))
    
    # update loop control parameters
    count+=1
    if count >= MAX_LIMIT:
        print('Max iterations exceeded')
        break

output = open('output.txt', "w")
for position in path:
    line = str(position[0]) + ',' + str(position[1])
    output.write(line)
    output.write('\n')
output.close()

# =================#
#       Plots      #
# =================#

# plot the gradient along the path
plt.figure()
plt.plot(gradient_data)
plt.grid()
plt.xlabel('Iterations')
plt.ylabel('Norm of the Gradient')

# plot the potential energy field and its gradient

def f(x, y):
    return np.sin(x**2 + y**2)

MeshSize = 60
x = np.linspace(0, 6, MeshSize)
y = np.linspace(0, 6, MeshSize)

X, Y = np.meshgrid(x, y)
Z = np.zeros([MeshSize, MeshSize])
gradZ = np.zeros([MeshSize, MeshSize])

for y in range(MeshSize):
    for x in range(MeshSize):
        Z[y][x] = potentialField(X[y][x], Y[y][x])

fig = plt.figure()
ax = plt.axes(projection = '3d')
# obstacles
#for P in obstacleList:
#    ax.plot(np.append(P[:, 0], P[0,0]), np.append(P[:, 1], P[0, 1]))
# potential field
ax.contour3D(X, Y, Z, 70)
# path of the robot
#ax.plot(path[:,0], path[:, 1], 'y', linestyle = 'dashed')
ax.set_xlabel('x')
ax.set_ylabel('y')
#ax.set_zlabel('z')
ax.grid()

plt.show()

