#!/usr/bin/env python3

import rospy
from sc627_helper.msg import ObsData
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import numpy as np
from tf.transformations import euler_from_quaternion

ANG_MAX = math.pi/18
VEL_MAX = 0.15
ITERS = 1000
i = 0

current_bot = { # pose data of current bot
    'pos' : np.array([0, 0]), # position
    'vel' : np.array([0, 0]), # velocity
    'yaw' : 0, # yaw
}

left_bot = { # pose data of left bot
    'pos' : np.array([0, 0]), # position
    'vel' : np.array([0, 0]), # velocity
    'yaw' : 0, # yaw
}

right_bot = { # pose data of right bot
    'pos' : np.array([0, 0]), # position
    'vel' : np.array([0, 0]), # velocity
    'yaw' : 0, # yaw
}

def velocity_convert(x, y, theta, vel_x, vel_y):
    '''
    Robot pose (x, y, theta)  Note - theta in (0, 2pi)
    Velocity vector (vel_x, vel_y)
    '''

    gain_ang = 1 #modify if necessary
    
    ang = math.atan2(vel_y, vel_x)
    if ang < 0:
        ang += 2 * math.pi
    
    ang_err = min(max(ang - theta, -ANG_MAX), ANG_MAX)

    v_lin =  min(max(math.cos(ang_err) * math.sqrt(vel_x ** 2 + vel_y ** 2), -VEL_MAX), VEL_MAX)
    v_ang = gain_ang * ang_err
    return v_lin, v_ang

# def computePose(vec):
#     vec = vec/np.linalg.norm(vec)
#     x = vec[0]
#     y = vec[1]
#     if x > 0 and y >= 0:
#         theta = np.arctan(y/x)
#     elif x < 0 and y >= 0:
#         theta = np.pi - np.arctan(abs(y/x))
#     elif x < 0 and y < 0:
#         theta = np.pi + np.arctan(abs(y/x))
#     elif x > 0 and y < 0:
#         theta = 2*np.pi - np.arctan(abs(y/x))
#     elif x == 0 and y > 0:
#         theta = np.pi/2
#     elif x == 0 and y < 0:
#         theta = 3*np.pi/2

#     return theta # between 0 to 2*pi radians

def callback_odom(data):
    '''
    Get robot data
    '''
    # global current_bot
    current_bot['pos'][0] = data.pose.pose.position.x
    current_bot['pos'][1] = data.pose.pose.position.y*0
    orient = data.pose.pose.orientation
    (roll, pitch, yaw) = euler_from_quaternion([orient.x, orient.y, orient.z, orient.w])
    if yaw < 0:
        yaw += 2 * math.pi
    current_bot['yaw'] = yaw
    current_bot['vel'][0] = data.twist.twist.linear.x
    current_bot['vel'][1] = 0*data.twist.twist.linear.y
    print(data)

    return 0

def callback_left_odom(data):
    '''
    Get left robot data
    '''
    print('left robot')
    # rospy.loginfo(data.pose.pose.position)
    # global left_bot
    left_bot['pos'][0] = data.pose.pose.position.x
    left_bot['pos'][1] = data.pose.pose.position.y*0
    orient = data.pose.pose.orientation
    (roll, pitch, yaw) = euler_from_quaternion([orient.x, orient.y, orient.z, orient.w])
    if yaw < 0:
        yaw += 2 * math.pi
    left_bot['yaw'] = yaw
    left_bot['vel'][0] = data.twist.twist.linear.x
    left_bot['vel'][1] = 0*data.twist.twist.linear.y

    return 0

def callback_right_odom(data):
    '''
    Get right robot data
    '''
    print('right robot')
    # global right_bot
    right_bot['pos'][0] = data.pose.pose.position.x
    right_bot['pos'][1] = data.pose.pose.position.y*0
    orient = data.pose.pose.orientation
    (roll, pitch, yaw) = euler_from_quaternion([orient.x, orient.y, orient.z, orient.w])
    if yaw < 0:
        yaw += 2 * math.pi
    right_bot['yaw'] = yaw
    right_bot['vel'][0] = data.twist.twist.linear.x
    right_bot['vel'][1] = 0*data.twist.twist.linear.y

    return 0

rospy.init_node('balancing', anonymous = True)
rospy.Subscriber('/odom', Odometry, callback_odom) #topic name fixed
rospy.Subscriber('/left_odom', Odometry, callback_left_odom) #topic name fixed
rospy.Subscriber('/right_odom', Odometry, callback_right_odom) #topic name fixed

pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
r = rospy.Rate(30)

for i in range(1000): #replace with balancing reached?
    #calculate v_x, v_y as per the balancing strategy
    
    #Make sure your velocity vector is feasible (magnitude and direction)
    current_bot['vel'][0] = (left_bot['pos'][0] + right_bot['pos'][0])/2 - current_bot['pos'][0]
    current_bot['vel'][1] = - current_bot['pos'][1]
    current_bot['pos'][0] = current_bot['pos'][0] + 1.5*current_bot['vel'][0]
    current_bot['pos'][1] = current_bot['pos'][1] + 0.01*current_bot['vel'][1]
    
    #convert velocity vector to linear and angular velocties using velocity_convert function given above
    x = current_bot['pos'][0]
    y = current_bot['pos'][1]
    theta = current_bot['yaw'] #*computePose(current_bot['vel']/np.linalg.norm(current_bot['vel']))
    vel_x = current_bot['vel'][0]
    vel_y = current_bot['vel'][1]
    v_lin, v_ang = velocity_convert(x, y, theta, vel_x, vel_y)
    
    #publish the velocities below
    vel_msg = Twist()
    vel_msg.linear.x = v_lin
    vel_msg.angular.z = v_ang
    pub_vel.publish(vel_msg)
    
    #store robot path with time stamps (data available in odom topic)
    
    r.sleep()



