#!/usr/bin/env python3

# Created on: Jan 30, 2021
# Author: 
# Dhananjay Tiwari, 
# 5th year, Dual Degree
# Mechanical Engineering, 
# Indian Institute of Technology Bombay
# stores all the functions required for bug_base and bug_1 implementation

import numpy as np
import matplotlib.pyplot as plt

# ------------
def norm(vec):
    # returns norm of a vec
    return np.linalg.norm(vec)

# 
def minArray(arr):
    # computes minimum value of a 1D array
    return min

# ----------------------------
def arePointsDistinct(p1, p2):
    # inputs
    # p1, p2 are two points on 2D plane

    # output
    # flag = 0, if points p1 and p2 are not distinct
    # flag = 1, if points p1 and p2 are distinct

    tol = 1e-8
    #print(p1, p2)
    if norm(p1-p2) <= tol:
        isDistinct = 0
    elif norm(p1-p2) > tol:
        isDistinct = 1

    return isDistinct

# --------------------------------------
def computeLineThroughTwoPoints(p1, p2):
    # inputs
    # p1, p2 are distinct points on 2D plane
    
    # outputs
    # [a, b, c], parameters defining the line {(x,y) | ax + by + c = 0} 
    # passing through p1 with slope m = (y1 - y2)/(x1 - x2)

    # if two are not distinct then the line does not exist
    if arePointsDistinct(p1, p2) == 0:
        print('Error inside computeLineThroughTwoPoints(p1, p2)')
        print('p1 and p2 are not distinct')
    
    # if x-coordinates of two points is same, then the slope is infinite
    elif p1[0] == p2[0] and arePointsDistinct(p1, p2) == 1:
        # line of the form x - x1 = 0 (normalized already)
        a = 1
        b = 0
        c = -p1[0]
    
    # a line with non-zero slope passing through two distinct points
    elif arePointsDistinct(p1, p2) == 1:
        m = (p1[1] - p2[1]) / (p1[0] - p2[0]) # slope
        
        # line of the form m*x - y - (m*x1 - y1) = 0
        a = m
        b = -1.0
        c = p1[1] - m * p1[0]
        
        # normalize
        if a != 0:
            factor = a/abs(a)*np.sqrt(a**2 + b**2)
            a = a/factor
            b = b/factor
            c = c/factor
        if a == 0:
            factor = b
            b = b/factor
            c = c/factor
            
    return a, b, c

# ----------------------------------------
def computeDistancePointToLine(q, p1, p2):
    # inputs
    # q, a point on 2D plane
    # p1, p2, distinct points on 2D plane through which the line passes

    # output
    # distance between the point q and the line passing through points p1 and p2
    # the orthogonal projection of q on the line through p1 and p2

    a, b, c = computeLineThroughTwoPoints(p1, p2)
    
    pq = np.array([0.0, 0.0])
    pq[0] = (b**2*q[0] - a*b*q[1] - a*c)/(a**2 + b**2)
    pq[1] = (a**2*q[1] - a*b*q[0] - b*c)/(a**2 + b**2)

    d_q_to_p1p2 = norm(pq - q)
    
    return d_q_to_p1p2, pq

# -------------------------------------------
def computeDistancePointToSegment(q, p1, p2):
    # inputs
    # q, a point on the 2D plane
    # p1, p2, distinct points on 2D plane

    # outputs
    # d distance between point q and the line segment joining p1 and p2
    # w indicator for the location of point q for three scenarios
    # w = 0, point closest to q is inside the segment
    # w = 1, q is closest to p1 on the line segment
    # w = 2, q is closest to p2 on the line segment
    # pq, the point of closest distance

    # find the shortest distance between line through p1, p2 and q
    dummy, pq = computeDistancePointToLine(q, p1, p2)
    
    if abs(norm(pq-p1) - norm(pq-p2)) < norm(p1-p2) - 2*1e-8 and norm(pq-p1) > 1e-8 and norm(pq-p2) > 1e-8:
        w = 0
        d = norm(pq - q)
    elif norm(pq-p1) - norm(pq-p2) + norm(p1 - p2) <= 1e-10 or norm(pq-p1) <= 1e-8:
        w = 1
        d = norm(p1 - q)
        pq = p1
    elif norm(pq-p1) - norm(pq-p2) - norm(p1 - p2) <= 1e-10 or norm(pq-p2) <= 1e-8:
        w = 2
        d = norm(p2 - q)
        pq = p2

    return np.append(np.array([d, w]), pq)

# --------------------------------------
def computeDistancePointToPolygon(P, q):
    # inputs
    # P is a polygon represented by a 2D array of points
    # q is a given point in 2D plane

    # output
    # the distance between point q and the closest point on the polygon
    # the closest point to q on the polygon

    # number of sides of the polygon
    numSides = np.shape(P)[0]
    
    # stores the output of computeDistancePointToSegment function
    pointsToSegOut = np.zeros([numSides, 4]) # d, w, pq
    
    for i in range(numSides):
        if i < numSides-1:
            pointsToSegOut[i, :] = computeDistancePointToSegment(q, P[i, :], P[i+1, :])
        elif i == numSides-1:
            pointsToSegOut[i, :] = computeDistancePointToSegment(q, P[i, :], P[0, :])

    d_pointToPolygon = min(pointsToSegOut[:,0])
    i_min = np.argmin(pointsToSegOut[:,0])
    w_min = pointsToSegOut[i_min, 1]
    closestPointOnPolygon = pointsToSegOut[i_min, 2:]
    
    return np.append(np.array([d_pointToPolygon, w_min, i_min]), closestPointOnPolygon)

def computeTangentVectorToPolygon(P, q):
    # inputs
    # P is a polygon represented by a 2D array of points
    # q is a given point in 2D plane

    # outputs
    # a 2D tangent vector to the polygon
    numSides = np.shape(P)[0]
    pointToPolygonOut = computeDistancePointToPolygon(P, q)
    w_min = pointToPolygonOut[1]
    i_min = int(pointToPolygonOut[2])
    closestPointOnPolygon = pointToPolygonOut[3:]
    
    if i_min < numSides-1:
        segVector = P[i_min+1, :] - P[i_min, :]
    if i_min == numSides-1:
        segVector = P[0, :] - P[i_min, :]

    if w_min == 0:
        tVector = segVector
        tVector = tVector/norm(tVector)
    elif w_min == 1 or w_min == 2:
        nVector = q - closestPointOnPolygon
        nVector = nVector/norm(nVector)
        tVector = np.array([-nVector[1], nVector[0]])

    return tVector