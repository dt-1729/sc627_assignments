# Created on: Feb 1, 2021
# Author: 
# Dhananjay Tiwari, 
# 5th year, Dual Degree
# Mechanical Engineering, 
# Indian Institute of Technology Bombay
# tests the functions required for bug_base and bug_1 implementation

from tkinter.filedialog import test
import numpy as np
import matplotlib.pyplot as plt
from helper import *

tol = 1e-8

# ==========================
#       TEST FUNCTIONS
# ==========================

# ______________________________________________
def test_computeLineThroughTwoPoints(testcases):
    # inputs
    # testcases is an array of two points and expected line parameters, [p1.T, p2.T, a, b, c]
    
    # output
    # number of testcases passed
    print('_________________________________________________________\n')
    print('Testing computeLineThroughTwoPoints(p1, p2)')
    numTests = np.shape(testcases)[0]
    successCount = 0
    failCount = 0
    failedCases = []

    for i in range(numTests):
        a,  b,  c  = computeLineThroughTwoPoints(testcases[i, 0:2], testcases[i, 2:4])
        ai, bi, ci = testcases[i, 4:]
    
        # normalize testcase line parameters
        factor = ai/abs(ai)*np.sqrt(ai**2 + bi**2)
        ai = ai/factor
        bi = bi/factor
        ci = ci/factor

        if norm(np.array([a, b, c]) - np.array([ai, bi, ci])) <= tol:
            successCount = successCount + 1
        else:
            failCount   = failCount + 1
            failedCases = failedCases + [i]
            print('--------------------------')
            print(a, b, c, '\n', ai, bi, ci)
            print('--------------------------')

    if failCount == 0:
        print(successCount, 'out of', numTests, 'testcases passed!')
        print('_________________________________________________________\n')
    

    elif failCount > 0:  
        print(failCount, ' out of ', numTests, ' failed\n', 'Failed cases : ', failedCases)
        print('_________________________________________________________\n')

    return 0

def test_computeDistancePointToLine(testcases):
    # inputs
    # tescases is an array of three points and the distance, q, p1, p2, d
    
    # outputs
    # test result
    print('_________________________________________________________\n')
    print('Testing computeDistancePointToLine(p1, p2)')
    numTests = np.shape(testcases)[0]
    successCount = 0
    failCount = 0
    failedCases = []

    for i in range(numTests):
        d, pq = computeDistancePointToLine(testcases[i, 0:2], testcases[i, 2:4], testcases[i, 4:6])
        ai, bi, ci = computeLineThroughTwoPoints(testcases[i, 2:4], testcases[i, 4:6])

        if abs(d - testcases[i, 6]) <= tol and abs(d - norm(pq-testcases[i, 0:2])) <= tol and abs(pq[0]*ai + pq[1]*bi + ci) <= tol:
            successCount = successCount + 1
        else:
            failCount = failCount + 1
            failedCases = failedCases + [i]
        
    if failCount == 0:
        print(successCount, 'out of', numTests, 'testcases passed!')
        print('_________________________________________________________\n')
    

    elif failCount > 0:  
        print(failCount, ' out of ', numTests, ' failed\n', 'Failed cases : ', failedCases)
        print('_________________________________________________________\n')
    
    return 0

def test_computeDistancePointToSegment(testcases):
    # inputs
    # tescases is an array of three points, the distance and a flag, q, p1, p2, d, w
    
    # outputs
    # test result
    print('_________________________________________________________\n')
    print('Testing computeDistancePointToSegment(p1, p2)')
    numTests = np.shape(testcases)[0]
    successCount = 0
    failCount = 0
    failedCases = []

    for i in range(numTests):
        out = computeDistancePointToSegment(testcases[i, 0:2], testcases[i, 2:4], testcases[i, 4:6])
        d = out[0]
        w = out[1]
        #pq = out[2:]
        
        if d - testcases[i, 6] <= 1e-8 and w == testcases[i, 7]:
            successCount = successCount + 1
        else:
            failCount = failCount + 1
            failedCases = failedCases + [i]
            #print(d, w)
            #print(testcases[i, :])

    if failCount == 0:
        print(successCount, 'out of', numTests, 'testcases passed!')
        print('_________________________________________________________\n')
    
    elif failCount > 0:  
        print(failCount, ' out of ', numTests, ' failed\n', 'Failed cases : ', failedCases)
        print('_________________________________________________________\n')

    return 0

def test_computeDistancePointToPolygon(testcases):
    # inputs
    # testcases is a tuple of a polygon, a point and the output results, P, q, output array

    # outputs
    # test result
    
    print('_________________________________________________________\n')
    print('Testing computeDistancePointToPolygon(P, q)')
    numTests = np.shape(testcases)[0]
    successCount = 0
    failCount = 0
    failedCases = []

    for i in range(numTests):
        out = computeDistancePointToPolygon(testcases[i][0], testcases[i][1])
        if abs(out[0] - testcases[i][2][0]) <= 1e-10 and out[1] == testcases[i][2][1]:
            successCount = successCount + 1
        else:
            failCount = failCount + 1
            failedCases = failedCases + [i]
        
    
    if failCount == 0:
        print(successCount, 'out of', numTests, 'testcases passed!')
        print('_________________________________________________________\n')
    
    elif failCount > 0:  
        print(failCount, ' out of ', numTests, ' failed\n', 'Failed cases : ', failedCases)
        print('_________________________________________________________\n')
    
    return 0

def test_computeTangentVectorToPolygon(testcases):
    # inputs
    # testcases is a tuple of a polygon, a point and the output tangent vectors

    # outputs
    # test result

    print('_________________________________________________________\n')
    print('Testing computeTangentVectorToPolygon(P, q)')
    numTests = np.shape(testcases)[0]
    successCount = 0
    failCount = 0
    failedCases = []

    for i in range(numTests):
        tVector = computeTangentVectorToPolygon(testcases[i][0], testcases[i][1])
        if norm(tVector - testcases4[i][2]) <= 1e-8:
            successCount = successCount + 1
        else:
            failCount = failCount + 1
            failedCases = failedCases + [i]
        
    
    if failCount == 0:
        print(successCount, 'out of', numTests, 'testcases passed!')
        print('_________________________________________________________\n')
    
    elif failCount > 0:  
        print(failCount, ' out of ', numTests, ' failed\n', 'Failed cases : ', failedCases)
        print('_________________________________________________________\n')

    return 0

# ===============================
#         PERFORM TESTS
# ===============================

# _______________________________________________________________

# test line through two points
testcases0 = np.array([
    # x1,  y1,       x2,  y2,        ai,   bi,     ci
    [0.0, 0.0,      1.0, 1.0,       1.0, -1.0,    0.0], # line 1
    [1.0, 0.0,      0.0, 1.0,       1.0,  1.0,   -1.0], # line 2
    [1.5, 0.0,      1.5, 2.0,       1.0,  0.0,   -1.5], # line 3
    [1.7, 0.0,     1.72, 5.0,     250.0, -1.0, -425.0],  # line 4
    [0, -np.sqrt(3), -np.sqrt(5), 0, np.sqrt(3), np.sqrt(5), np.sqrt(15)]
])
test_computeLineThroughTwoPoints(testcases0)

# _______________________________________________________________

# test distance of a point from a line
testcases1 = np.array([
    # q,    p1,     p2,     d
    [0.0, 0.0,  1.0, 0.0,  0.0, 1.0,  1/np.sqrt(2)],
    [1.0, 0.0,  0.0, 0.0,  3.0, 3.0,  1/np.sqrt(2)],
    [3.0, 5.0,  7/3, 0.0,  0.0, 7/2,  12/np.sqrt(13)],
    [14.0, 11.0,  0.0, -4/5,  -4/np.sqrt(7), 0.0,  (59+14*np.sqrt(7))/np.sqrt(32)],
    [100.0, 140.0,  0.0, 0.0,  0.0, 5.0,  100.0],
    [21.0, 47.0,  0.0, 1438/7,  1438/14, 0.0,  815/np.sqrt(245)],
    [3.0, 0.0,  -2.0, 0.0,  0.0, 4.0,  2*np.sqrt(5)]
])
test_computeDistancePointToLine(testcases1)

# _______________________________________________________________

# test distance of a point from a line segment
testcases2 = np.array([
    # q,    p1,     p2,     d,      w
    [0.0, 0.0,  1.0, 0.0,  0.0, 1.0,  1/np.sqrt(2), 0],
    [1.0, 0.0,  0.0, 0.0,  3.0, 3.0,  1/np.sqrt(2),  0], # q is closer to p1
    [3.0, 5.0,  7/3, 0.0,  0.0, 7/2,  12/np.sqrt(13),  0],
    [0.0, 0.0,  0.0, 1.0,  2.0, 2.0,  1,  1],
    [3.0, 0.0,  -2.0, 0.0,  0.0, 4.0,  2*np.sqrt(5),  0],
    [10.0, 10.0,  -2.0, 0.0,  0.0, 4.0,  np.sqrt(136),  2],
    [-3.0, 0.0,  0.0, 4.0,  -2.0, 0.0,  1.0,  2],
    [1.0, 4.0,  1.0, 0.0,  1.0, 2.0,  3.0, 2]
])
test_computeDistancePointToSegment(testcases2)

# _______________________________________________________________

# test distance of a point from a polygon
P1 = np.array([
    [0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]
])
P2 = np.array([
    [1.0, 1.0], [-1.0, 2.0], [0.0, 0.0], [0.0, 0.5]
])
P3 = np.array([ 
    [4.0, 2.0], [0.0, 5.0], [-3.0, 5.0], [-4.0, -1.0], [4.0, -1.0]
])
P4 = np.array([ 
    [12.0, 5.0], [7.0, 12.0], [0.0, 0.0]
])

testcases3 = (
    (P1, np.array([1.5, 0.5]), np.array([0.5, 0, 1, 1.0, 0.5])),
    (P1, np.array([2.0, 2.0]), np.array([np.sqrt(2), 2, 1, 1.0, 1.0])),
    (P1, np.array([1.0, 4.0]), np.array([3.0, 2, 2, 1.0, 1.0])),
    (P3, np.array([6.0, 6.0]), np.array([4.4, 0, 0, 84/25, 72/25])),
    (P4, np.array([-2.0, -2.0]), np.array([np.sqrt(8), 2, 1, 0.0, 0.0]))
)

test_computeDistancePointToPolygon(testcases3)

# _______________________________________________________________

# test tangent of a polygon

testcases4 = (
    (P1, np.array([1.5, 0.5]), np.array([0.0, 1.0])),
    (P1, np.array([2.0, 2.0]), np.array([-1.0, 1.0])/np.sqrt(2)),
    (P1, np.array([1.0, 4.0]), np.array([-1.0, 0.0])),
    (P3, np.array([6.0, 6.0]), np.array([-4/5, 3/5])),
    (P4, np.array([-2.0, -2.0]), np.array([1, -1])/np.sqrt(2))
)

test_computeTangentVectorToPolygon(testcases4)