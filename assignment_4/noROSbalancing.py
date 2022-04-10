
### IMPORT MODULES
import numpy as np
import matplotlib.pyplot as plt

### DEFINE CONSTANTS
N = 8 # number of robots
R = np.array([
    [0, 0],     # 0
    [1, 0],     # 1
    [2, 0],     # 2
    [3, 0],     # 3
    [4, 0],     # 4
    [5, 0],    # 5
    [5.5, 0],  # 6
    [14, 0]     # 7
]) # initialize robots locations on the x-axis
V_max = 0.15 # maximum velocity for each robot m/s
iters = 600
kp = 0.1
R_data = np.array([R])

for i in range(iters):
    for r in range(1, len(R)-1):
        midpt = (R[r-1,:] + R[r+1, :])/2
        R[r,:] = R[r, :] + kp * (midpt - R[r,:])
    
    R_data = np.append(R_data, np.array([R]), axis = 0)

leg = []
for r in range(len(R)):
    plt.plot(R_data[:, r, :])
    leg = leg + [r'R' + str(r)]

plt.xlabel(r'number of iterations')
plt.ylabel(r'Robot x-positions')
plt.grid()
plt.legend(leg)
plt.show()
