import numpy as np
import matplotlib.pyplot as plt
from modern_robotics import *

# chassis phi, chassis x, chassis y, J1, J2, J3, J4, J5, W1, W2, W3, W4, gripper state

def NextState(thetalist,dthetalist,dt,dthetamax):
    '''
    Args:
        thetalist: 12-vector representing the current configuration of the robot (3 variables for the chassis configuration, 5 variables for the arm configuration, and 4 variables for the wheel angles).
        dthetalist: 9-vector of controls indicating the arm joint speeds (5 variables) and the wheel speeds u (4 variables).
        dt: timestep Δt.
        dthetamax: positive real value indicating the maximum angular speed of the arm joints and the wheels.
    Returns:
        thetalist_f: 12-vector representing the configuration of the robot time Δt later.

    Use forward dynamics to get joint accelerations
    Use Euler step to get thlist_f
    '''
    # Constants
    r = 0.0475
    l = 0.47/2
    w = 0.3/2
    Tsb = np.array([[np.cos(thetalist[0]),-np.sin(thetalist[0]),0,thetalist[1]],
                [np.sin(thetalist[0]),np.cos(thetalist[0]),0,thetalist[2]],
                [0,0,1,0.0963],
                [0,0,0,1]])
    
    #remove gripper
    grip = thetalist[-1]
    thetalist = thetalist[:-1]

    # Enforce velocity limit
    for i in range(len(dthetalist)):
        if dthetalist[i] > dthetamax:
            dthetalist[i] = dthetamax
        elif dthetalist[i] < -dthetamax:
            dthetalist[i] = -dthetamax

    # Calculate change in wheel angle            
    dthetalist_wheel = dthetalist[5:]
    dth = dt*dthetalist_wheel

    # Calculate twist
    Vb = np.dot(0.25*r*np.array([[-1/(l+w),1/(l+w),1/(l+w),-1/(l+w)],
                        [1,1,1,1],
                        [-1,1,-1,1]]),dth)
    Vb6 = np.array([0,0,Vb[0],Vb[1],Vb[2],0])
    Vb_se3 = VecTose3(Vb6)

    # Transformations
    Tbb_prime = MatrixExp6(Vb_se3)
    Tsb_prime = np.matmul(Tsb,Tbb_prime)
    xb = Tsb_prime[0][3]
    yb = Tsb_prime[1][3]
    phi = np.arccos(Tsb_prime[0][0])
    
    # Arm & Wheel angles
    thetalist9 = thetalist[3:]
    
    # Euler Step for arm
    ddthetalist = np.array([0,0,0,0,0,0,0,0,0])
    thetalist_new,dthetalist_new = EulerStep(thetalist9, dthetalist, ddthetalist, dt)
    
    # Combine calculations
    thetalist_f = np.append(np.array([phi,xb,yb]),thetalist_new)
    thetalist_f = np.append(thetalist_f,grip)

    # Write to csv
    myoutput = " %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f\n" % (thetalist_f[0],thetalist_f[1],thetalist_f[2],thetalist_f[3],thetalist_f[4],thetalist_f[5],thetalist_f[6],thetalist_f[7],thetalist_f[8],thetalist_f[9],thetalist_f[10],thetalist_f[11],thetalist_f[12])
    f.write(myoutput)

    return thetalist_f



#Inputs
dthetalist = np.array([1,1,1,1,1,5,5,5,5])  #joint velocities then wheel velocities
dt = 0.01
dthetamax = 12
thetalist = np.array([0,0,0,0,0,0,0,0,0,0,0,0,0])

# Open csv
f = open('step.csv','w')

for i in range(100):
    thetalist = NextState(thetalist,dthetalist,dt,dthetamax)