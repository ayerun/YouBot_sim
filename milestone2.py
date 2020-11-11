import numpy as np
import matplotlib.pyplot as plt
from modern_robotics import *

'''
Input:
X The initial configuration of the end-effector in the reference trajectory: Tse,initial.
X The cube's initial configuration: Tsc,initial.
X The cube's desired final configuration: Tsc,final.
X The end-effector's configuration relative to the cube when it is grasping the cube: Tce,grasp.
X The end-effector's standoff configuration above the cube, before and after grasping, relative to the cube: Tce,standoff. 
X The number of trajectory reference configurations per 0.01 seconds: k. The value k is an integer with a value of 1 or greater.

Output:
CSV containing Tse for entire trajectory
'''

def TrajectoryGenerator(Tse_i,Tsc_i,Tsc_f,Tce_g,Tce_s,k):
    
    '''trajectory segment 1'''

    #convert standoff configuration 
    Tse_si = np.dot(Tsc_i,Tce_s)

    #trajectory parameters
    Tf = 10
    N = (Tf*k)/0.01

    #calculate trajectory
    seg1 = ScrewTrajectory(Tse_i,Tse_si,Tf,N,3)

    #Write to csv file
    f = open('seg.csv','w')
    for i in range(int(np.shape(seg1)[0]/k)):
        myoutput = " %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f\n" % (seg1[k*i][0][0],seg1[k*i][0][1],seg1[k*i][0][2],seg1[k*i][1][0],seg1[k*i][1][1],seg1[k*i][1][2],seg1[k*i][2][0],seg1[k*i][2][1],seg1[k*i][2][2],seg1[k*i][0][3],seg1[k*i][1][3],seg1[k*i][2][3],0)
        f.write(myoutput)



#End effector initial configuration
Tse_i = np.array([[0,0,1,0],[0,1,0,0],[-1,0,0,0.5],[0,0,0,1]])

#Cube initial and final configurations
Tsc_i = np.array([[1,0,0,1],[0,1,0,0],[0,0,1,0.025],[0,0,0,1]])
Tsc_f = np.array([[0,1,0,0],[-1,0,0,-1],[0,0,1,0.025],[0,0,0,1]])

#Grasp configuration
Tce_i = np.dot(TransInv(Tsc_i),Tse_i)
Tce_g = Tce_i
Tce_g[0][3] = 0
Tce_g[1][3] = 0
Tce_g[2][3] = 0

#Standoff configuration
Tce_s = Tce_g
Tce_s[0][3] = 0
Tce_s[1][3] = 0
Tce_s[2][3] = 0.225

#reference configurations per 0.01 seconds
k = 2

#Generate Trajectory
TrajectoryGenerator(Tse_i,Tsc_i,Tsc_f,Tce_g,Tce_s,k)
