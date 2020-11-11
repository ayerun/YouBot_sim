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
    
    '''
    Trajectory Segment 1
    Move to Standoff Position 1
    '''
    #end effector configuration in first standoff
    Tse_si = np.dot(Tsc_i,Tce_s)

    #trajectory parameters
    Tf = 7
    N = (Tf*k)/0.01

    #calculate trajectory
    seg1 = ScrewTrajectory(Tse_i,Tse_si,Tf,N,3)

    #write to csv file
    f = open('seg.csv','w')
    for i in range(int(np.shape(seg1)[0]/k)):
        myoutput = " %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f\n" % (seg1[k*i][0][0],seg1[k*i][0][1],seg1[k*i][0][2],seg1[k*i][1][0],seg1[k*i][1][1],seg1[k*i][1][2],seg1[k*i][2][0],seg1[k*i][2][1],seg1[k*i][2][2],seg1[k*i][0][3],seg1[k*i][1][3],seg1[k*i][2][3],0)
        f.write(myoutput)

    '''
    Trajectory Segment 2
    Move to Grasping configuration
    '''
    #end effector configuration for grasping
    Tse_g = np.dot(Tsc_i,Tce_g)

    #trajectory parameters
    Tf = 2
    N = (Tf*k)/0.01
    
    #calculate trajectory
    seg2 = ScrewTrajectory(Tse_si,Tse_g,Tf,N,3)

    #write to csv file
    for i in range(int(np.shape(seg2)[0]/k)):
        myoutput = " %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f\n" % (seg2[k*i][0][0],seg2[k*i][0][1],seg2[k*i][0][2],seg2[k*i][1][0],seg2[k*i][1][1],seg2[k*i][1][2],seg2[k*i][2][0],seg2[k*i][2][1],seg2[k*i][2][2],seg2[k*i][0][3],seg2[k*i][1][3],seg2[k*i][2][3],0)
        f.write(myoutput)

    '''
    Trajectory Segment 3
    Grasp Cube
    '''
    #write to csv file
    for i in range(63):
        myoutput = " %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f\n" % (seg2[-1][0][0],seg2[-1][0][1],seg2[-1][0][2],seg2[-1][1][0],seg2[-1][1][1],seg2[-1][1][2],seg2[-1][2][0],seg2[-1][2][1],seg2[-1][2][2],seg2[-1][0][3],seg2[-1][1][3],seg2[-1][2][3],1)
        f.write(myoutput)

    '''
    Trajectory Segment 4
    Return to Standoff Position 1
    '''
    #calculate trajectory
    seg4 = ScrewTrajectory(Tse_g,Tse_si,Tf,N,3)

    #write to csv file
    for i in range(int(np.shape(seg4)[0]/k)):
        myoutput = " %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f\n" % (seg4[k*i][0][0],seg4[k*i][0][1],seg4[k*i][0][2],seg4[k*i][1][0],seg4[k*i][1][1],seg4[k*i][1][2],seg4[k*i][2][0],seg4[k*i][2][1],seg4[k*i][2][2],seg4[k*i][0][3],seg4[k*i][1][3],seg4[k*i][2][3],1)
        f.write(myoutput)

    '''
    Trajectory Segment 5
    Move to Standoff Position 2
    '''
    #end effector configuration in final standoff
    Tse_sf = np.dot(Tsc_f,Tce_s)

    #trajectory parameters
    Tf = 7
    N = (Tf*k)/0.01

    #calculate trajectory
    seg5 = ScrewTrajectory(Tse_si,Tse_sf,Tf,N,3)

    #write to csv file
    for i in range(int(np.shape(seg5)[0]/k)):
        myoutput = " %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f\n" % (seg5[k*i][0][0],seg5[k*i][0][1],seg5[k*i][0][2],seg5[k*i][1][0],seg5[k*i][1][1],seg5[k*i][1][2],seg5[k*i][2][0],seg5[k*i][2][1],seg5[k*i][2][2],seg5[k*i][0][3],seg5[k*i][1][3],seg5[k*i][2][3],1)
        f.write(myoutput)

#End effector initial configuration
Tse_i = np.array([[0,0,1,0],[0,1,0,0],[-1,0,0,0.5],[0,0,0,1]])

#Cube initial and final configurations
Tsc_i = np.array([[1,0,0,1],[0,1,0,0],[0,0,1,0.025],[0,0,0,1]])
Tsc_f = np.array([[0,1,0,0],[-1,0,0,-1],[0,0,1,0.025],[0,0,0,1]])

#Grasp configuration
Tce_i = np.dot(TransInv(Tsc_i),Tse_i)
Tce_g = np.copy(Tce_i)
Tce_g[0][3] = 0
Tce_g[1][3] = 0
Tce_g[2][3] = 0

#Standoff configuration
Tce_s = np.copy(Tce_g)
Tce_s[0][3] = 0
Tce_s[1][3] = 0
Tce_s[2][3] = 0.225

#reference configurations per 0.01 seconds
k = 2

#Generate Trajectory
TrajectoryGenerator(Tse_i,Tsc_i,Tsc_f,Tce_g,Tce_s,k)
