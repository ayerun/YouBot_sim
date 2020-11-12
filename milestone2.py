import numpy as np
import matplotlib.pyplot as plt
from modern_robotics import *

#To generate the include csv file run the following command in your Linux terminal:
#python3 (path_to_file)/milestone2.py

def TrajectoryGenerator(Tse_i,Tsc_i,Tsc_f,Tce_g,Tce_s,k):
    
    '''
    Input:
    Tse_i: The initial configuration of the end-effector in the reference trajectory: Tse,initial.
    Tsc_i: The cube's initial configuration: Tsc,initial.
    Tsc_f: The cube's desired final configuration: Tsc,final.
    Tce_g: The end-effector's configuration relative to the cube when it is grasping the cube: Tce,grasp.
    Tce_s: The end-effector's standoff configuration above the cube, before and after grasping, relative to the cube: Tce,standoff. 
    k:     The number of trajectory reference configurations per 0.01 seconds: k. The value k is an integer with a value of 1 or greater.

    Output:
    Traj: matrix containing Tse at all time steps
    seg.csv: csv containing Tse for entire trajectory
    '''

    '''
    Trajectory Segment 1
    Move to Standoff Position 1
    '''
    #end effector configuration in first standoff
    Tse_si = np.dot(Tsc_i,Tce_s)

    #trajectory parameters
    Tf1 = 7
    N1 = (Tf1*k)/0.01

    #calculate trajectory
    seg1 = ScrewTrajectory(Tse_i,Tse_si,Tf1,N1,3)

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
    Tf2 = 2
    N2 = (Tf2*k)/0.01
    
    #calculate trajectory
    seg2 = ScrewTrajectory(Tse_si,Tse_g,Tf2,N2,3)

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
    seg4 = ScrewTrajectory(Tse_g,Tse_si,Tf2,N2,3)

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

    #calculate trajectory
    seg5 = ScrewTrajectory(Tse_si,Tse_sf,Tf1,N1,3)

    #write to csv file
    for i in range(int(np.shape(seg5)[0]/k)):
        myoutput = " %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f\n" % (seg5[k*i][0][0],seg5[k*i][0][1],seg5[k*i][0][2],seg5[k*i][1][0],seg5[k*i][1][1],seg5[k*i][1][2],seg5[k*i][2][0],seg5[k*i][2][1],seg5[k*i][2][2],seg5[k*i][0][3],seg5[k*i][1][3],seg5[k*i][2][3],1)
        f.write(myoutput)

    '''
    Trajectory Segment 6
    Move to Placing configuration
    '''
    #end effector configuration for placing
    Tse_p = np.dot(Tsc_f,Tce_g)
    
    #calculate trajectory
    seg6 = ScrewTrajectory(Tse_sf,Tse_p,Tf2,N2,3)

    #write to csv file
    for i in range(int(np.shape(seg6)[0]/k)):
        myoutput = " %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f\n" % (seg6[k*i][0][0],seg6[k*i][0][1],seg6[k*i][0][2],seg6[k*i][1][0],seg6[k*i][1][1],seg6[k*i][1][2],seg6[k*i][2][0],seg6[k*i][2][1],seg6[k*i][2][2],seg6[k*i][0][3],seg6[k*i][1][3],seg6[k*i][2][3],1)
        f.write(myoutput)

    '''
    Trajectory Segment 7
    Place Cube
    '''
    #write to csv file
    for i in range(63):
        myoutput = " %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f\n" % (seg6[-1][0][0],seg6[-1][0][1],seg6[-1][0][2],seg6[-1][1][0],seg6[-1][1][1],seg6[-1][1][2],seg6[-1][2][0],seg6[-1][2][1],seg6[-1][2][2],seg6[-1][0][3],seg6[-1][1][3],seg6[-1][2][3],0)
        f.write(myoutput)

    '''
    Trajectory Segment 8
    Return to Standoff configuration
    '''
    #calculate trajectory
    seg8 = ScrewTrajectory(Tse_p,Tse_sf,Tf2,N2,3)

    #write to csv file
    for i in range(int(np.shape(seg8)[0]/k)):
        myoutput = " %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f\n" % (seg8[k*i][0][0],seg8[k*i][0][1],seg8[k*i][0][2],seg8[k*i][1][0],seg8[k*i][1][1],seg8[k*i][1][2],seg8[k*i][2][0],seg8[k*i][2][1],seg8[k*i][2][2],seg8[k*i][0][3],seg8[k*i][1][3],seg8[k*i][2][3],0)
        f.write(myoutput)
    
    '''
    Output Trajectory
    '''
    #Initialize trajectory matrix
    rows = (N1*2+N2*4)/2+63*2
    traj = np.zeros((int(rows),13))

    #Populate Trajectory Matrix
    for i in range(int(np.shape(seg1)[0]/k)):
        traj[i][0]=seg1[k*i][0][0]
        traj[i][1]=seg1[k*i][0][1]
        traj[i][2]=seg1[k*i][0][2]
        traj[i][3]=seg1[k*i][1][0]
        traj[i][4]=seg1[k*i][1][1]
        traj[i][5]=seg1[k*i][1][2]
        traj[i][6]=seg1[k*i][2][0]
        traj[i][7]=seg1[k*i][2][1]
        traj[i][8]=seg1[k*i][2][2]
        traj[i][9]=seg1[k*i][0][3]
        traj[i][10]=seg1[k*i][1][3]
        traj[i][11]=seg1[k*i][2][3]
        traj[i][12]=0
    for i in range(int(np.shape(seg2)[0]/k)):
        traj[int(N1/k)+i][0]=seg2[k*i][0][0]
        traj[int(N1/k)+i][1]=seg2[k*i][0][1]
        traj[int(N1/k)+i][2]=seg2[k*i][0][2]
        traj[int(N1/k)+i][3]=seg2[k*i][1][0]
        traj[int(N1/k)+i][4]=seg2[k*i][1][1]
        traj[int(N1/k)+i][5]=seg2[k*i][1][2]
        traj[int(N1/k)+i][6]=seg2[k*i][2][0]
        traj[int(N1/k)+i][7]=seg2[k*i][2][1]
        traj[int(N1/k)+i][8]=seg2[k*i][2][2]
        traj[int(N1/k)+i][9]=seg2[k*i][0][3]
        traj[int(N1/k)+i][10]=seg2[k*i][1][3]
        traj[int(N1/k)+i][11]=seg2[k*i][2][3]
        traj[int(N1/k)+i][12]=0
    for i in range(63):
        i = int(i+N1/k+N2/k)
        traj[i][0]=seg2[-1][0][0]
        traj[i][1]=seg2[-1][0][1]
        traj[i][2]=seg2[-1][0][2]
        traj[i][3]=seg2[-1][1][0]
        traj[i][4]=seg2[-1][1][1]
        traj[i][5]=seg2[-1][1][2]
        traj[i][6]=seg2[-1][2][0]
        traj[i][7]=seg2[-1][2][1]
        traj[i][8]=seg2[-1][2][2]
        traj[i][9]=seg2[-1][0][3]
        traj[i][10]=seg2[-1][1][3]
        traj[i][11]=seg2[-1][2][3]
        traj[i][12]=1
    for i in range(int(np.shape(seg4)[0]/k)):
        i2 = int(i+N1/k+N2/k+63)
        traj[i2][0]=seg4[k*i][0][0]
        traj[i2][1]=seg4[k*i][0][1]
        traj[i2][2]=seg4[k*i][0][2]
        traj[i2][3]=seg4[k*i][1][0]
        traj[i2][4]=seg4[k*i][1][1]
        traj[i2][5]=seg4[k*i][1][2]
        traj[i2][6]=seg4[k*i][2][0]
        traj[i2][7]=seg4[k*i][2][1]
        traj[i2][8]=seg4[k*i][2][2]
        traj[i2][9]=seg4[k*i][0][3]
        traj[i2][10]=seg4[k*i][1][3]
        traj[i2][11]=seg4[k*i][2][3]
        traj[i2][12]=1
    for i in range(int(np.shape(seg5)[0]/k)):
        i2 = int(i+N1/k+N2/k+N2/k+63)
        traj[i2][0]=seg5[k*i][0][0]
        traj[i2][1]=seg5[k*i][0][1]
        traj[i2][2]=seg5[k*i][0][2]
        traj[i2][3]=seg5[k*i][1][0]
        traj[i2][4]=seg5[k*i][1][1]
        traj[i2][5]=seg5[k*i][1][2]
        traj[i2][6]=seg5[k*i][2][0]
        traj[i2][7]=seg5[k*i][2][1]
        traj[i2][8]=seg5[k*i][2][2]
        traj[i2][9]=seg5[k*i][0][3]
        traj[i2][10]=seg5[k*i][1][3]
        traj[i2][11]=seg5[k*i][2][3]
        traj[i2][12]=1
    for i in range(int(np.shape(seg6)[0]/k)):
        i2 = int(i+N1/k+N1/k+N2/k+N2/k+63)
        traj[i2][0]=seg6[k*i][0][0]
        traj[i2][1]=seg6[k*i][0][1]
        traj[i2][2]=seg6[k*i][0][2]
        traj[i2][3]=seg6[k*i][1][0]
        traj[i2][4]=seg6[k*i][1][1]
        traj[i2][5]=seg6[k*i][1][2]
        traj[i2][6]=seg6[k*i][2][0]
        traj[i2][7]=seg6[k*i][2][1]
        traj[i2][8]=seg6[k*i][2][2]
        traj[i2][9]=seg6[k*i][0][3]
        traj[i2][10]=seg6[k*i][1][3]
        traj[i2][11]=seg6[k*i][2][3]
        traj[i2][12]=1
    for i in range(63):
        i2 = int(i+N1/k+N1/k+N2/k+N2/k+N2/k+63)
        traj[i2][0]=seg6[-1][0][0]
        traj[i2][1]=seg6[-1][0][1]
        traj[i2][2]=seg6[-1][0][2]
        traj[i2][3]=seg6[-1][1][0]
        traj[i2][4]=seg6[-1][1][1]
        traj[i2][5]=seg6[-1][1][2]
        traj[i2][6]=seg6[-1][2][0]
        traj[i2][7]=seg6[-1][2][1]
        traj[i2][8]=seg6[-1][2][2]
        traj[i2][9]=seg6[-1][0][3]
        traj[i2][10]=seg6[-1][1][3]
        traj[i2][11]=seg6[-1][2][3]
        traj[i2][12]=0
    for i in range(int(np.shape(seg8)[0]/k)):
        i2 = int(i+N1/k+N1/k+N2/k+N2/k+N2/k+63*2)
        traj[i2][0]=seg8[k*i][0][0]
        traj[i2][1]=seg8[k*i][0][1]
        traj[i2][2]=seg8[k*i][0][2]
        traj[i2][3]=seg8[k*i][1][0]
        traj[i2][4]=seg8[k*i][1][1]
        traj[i2][5]=seg8[k*i][1][2]
        traj[i2][6]=seg8[k*i][2][0]
        traj[i2][7]=seg8[k*i][2][1]
        traj[i2][8]=seg8[k*i][2][2]
        traj[i2][9]=seg8[k*i][0][3]
        traj[i2][10]=seg8[k*i][1][3]
        traj[i2][11]=seg8[k*i][2][3]
        traj[i2][12]=0

    return traj




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

#Reference configurations per 0.01 seconds
k = 2

#Generate Trajectory
traj = TrajectoryGenerator(Tse_i,Tsc_i,Tsc_f,Tce_g,Tce_s,k)
