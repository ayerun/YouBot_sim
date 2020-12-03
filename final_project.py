import numpy as np
import matplotlib.pyplot as plt
from modern_robotics import *
import logging

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
    #Reverse dthetalist
    dthetalist = np.array([dthetalist[4],dthetalist[5],dthetalist[6],dthetalist[7],dthetalist[8],dthetalist[0],dthetalist[1],dthetalist[2],dthetalist[3]])

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

    '''
    Trajectory Segment 4
    Return to Standoff Position 1
    '''
    #calculate trajectory
    seg4 = ScrewTrajectory(Tse_g,Tse_si,Tf2,N2,3)

    '''
    Trajectory Segment 5
    Move to Standoff Position 2
    '''
    #end effector configuration in final standoff
    Tse_sf = np.dot(Tsc_f,Tce_s)

    #calculate trajectory
    seg5 = ScrewTrajectory(Tse_si,Tse_sf,Tf1,N1,3)

    '''
    Trajectory Segment 6
    Move to Placing configuration
    '''
    #end effector configuration for placing
    Tse_p = np.dot(Tsc_f,Tce_g)
    
    #calculate trajectory
    seg6 = ScrewTrajectory(Tse_sf,Tse_p,Tf2,N2,3)

    '''
    Trajectory Segment 8
    Return to Standoff configuration
    '''
    #calculate trajectory
    seg8 = ScrewTrajectory(Tse_p,Tse_sf,Tf2,N2,3)
    
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

def FeedbackControl(X,Xd,Xd_next,Kp,Ki,dt):
    '''
    Args:
        X: current actual end-effector configuration X (also written Tse)
        Xd: current end-effector reference configuration Xd (i.e., Tse,d)
        Xd_next: end-effector reference configuration at the next timestep in the reference trajectory, Xd,next (i.e., Tse,d,next), at a time Δt later
        Kp: proportional gain
        Kd: derivative gain
        dt: timestep Δt between reference trajectory configurations
    Returns:
        V: commanded end-effector twist 
    '''
    global Xe_i #numerical integral of error from 0 to t
    global time

    Xe = se3ToVec(MatrixLog6(np.matmul(TransInv(X),Xd)))
    Xe_list[time] = Xe
    Vd = se3ToVec(MatrixLog6(np.matmul(TransInv(Xd),Xd_next))/dt)
    Ad = Adjoint(np.matmul(TransInv(X),Xd))
    Xe_i += dt*Xe
    V = np.dot(Ad,Vd)+np.dot(Kp,Xe)+np.dot(Ki,Xe_i)

    # Write to csv
    myoutput = " %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f, %10.6f\n" % (dt*time,Xe[0],Xe[1],Xe[2],Xe[3],Xe[4],Xe[5])
    e.write(myoutput)

    # increment timer
    time+=1

    return V

def Get_Jacobians(config):
    '''
    Args:
        config: [phi,x,y,theta1,theta2,theta3,theta4,theta5]
    Returns:
        Tse: end effector configuration
        Jarm: Jacobian of arm
        Jbase: Jacobian of chassis
        Je: Jacobian of entire robot
        Je_inv: pseudoinverse of Je
    '''
    # Jarm calculation
    Jarm = JacobianBody(Blist,config[3:])

    T0e = FKinBody(M0e,Blist,config[3:])
    Tsb = np.array([[np.cos(config[0]),-np.sin(config[0]),0,config[1]],
                [np.sin(config[0]),np.cos(config[0]),0,config[2]],
                [0,0,1,0.0963],
                [0,0,0,1]])
    Tbe = np.matmul(Tb0,T0e)
    Tse = np.matmul(Tsb,Tbe)

    # Jbase calculation
    Jbase = np.matmul(Adjoint(TransInv(Tbe)),F6)

    # Pseudoinverse calculation
    Je = np.concatenate((Jbase,Jarm),axis=1)
    Je_inv = np.linalg.pinv(Je)
    
    #Round Jacobians
    for i in range(np.shape(Je)[0]):
        for j in range(np.shape(Je)[1]):
            Je[i][j] = round(Je[i][j],5)
    for i in range(np.shape(Je_inv)[0]):
        for j in range(np.shape(Je_inv)[1]):
            Je_inv[i][j] = round(Je_inv[i][j],5)
    return Tse,Jarm,Jbase,Je,Je_inv

def Pick_and_Place(Tsc_i,Tsc_f,config_a,config_r):
    '''
    Args:
        Tsc_i: initial resting configuration of the cube object (which has a known geometry), represented by a frame attached to the center of the object
        Tsc_f: desired final resting configuration of the cube object
        config_a: actual initial configuration of the youBot
        config_r: reference initial configuration of the youBot (which will generally be different from the actual initial configuration, to allow you to test feedback control)
    Returns:
        csv file containing robot configuration for Coppelia Sim
        csv file containing the 6-vector end-effector error
    '''
    #Calculate Tse_i (reference end effector initial configuration)
    T0e = FKinBody(M0e,Blist,config_r[3:8])
    Tsb = np.array([[np.cos(config_r[0]),-np.sin(config_r[0]),0,config_r[1]],
                [np.sin(config_r[0]),np.cos(config_r[0]),0,config_r[2]],
                [0,0,1,0.0963],
                [0,0,0,1]])
    Tbe = np.matmul(Tb0,T0e)
    Tse_i = np.matmul(Tsb,Tbe)

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

    #Generate Reference Trajectory
    traj = TrajectoryGenerator(Tse_i,Tsc_i,Tsc_f,Tce_g,Tce_s,k)
    
    #Loop through reference trajectory
    for i in range(np.shape(traj)[0]-1):
        Xd = np.array([[traj[i][0],traj[i][1],traj[i][2],traj[i][9]],
                       [traj[i][3],traj[i][4],traj[i][5],traj[i][10]],
                       [traj[i][6],traj[i][7],traj[i][8],traj[i][11]],
                       [0,0,0,1]])
        Xd_next = np.array([[traj[i+1][0],traj[i+1][1],traj[i+1][2],traj[i+1][9]],
                            [traj[i+1][3],traj[i+1][4],traj[i+1][5],traj[i+1][10]],
                            [traj[i+1][6],traj[i+1][7],traj[i+1][8],traj[i+1][11]],
                            [0,0,0,1]])
        
        # Calculate Tse and Jacobians
        X,Jarm,Jbase,Je,Je_inv = Get_Jacobians(config_a[:8])

        # Calculate End Effector Twist
        V = FeedbackControl(X,Xd,Xd_next,Kp,Ki,dt)

        # Calculate Controls
        controls = np.matmul(Je_inv,V)

        # Calculate next configuration
        config_a = NextState(config_a,controls,dt,dthetamax)
        config_a[-1] = traj[i+1][-1]

    return

#Create log
logging.basicConfig(filename="run.log",level=logging.INFO,format = '%(asctime)s %(processName)s %(name)s %(message)s')

# Constants
r = 0.0475
l = 0.47/2
w = 0.3/2
Blist = np.array([[0,0,1,0,0.033,0],
                  [0,-1,0,-0.5076,0,0],
                  [0,-1,0,-0.3526,0,0],
                  [0,-1,0,-0.2167,0,0],
                  [0,0,1,0,0,0]]).T
M0e = np.array([[1,0,0,0.033],
                [0,1,0,0],
                [0,0,1,0.6546],
                [0,0,0,1]])
Tb0 = np.array([[1,0,0,0.1662],
                [0,1,0,0],
                [0,0,1,0.0026],
                [0,0,0,1]])
F = 0.25*r*np.array([[-1/(l+w),1/(l+w),1/(l+w),-1/(l+w)],[1,1,1,1],[-1,1,-1,1]])
F6 = np.concatenate((np.zeros((2,np.shape(F)[0]+1)),F,np.zeros((1,np.shape(F)[0]+1))))
Ki_const = 0.2
Ki = Ki_const*np.identity(6)
Kp_const = 5.5
Kp = Kp_const*np.identity(6)
Xe_i = 0
Xe_list = np.zeros((2325,6))
dt = 0.01
time = 0
dthetamax = 20

#Cube initial and final configurations
Tsc_i = np.array([[1,0,0,1],[0,1,0,0],[0,0,1,0.025],[0,0,0,1]])
Tsc_f = np.array([[0,1,0,0],[-1,0,0,-1],[0,0,1,0.025],[0,0,0,1]])

#Actual robot configuration
config_a = np.array([0.6,-0.4159,0,0,0,0,3*np.pi/2,0,0,0,0,0,0])

#Reference robot configuration
config_r = np.array([0,-0.4159,0.2,0,0,0,3*np.pi/2,0,0,0,0,0,0])

# Open csv files
f = open('final.csv','w')
e = open('error.csv','w')

# Populate csv files
logging.info('Running program to generate .csv files for simulation and error plotting')
Pick_and_Place(Tsc_i,Tsc_f,config_a,config_r)

#Plot Error
logging.info('Creating error plot')
tlist = np.linspace(0,23.25,2325)
fig1 = plt.figure()
plt.plot(tlist,Xe_list[:,0])
plt.plot(tlist,Xe_list[:,1])
plt.plot(tlist,Xe_list[:,2])
plt.plot(tlist,Xe_list[:,3])
plt.plot(tlist,Xe_list[:,4])
plt.plot(tlist,Xe_list[:,5])
plt.xlabel('Time (s)')
plt.ylabel('X error')
plt.legend(['Xerr[0]','Xerr[1]','Xerr[2]','Xerr[3]','Xerr[4]','Xerr[5]'])
plt.title('YouBot Configuration Error Response')
plt.show()
logging.info('Program complete')