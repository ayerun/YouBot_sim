import numpy as np
import matplotlib.pyplot as plt
from modern_robotics import *

# chassis phi, chassis x, chassis y, J1, J2, J3, J4, J5, W1, W2, W3, W4, gripper state

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

    Xe = se3ToVec(MatrixLog6(np.matmul(TransInv(X),Xd)))
    Vd = se3ToVec(MatrixLog6(np.matmul(TransInv(Xd),Xd_next))/dt)
    Ad = Adjoint(np.matmul(TransInv(X),Xd))
    Xe_i += dt*Xe
    V = np.dot(Ad,Vd)+np.dot(Kp,Xe)+np.dot(Ki,Xe_i)

    return V

def Get_Jacobians(config):
    '''
    Args:
        config: [phi,x,y,theta1,theta2,theta3,theta4,theta5]
    Returns:
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
    return Jarm,Jbase,Je,Je_inv

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

#Given
config = np.array([0,0,0,0,0,0.2,-1.6,0])   #phi,x,y,th1,th2,th3,th4,th5
Xd = np.array([[0,0,1,0.5],[0,1,0,0],[-1,0,0,0.5],[0,0,0,1]])
Xd_next = np.array([[0,0,1,0.6],[0,1,0,0],[-1,0,0,0.3],[0,0,0,1]])
X = np.array([[0.170,0,0.985,0.387],[0,1,0,0],[-0.985,0,0.170,0.570],[0,0,0,1]])
# Kp = np.zeros((6,6))
Kp = np.identity(6)
Ki = np.zeros((6,6))
dt = 0.01
Xe_i = 0

V = FeedbackControl(X,Xd,Xd_next,Kp,Ki,dt)

Jarm,Jbase,Je,Je_inv = Get_Jacobians(config)

vels = np.matmul(Je_inv,V)
print(vels)