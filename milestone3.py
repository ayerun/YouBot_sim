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
    Vd = se3ToVec(np.matmul(TransInv(Xd),(Xd_next-Xd)/dt))
    Ad = Adjoint(np.matmul(TransInv(X),Xd))
    Xe_i += dt*Xe
    V = np.dot(Ad,Vd)+np.dot(Kp,Xe)+np.dot(Ki,Xe_i)

    return V

# Given
config = np.array([0,0,0,0,0,0.2,-1.6,0])
Xd = np.array([[0,0,1,0.5],[0,1,0,0],[-1,0,0,0.5],[0,0,0,1]])
Xd_next = np.array([[0,0,1,0.6],[0,1,0,0],[-1,0,0,0.3],[0,0,0,1]])
X = np.array([[0.170,0,0.985,0.387],[0,1,0,0],[-0.985,0,0.170,0.570],[0,0,0,1]])
Kp = np.zeros((6,6))
Ki = Kp
dt = 0.01
Xe_i = 0
V = FeedbackControl(X,Xd,Xd_next,Kp,Ki,dt)
