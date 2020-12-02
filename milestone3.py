import numpy as np
import matplotlib.pyplot as plt
from modern_robotics import *

# chassis phi, chassis x, chassis y, J1, J2, J3, J4, J5, W1, W2, W3, W4, gripper state

def FeedbackControl(Tse,Tse_d,Tse_d_next,Kp,Ki,dt):
    '''
    Args:
        Tse: current actual end-effector configuration X (also written Tse)
        Tse_d: current end-effector reference configuration Xd (i.e., Tse,d)
        Tse_d_next: end-effector reference configuration at the next timestep in the reference trajectory, Xd,next (i.e., Tse,d,next), at a time Δt later
        Kp: proportional gain
        Kd: derivative gain
        dt: timestep Δt between reference trajectory configurations
    Returns:
        V: commanded end-effector twist 
    '''
    return 0

