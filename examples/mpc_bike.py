import numpy as np

"""a simplified toolkit for solving an MPC bicycle model"""

DT = 0.01

def discrete_bicycle(state, action):
    x, y, theta = state
    velocity, steering_angle = action

    return np.array([x + DT*velocity*np.cos(theta),
                     y + DT*velocity*np.sin(theta),
                     theta + DT*velocity*np.sin(steering_angle)])


