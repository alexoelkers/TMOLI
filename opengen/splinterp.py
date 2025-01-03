from utils import *
import numpy as np

"""a simple library for spline interpolation through state space
used for interpolation of control points in MPC"""

def next_guide_point(guide_state):
    x, y, phi, v, delta, a = guide_state
    if x < XG - K:
        next_state = [x + VG * DT,
                      Y0, 
                      0,
                      VG,
                      0,
                      0,]
    elif x >= XG - K and y < Y0 + K:
        #theta = np.atan((x + K - XG)/(y - Y0))
        theta = np.atan2(x + K - XG, K - (y + Y0))
        next_theta = theta + OMEGAG * DT
        print(f"theta = {theta}, next theta = {next_theta}")
        next_state = [XG + K*(np.sin(next_theta) - 1),
                      Y0 + K*(1 - np.cos(next_theta)),
                      next_theta,
                      VG, 
                      np.asin(1 / K),
                      0]
    elif y >= Y0 + K:
        next_state = [XG,
                      y + VG * DT,
                      np.pi / 2,
                      VG,
                      0,
                      0]
    return np.array(next_state)

def generate_guide_trajectory(state):
    """a function to generate a guide trajectory for the mpc controller"""
    trajectory = []
    for n in range(N):
        state = next_guide_point(state)
        trajectory.extend(state)
    return trajectory

if __name__ == "__main__":
    import matplotlib.pyplot as plt 

    state = [XG - (K + 1), 
             Y0, 
             0, 
             VG, 
             0,
             0]
    
    trajectory = np.array(generate_guide_trajectory(state))
    trajectory = trajectory.reshape((N, NX))
    fig, ax = plt.subplots()
    ax.scatter(trajectory[:, 0], trajectory[:, 1])
    ax.set(aspect="equal")