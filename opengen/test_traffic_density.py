import opengen as og
import matplotlib.pyplot as plt
import numpy as np

from constants import *
from solver import main as compile_solver
from interface import simulate as simulate
from interface import collision_detector
from SolverError import SolverError


def get_obstacle_definition(x0, spacing, velocity, y=1.93):
        return [(x0 + spacing*i, y, 0, velocity) for i in range(OBS_N)]


def main():
    """a function to test the traffic density which the solver can tollerate"""
    compile_solver()

    mng = og.tcp.OptimizerTcpManager('my_optimizers/navigation_obstacle', port=12345)

    mng.start()
    mng.ping()

    for spacing in np.linspace(20, 1, 25):

        # Initial car state: x, y, theta, velocity (v), steering angle (phi)
        x = np.array([0, 0, 0, 0, 0])

        obstacle_definition = get_obstacle_definition(130, spacing, -2)

        try:
            x_history, u_history, obstacle_history = simulate(mng, x, obstacle_definition) 
            print(f"solver success for spacing = {spacing}")
        except SolverError:
            print(f"SolverError: solver failed to converge for spacing = {spacing}") 
        

    # Close the TCP connection
    mng.kill()

    # Convert histories to numpy arrays for easier plotting
    x_history = np.array(x_history)
    u_history = np.array(u_history)
    obstacle_history = np.array(obstacle_history)
    print(f"spacing = {spacing}")


    collision_detector(x_history, obstacle_history)


if __name__ == "__main__":
    print(f"starting test")
    main()