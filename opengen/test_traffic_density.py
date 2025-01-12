import opengen as og
import matplotlib.pyplot as plt
import numpy as np
import time 

from mpl_toolkits.axes_grid1.inset_locator import inset_axes

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

    test_spacings = np.linspace(20, 10, 20)
    spacing_success = []
    spacing_solve_time = []
    spacing_collisions = []

    for spacing in test_spacings:

        # Initial car state: x, y, theta, velocity (v), steering angle (phi)
        x = np.array([0, 0, 0, 0, 0])

        obstacle_definition = get_obstacle_definition(130, spacing, -2)

        try:
            start = time.time()
            x_history, u_history, obstacle_history = simulate(mng, x, obstacle_definition)
            solve_time = time.time() - start
            spacing_solve_time.append(solve_time)
            print(f"solver success for spacing = {spacing}")
            spacing_success.append(True)
            spacing_collisions.append(collision_detector(x_history, obstacle_history))

        except SolverError:
            print(f"SolverError: solver failed to converge for spacing = {spacing}") 
            spacing_solve_time.append(np.nan)
            spacing_success.append(False)
            spacing_collisions.append([])
        

    # Close the TCP connection
    mng.kill()

    spacing_solve_time = np.array(spacing_solve_time)
    spacing_success = np.array(spacing_success)
    # spacing_collisions = np.array(spacing_collisions)

    # Convert histories to numpy arrays for easier plotting

    # plot the solve time, and note minimum successful spacing
    fig, ax = plt.subplots()
    ax.scatter(test_spacings[spacing_success], spacing_solve_time[spacing_success])
    ax.set(ylabel="solve time [s]",
           xlabel="oncoming traffic spacing [m]",
           title=f"10 - 20m spacing, 20 runs")

    # plot success rate of solves
    success_fraction = np.sum(spacing_success) / len(spacing_success)
    ax2 = inset_axes(ax,
                            width="35%", # width = 30% of parent_bbox
                            height=1.5, # height : 1 inch
                            loc=1)
    # fig2, ax2 = plt.subplots()
    ax2.pie([success_fraction, 1 - success_fraction],
            labels=["success", "failure"],
            autopct='%1.1f%%')
    
    fig.tight_layout()
    fig.savefig("./plots/20_10m_spacing_no_IP.png")

if __name__ == "__main__":
    print(f"starting test")
    main()