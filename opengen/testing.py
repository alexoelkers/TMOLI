import numpy as np
import matplotlib.pyplot as plt

from constants import *
from solver import main as compile_solver
from interface import main as run_solver

def plot_results(x_history, u_history, obstacle_history, obs):
    # Plot car state variables over time
    fig1, ax1 = plt.subplots()
    ax1.plot(np.arange(0, T * DT, DT), x_history[:, 2:], label=["Theta", "V", "Phi"])
    ax1.set_title("State Variables Over Time")
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("State Values")
    ax1.legend(["Theta (Heading)", "Velocity (V)", "Steering Angle (Phi)"])
    print(f"max state variables: {np.max(x_history, 0)}")


    # Plot car trajectory
    fig2, ax2 = plt.subplots()
    ax2.plot(x_history[:, 0], x_history[:, 1], label="Car Trajectory")
    ax2.set(aspect="equal")
    ax2.set_title("Car trajectory")
    ax2.legend()

    # Plot lateral acceleration
    fig3, ax3 = plt.subplots()
    ax3.plot(np.arange(0, T * DT, DT), np.tan(x_history[:, 4]) * x_history[:, 3] ** 2 / L)
    ax3.set_title("Lateral Acceleration")

    # Plot obstacle and car positions over time
    fig4, ax4 = plt.subplots()
    ax4.plot(np.arange(0, T * DT, DT), x_history[:, 0], label="Car X Position", linestyle="-", color="blue")
    ax4.plot(np.arange(0, T * DT, DT), x_history[:, 1], label="Car Y Position", linestyle="-", color="green")
    
    # plot obstacle 2 position
    ax4.plot(np.arange(0, T * DT, DT), obstacle_history[obs, :, 0], label="Obs X Position", linestyle="-", color="red")
    ax4.plot(np.arange(0, T * DT, DT), obstacle_history[obs, :, 1], label="Obs Y Position", linestyle="-", color="orange")
    ax4.set_title("Car Positions Over Time")
    ax4.set_xlabel("Time (s)")
    ax4.set_ylabel("Position (m)")
    ax4.legend()

    # Plot car state variables over time
    fig5, ax5 = plt.subplots()
    ax5.plot(np.arange(0, T * DT, DT), u_history[:, :], label=["u[0]", "u[1]"])
    ax5.set_title("State Variables Over Time")
    ax5.set_xlabel("Time (s)")
    ax5.set_ylabel("State Values")
    ax5.legend(["u[0]", "Acceleration"])
    # Show all plots
    plt.show()

if __name__ == "__main__":
    user_input = input("Recompile solver? [y/N]")
    if user_input == "Y" or user_input == "y":
        compile_solver()
    # define vehicle initial state
    x0 = np.array([0, 0, 0, 0, 0])

    # define turn parameters
    y0, x_goal, v_goal, turn_r = Y0, XG, VG, K
    turn_params = (y0, x_goal, v_goal, turn_r)
    x_history, u_history, obstacle_history, collision_status, overlap = run_solver(x0, turn_params)
    plot_results(x_history, u_history, obstacle_history, 0)