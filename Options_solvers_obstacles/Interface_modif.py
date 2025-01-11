import opengen as og
import casadi.casadi as cs
import matplotlib.pyplot as plt
import numpy as np

from constants import *
import splinterp as sp


# Simulation parameters
SIMTIME = 30  # Maximum allowable simulation time
T = int(SIMTIME / DT)  # Total simulation steps

def car_ode(x, u):
    """Car dynamics using a simple kinematic model."""
    x, y, theta, v, phi, a = x
    x += v * np.cos(theta) * DT
    y += v * np.sin(theta) * DT
    theta += v * np.sin(phi) * DT
    theta = (theta + np.pi) % (2 * np.pi) - np.pi
    v += a * DT
    phi += u[0] * DT
    a += u[1] * DT
    return np.array([x, y, theta, v, phi, a])


def main():
    # Start the TCP server for real-time optimization
    mng = og.tcp.OptimizerTcpManager('my_optimizers/navigation', port=12345)
    mng.start()
    mng.ping()

    # Initial car state: x, y, theta, velocity (v), steering angle (phi), acceleration (a)
    x = np.array([0, 0, 0, 4, 0, 0])

    u_history = []  # Store control inputs over time
    x_history = []  # Store state trajectories over time

    for t in np.arange(0, T * DT, DT):
        # Generate the goal trajectory
        goal = sp.generate_guide_trajectory(x)

        # Call the optimizer with current state and goal
        solution = mng.call([*x, *goal], initial_guess=[0.0] * (NU * N))

        if solution.is_ok():
            u = solution.get().solution[:NU]
        else:
            err = solution.get()
            raise ValueError(err.message)

        # Update the car's state and append history
        x_history.append(x)
        u_history.append(u)
        x = car_ode(x, u)

    # Close the TCP connection
    mng.kill()

    # Convert histories to numpy arrays for easier plotting
    x_history = np.array(x_history)
    u_history = np.array(u_history)

    # ---- Plot Results ----

    # Plot car state variables over time
    fig1, ax1 = plt.subplots()
    ax1.plot(np.arange(0, T * DT, DT), x_history[:, 1:], label=["Y", "Theta", "V", "Phi", "A"])
    ax1.set_title("State Variables Over Time")
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("State Values")
    ax1.legend(["Y Position", "Theta (Heading)", "Velocity (V)", "Steering Angle (Phi)", "Acceleration (A)"])

    # Plot car trajectory
    fig2, ax2 = plt.subplots()
    ax2.plot(x_history[:, 0], x_history[:, 1], label="Car Trajectory")
    ax2.set(aspect="equal")
    ax2.set_title("Car trajectory")
    ax2.legend()

    # Plot lateral acceleration
    fig3, ax3 = plt.subplots()
    ax3.plot(np.arange(0, T * DT, DT), x_history[:, 3] ** 2 / (L / (np.sin(x_history[:, 4]))))
    ax3.set_title("Lateral Acceleration")

    # Plot obstacle and car positions over time
    fig4, ax4 = plt.subplots()
    ax4.plot(np.arange(0, T * DT, DT), x_history[:, 0], label="Car X Position", linestyle="-", color="blue")
    ax4.plot(np.arange(0, T * DT, DT), x_history[:, 1], label="Car Y Position", linestyle="-", color="green")
    ax4.set_title("Car Positions Over Time")
    ax4.set_xlabel("Time (s)")
    ax4.set_ylabel("Position (m)")
    ax4.legend()

    # Show all plots
    plt.show()


if __name__ == "__main__":
    main()
