import opengen as og
import casadi.casadi as cs
import matplotlib.pyplot as plt
import numpy as np
import time

from constants import *
import splinterp as sp
import obstacle_generator as obs_gen

def car_ode(x, u):

    """an ODE for the state space of the car in the form 
    x[k+1] = f(x[k],u[k])
    
    Parameters:
    -----------
    x (ndarray): state of car at time k, contains 
                 [x-pos, y-pos, psi, v, delta, a]
    u (ndarray): input to car at time k, contains
                 [ddelta, jerk]

    Returns:
    --------
    x (ndarray): the state of the car at time k+1, in same form as above
    """
    x, y, psi, v, delta = x
    x += v * np.cos(psi) * DT
    y += v * np.sin(psi) * DT
    psi += v * np.sin(delta) * DT
    psi = (psi + np.pi) % (2 * np.pi) - np.pi

    v += u[1] * DT
    delta += u[0] * DT
    return np.array([x, y, psi, v, delta])

def collision_detector(x_history, obstacles_history):
    for step, t in enumerate(np.arange(0, T * DT, DT)):
        for obstacle in range(OBS_N):
            x_delta = x_history[step, 0] - obstacles_history[obstacle, step, 0]
            y_delta = x_history[step, 1] - obstacles_history[obstacle, step, 1]
            distance = np.sqrt(x_delta**2 + y_delta**2)

            if distance < 1:
                print(f"Collision with obstacle {obstacle} at x={round(x_history[step, 0],1)} and y={round(x_history[step,1],1)}. Time={round(t, 2)}. Distance={round(1 - distance, 2)}")




def main():
    """The primary control loop for simulating the car's motion through state space

    Parameters:
    -----------
    None

    Returns:
    --------
    None
    """
    mng = og.tcp.OptimizerTcpManager('my_optimizers/navigation_obstacle', port=12346)

    mng.start()
    mng.ping()

    # Initial car state: x, y, theta, velocity (v), steering angle (phi)
    x = np.array([0, 0, 0, 0, 0])

    u_history = []  # Store control inputs over time
    x_history = []  # Store state trajectories over time
    obstacle_history = []
    for i in range(OBS_N):
        obstacle_history.append([])

    start = time.process_time()
    f2_norms = []

    for t in np.arange(0, T * DT, DT):
        # Generate the goal trajectory
        goal = sp.generate_guide_trajectory(x)
        obstacles = obs_gen.get_obstacle_list(t)
        for i in range(OBS_N):
            obstacle_history[i].append((obstacles[2*i:2*(i+1)]))

        # Call the optimizer with current state and goal
        solution = mng.call([*x, *goal, *obstacles], initial_guess=[0.0] * (NU * N))
        if solution.is_ok():
            u = solution.get().solution[:NU]
            status = solution.get().exit_status
            f2_norms.append((solution.get().f2_norm, t))
            if status != "Converged":
                print(f"f2: {solution.get().f2_norm}")
                print(f"Warning! {solution.get().exit_status} at {round(t, 2)} s")
                print(solution.get().num_inner_iterations)
                print(solution.get().num_outer_iterations)
        else:
            err = solution.get()
            raise ValueError(f"time {t}: {err.message}")

        # Update the car's state and append history
        x_history.append(x)
        u_history.append(u)

        x = car_ode(x, u)   # update car state    

    stop = time.process_time()
    print(f"Solved in {round(stop - start, 2)} s")
    print(f"Max f2: {max(f2_norms, key= lambda x: x[0])}")

    # Close the TCP connection
    mng.kill()

    # Convert histories to numpy arrays for easier plotting
    x_history = np.array(x_history)
    u_history = np.array(u_history)
    obstacle_history = np.array(obstacle_history)
    print(f"obstacle history shape = {obstacle_history.shape}")


    collision_detector(x_history, obstacle_history)

    # ---- Plot Results ----

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
    obs = 0
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
    main()
