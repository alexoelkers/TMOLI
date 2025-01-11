import opengen as og
import casadi.casadi as cs
import matplotlib.pyplot as plt
import numpy as np
import time
import csv
import os

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
    x, y, psi, v, delta, a = x
    x += v * np.cos(psi) * DT
    y += v * np.sin(psi) * DT
    psi += v * np.sin(delta) * DT
    psi = (psi + np.pi) % (2 * np.pi) - np.pi

    v += a * DT
    delta += u[0] * DT
    a += u[1] * DT
    return np.array([x, y, psi, v, delta, a])


def main():
    """The primary control loop for simulating the car's motion through state space

    Parameters:
    -----------
    None

    Returns:
    --------
    None
    """
    mng = og.tcp.OptimizerTcpManager('my_optimizers/navigation_obstacle', port=12345)

    mng.start()
    mng.ping()

    # Initial car state: x, y, theta, velocity (v), steering angle (phi), acceleration (a)
    x = np.array([0, 0, 0, 1, 0, 0])

    u_history = []  # Store control inputs over time
    x_history = []  # Store state trajectories over time
    obstacle_history = []
    for i in range(OBS_N):
        obstacle_history.append([])

    start = time.process_time()

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
        else:
            err = solution.get()
            raise ValueError(f"time {t}: {err.message}")

        # Update the car's state and append history
        x_history.append(x)
        u_history.append(u)

        x = car_ode(x, u)   # update car state    

    stop = time.process_time()
    print(f"Solved in {round(stop - start, 2)} s")

    # Close the TCP connection
    mng.kill()

    # Convert histories to numpy arrays for easier plotting
    x_history = np.array(x_history)
    u_history = np.array(u_history)
    obstacle_history = np.array(obstacle_history)
    print(f"obstacle history shape = {obstacle_history.shape}")

    # ---- Plot Results ----

    # Plot car state variables over time
    fig1, ax1 = plt.subplots()
    ax1.plot(np.arange(0, T * DT, DT), x_history[:, 2:], label=["Theta", "V", "Phi", "A"])
    ax1.set_title("State Variables Over Time")
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("State Values")
    ax1.legend(["Theta (Heading)", "Velocity (V)", "Steering Angle (Phi)", "Acceleration (A)"])
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
    obs = 1
    print(obstacle_history[obs, :, :])
    ax4.plot(np.arange(0, T * DT, DT), obstacle_history[obs, :, 0], label="Obs X Position", linestyle="-", color="red")
    ax4.plot(np.arange(0, T * DT, DT), obstacle_history[obs, :, 1], label="Obs Y Position", linestyle="-", color="orange")
    ax4.set_title("Car Positions Over Time")
    ax4.set_xlabel("Time (s)")
    ax4.set_ylabel("Position (m)")
    ax4.legend()

    # Show all plots
    plt.show()

    with open('robot_data.csv', mode='w', newline='') as arquivo:
        writer = csv.writer(arquivo)
        writer.writerows(x_history)

    output_dir = "obstaculos_posicoes"
    os.makedirs(output_dir, exist_ok=True)

    # Salva as posições de cada obstáculo em ficheiros CSV diferentes
    for i, obstaculo in enumerate(obstacle_history):
        filename = os.path.join(output_dir, f"obstaculo_{i + 1}.csv")
    
    # Abre o ficheiro CSV para escrita
        with open(filename, mode='w', newline='') as file:
            writer = csv.writer(file)
        
            # Escreve as colunas de cabeçalho
            #writer.writerow(["Posição X", "Posição Y"])
        
            # Escreve as posições em cada instante
            for pos in obstaculo:
                writer.writerow(pos)

#print(f"Ficheiros CSV foram salvos na pasta '{output_dir}'.")


if __name__ == "__main__":
    main()
