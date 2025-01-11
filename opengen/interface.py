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
                return True, 1 - distance
                # print(f"Collision with obstacle {obstacle} at x={round(x_history[step, 0],1)} and y={round(x_history[step,1],1)}. Time={round(t, 2)}. Distance={round(1 - distance, 2)}")


# Função para limpar todos os arquivos em um diretório
def clear_directory(directory):
    for filename in os.listdir(directory):
        file_path = os.path.join(directory, filename)
        try:
            if os.path.isfile(file_path):  # Verifica se é um arquivo
                os.unlink(file_path)  # Remove o arquivo
        except Exception as e:
            print(f"Erro ao remover o arquivo {file_path}: {e}")

# Diretório de saída
output_dir = "obstaculos_posicoes"

# Limpa o diretório antes de salvar os novos arquivos
clear_directory(output_dir)




def main(x0, turn_params):
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
    x = x0

    u_history = []  # Store control inputs over time
    x_history = []  # Store state trajectories over time
    obstacle_history = []   # store obstacle positions over time
    for i in range(OBS_N):
        obstacle_history.append([])

    # start = time.process_time()
    f2_norms = []

    for t in np.arange(0, T * DT, DT):
        # Generate the goal trajectory
        goal = sp.generate_guide_trajectory(x, *turn_params)
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

    # stop = time.process_time()
    # print(f"Solved in {round(stop - start, 2)} s")
    # print(f"Max f2: {max(f2_norms, key= lambda x: x[0])}")

    # Close the TCP connection
    mng.kill()

    # Convert histories to numpy arrays for easier plotting
    x_history = np.array(x_history)
    u_history = np.array(u_history)
    obstacle_history = np.array(obstacle_history)

    collision_status, overlap = collision_detector(x_history, obstacle_history)


    return x_history, u_history, obstacle_history, collision_status, overlap


if __name__ == "__main__":
    # define vehicle initial state
    x0 = np.array([0, 0, 0, 0, 0])

    # define turn parameters
    y0, x_goal, v_goal, turn_r = Y0, XG, VG, K
    turn_params = (y0, x_goal, v_goal, turn_r)
    x_history, u_history, obstacle_history, collision_status, overlap = main(x0, turn_params)
