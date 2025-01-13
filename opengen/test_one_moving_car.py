import opengen as og
import matplotlib.pyplot as plt
import numpy as np

from constants import *
from solver import main as compile_solver
from interface import simulate as simulate
from interface import collision_detector
from SolverError import SolverError

def get_obstacle_definition(velocity):
        return [(102.3, 2, 0, velocity)]

def get_nearest_distance(x_history, obstacles_history):
    collisions = []
    min_dist = 10000
    for step, t in enumerate(np.arange(0, T * DT, DT)):
        for obstacle in range(OBS_N):
            x_delta = x_history[step, 0] - obstacles_history[obstacle, step, 0]
            y_delta = x_history[step, 1] - obstacles_history[obstacle, step, 1]
            distance = np.sqrt(x_delta**2 + y_delta**2)

            if distance < min_dist:
                min_dist = distance
                collisions = (obstacle, obstacles_history[obstacle, step, 0], obstacles_history[obstacle, step, 1], distance)

    return collisions


def main():
    """a function to test the traffic density which the solver can tollerate"""
    compile_solver()

    mng = og.tcp.OptimizerTcpManager('my_optimizers/navigation_obstacle', port=12345)
    failed = 0
    mng.start()
    mng.ping()

    for velocity in np.linspace(-0, -2.5, 51):

        # Initial car state: x, y, theta, velocity (v), steering angle (phi)
        x = np.array([0, 0, 0, 0, 0])

        obstacle_definition = get_obstacle_definition(velocity)

        try:
            x_history, u_history, obstacle_history = simulate(mng, x, obstacle_definition) 
            
            x_history = np.array(x_history)
            u_history = np.array(u_history)
            obstacle_history = np.array(obstacle_history)
            collisions = collision_detector(x_history, obstacle_history)
            #print(get_nearest_distance(x_history, obstacle_history)[3])
            
            if collisions == []:
                 print(f"solver success for velocity = {velocity}")
            else:
                 
                 print(f"Collision at v={velocity}: {collisions}")

        except SolverError:
            failed += 1
            print(f"SolverError: solver failed to converge for velocity = {velocity}") 
        
    print(f"failed: {failed}")
    # Close the TCP connection
    mng.kill()


if __name__ == "__main__":
    print(f"starting test")
    main()