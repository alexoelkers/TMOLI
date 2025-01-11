from constants import *
import random

obstacles_parameters = {
    "num_moving": 0,
    "distance_between_moving": 3,
    "num_static": 1,
    "distance_between_static": 20,
    "v_mu": -2,
    "v_sigma": 0.5,
    "arrival_x": 90,
    "arrival_time": 19,
    "moving_y": 3
}

def every_other(i):
    return 1 if i % 2 else -1

#(spawn x, spawn y, spawn time, x-velocity)
def create_obstacle_list(p):
    num_param_obs = p["num_moving"] + p["num_static"]
    if num_param_obs > OBS_N:
        raise ValueError(f"Number of obstacles in obstacles_parameter is larger than solver OBS_N. Set OBS_N >= {num_param_obs}")

    obstacles = []
    for i in range(obstacles_parameters["num_static"]):
        obstacles.append((30 + p["distance_between_static"]*i, 0, 0, 0))

    for i in range(obstacles_parameters["num_moving"]):
        reference_x = p["arrival_x"] - p["arrival_time"] * p["v_mu"]
        this_x = reference_x + every_other(i) * p["distance_between_moving"]
        this_v = random.normalvariate(p["v_mu"], p["v_sigma"])
        obstacles.append((this_x, p["moving_y"], 0, this_v))

    return obstacles

def _get_obstacle_locations(time):
    obstacle_list = create_obstacle_list(obstacles_parameters)
    len_obstacles = len(obstacle_list)

    if len_obstacles < OBS_N:
        obstacle_list += [(-10 - 25*i, 0, 0, 0) for i in range(OBS_N - len_obstacles)]

    locations = []
    for obstacle in obstacle_list:
        if time >= obstacle[2]:
            x = obstacle[0] + (time - obstacle[2]) * obstacle[3]
            y = obstacle[1] 
        else:
            #If the obstacle hasn't spawned yet, put in in the dummy position
            x = -10
            y = -10

        locations.extend([x, y])

    return locations

def get_obstacle_list(time) -> list:
    locations = []
    for i in range(N):
        locations.extend(_get_obstacle_locations(time + DT*i))

    return locations


if __name__ == "__main__":
    import matplotlib.pyplot as plt
    import numpy as np

    obstacles = get_obstacle_list(2)

    fig, ax = plt.subplots()
    print(obstacles)
    ax.scatter(obstacles[::OBS_N*2], obstacles[1::OBS_N*2])
