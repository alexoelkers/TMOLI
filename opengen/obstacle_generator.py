from constants import *

#(spawn x, spawn y, spawn time, x-velocity)
def get_obstacle_definition():
    return [(10, 0, 0, 0),
            (102.5, 1.93, 18.7, -2)]

def _get_obstacle_locations(time):
    obstacle_list = get_obstacle_definition()
    len_obstacles = len(obstacle_list)

    if len_obstacles < OBS_N:
        obstacle_list += [(-100, -100, 0, 0)] * (OBS_N - len_obstacles)

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
