from constants import *

#(spawn x, spawn y, spawn time, x-velocity)
def get_obstacle_definition():
    return [(XG + 10, OBS_Y, 1, -1), 
            (50, 0, 0, 0), 
            (XG + 10, OBS_Y, 3, -1), 
            (XG + 10, OBS_Y, 4, -1), 
            (XG + 10, OBS_Y, 5, -1)]

def _get_obstacle_locations(time):
    obstacle_list = get_obstacle_definition()
    len_obstacles = len(obstacle_list)

    if len_obstacles < OBS_N:
        obstacle_list += [(-10, -10, 0, 0)] * (OBS_N - len_obstacles)

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
    print(obstacles[::OBS_N*2])
    ax.scatter(obstacles[::OBS_N*2], obstacles[1::OBS_N*2])
