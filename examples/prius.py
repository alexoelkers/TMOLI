import numpy as np
import matplotlib.pyplot as plt

from urdfenvs.urdf_common.urdf_env import UrdfEnv
from urdfenvs.urdf_common.bicycle_model import BicycleModel


def run_prius(n_steps=1000, render=False, goal=True, obstacles=True):
    goal_queue = [(3, 3), (6, 3), (6, 9), (9, 12)]
    r_goal = 0.1

    robots = [
        BicycleModel(
            urdf='prius.urdf',
            mode="vel",
            scaling=0.3,
            wheel_radius = 0.31265,
            wheel_distance = 0.494,
            spawn_offset = np.array([-0.435, 0.0, 0.05]),
            actuated_wheels=['front_right_wheel_joint', 'front_left_wheel_joint', 'rear_right_wheel_joint', 'rear_left_wheel_joint'],
            steering_links=['front_right_steer_joint', 'front_left_steer_joint'],
            facing_direction='-x'
        )
    ]
    env: UrdfEnv = UrdfEnv(
        dt=0.01, robots=robots, render=render
    )
    action = np.array([1, 0])
    pos0 = np.array([0, 0, 0])
    goal = goal_queue.pop(0)
    ob = env.reset(pos=pos0)
    print(f"Initial observation : {ob}")
    history = []
    for i in range(n_steps):
        ob, *_ = env.step(action)
        # current_pos = ob['robot_0']['joint_state']['position']
        action = pointing_controller(ob, goal)
        history.append(ob)
        if goal_euclid_distance(ob, goal) < r_goal:
            goal = goal_queue.pop(0)
    env.close()
    return history, env

def pointing_controller(state, goal):
    """a function which takes the current position of the car as well as the
    goal position (a point) and returns a control input for the vehicle.

    parameters
    ----------
    position(ndarray): [x, y, theta]
    goal(ndarray): [x, y]
    """
    # unless otherwise stated, all maths is in radians
    position = state['robot_0']['joint_state']['position']
    angle_to_goal = np.arctan2(position[0] - goal[0], position[1] - goal[1])
    delta_angle = np.arccos(np.cos(angle_to_goal) * np.cos(position[2]) 
                            + np.sin(angle_to_goal) * np.sin(position[2]))
    return np.array([1, -delta_angle])
    

def goal_euclid_distance(state, goal):
    """a function to check whether the robot has reached its goal"""
    position = state['robot_0']['joint_state']['position']
    return np.hypot(position[0] - goal[0], position[1] - goal[1])


if __name__ == "__main__":
    history, env = run_prius(n_steps=5000, render=True)

