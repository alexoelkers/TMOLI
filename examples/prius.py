import numpy as np
import matplotlib.pyplot as plt

from urdfenvs.urdf_common.urdf_env import UrdfEnv
from urdfenvs.urdf_common.bicycle_model import BicycleModel


def run_prius(n_steps=1000, render=False, goal=True, obstacles=True):
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
    goal = np.array([0, 0])
    ob = env.reset(pos=pos0)
    print(f"Initial observation : {ob}")
    history = []
    for i in range(n_steps):
        ob, *_ = env.step(action)
        # current_pos = ob['robot_0']['joint_state']['position']
        action = pointing_controller(ob, goal)
        history.append(ob)
        if i % 200 == 0:
            # print(f"time = {i*0.01}")
            # print(f"position = {history[-1]['robot_0']['joint_state']['position']}")
            # print(f"{action}\n")
            pass
        elif i == 1001:
            goal = np.array([-100, 0])
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
    


if __name__ == "__main__":
    history, env = run_prius(n_steps=1000, render=False)
    print(f"success")
    positions = []
    for step in history:
        positions.append(step['robot_0']['joint_state']['position'])

    positions = np.array(positions)
    fig, ax = plt.subplots()
    ax.plot(positions[:, 0], positions[:, 1])
    plt.savefig("./examples/car_path")
    env.close()

