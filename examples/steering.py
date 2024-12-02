import numpy as np
import matplotlib.pyplot as plt

from urdfenvs.urdf_common.urdf_env import UrdfEnv
from urdfenvs.urdf_common.bicycle_model import BicycleModel

MAX_STEER = 0.87270631
L_ARROW = 0.5

def run_prius(n_steps=1000, render=False, goal=True, obstacles=True):
    goal = np.array([-5, 4])
    fig, ax = plt.subplots()
    fig2, ax2, = plt.subplots()
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
    ob = env.reset(pos=pos0)
    print(f"Initial observation : {ob}")
    history = []
    x = []
    y = []
    steering_angle = []
    control_action = []
    for i in range(n_steps):
        ob, *_ = env.step(action)
        action = controller(ob, goal)
        control_action.append(action[1])
        history.append(ob)
        current_pos = ob['robot_0']['joint_state']['position']
        x.append(current_pos[0])
        y.append(current_pos[1])
        steering_angle.append(ob['robot_0']['joint_state']['steering'][0] + current_pos[2])
    steering_angle = np.array(steering_angle)
    ax.plot(x, y)
    ax.scatter(*goal)
    ax2.plot(np.arange(0, 10, 0.01), control_action)
    for i, x_point, y_point, theta in zip(range(len(x)), x, y, steering_angle):
        if i % 57 == 0:
            ax.arrow(x_point, y_point, L_ARROW*np.cos(theta), L_ARROW*np.sin(theta), width=0.02)
    ax.set_aspect("equal")
    fig.savefig("./examples/circle_car.png")
    fig2.savefig("./examples/input.png")
    print(f"Final observations: {history[-1]}")
    env.close()
    return history


def controller(state, goal):
    """A simple controller to send the current state towards the goal"""
    # find the angle to the goal
    position = state['robot_0']['joint_state']['position']
    angle_to_goal = np.arctan2(goal[1] - position[1], goal[0] - position[0])
    steering_angle = angle_to_goal - position[2]
    return np.array([1, steering_angle])


if __name__ == "__main__":
    history = run_prius(n_steps=1000, render=False)