import numpy as np
import matplotlib.pyplot as plt

from urdfenvs.urdf_common.urdf_env import UrdfEnv
from urdfenvs.urdf_common.bicycle_model import BicycleModel

MAX_STEER = 0.87270631

def run_prius(n_steps=1000, render=False, goal=True, obstacles=True):
    fig, ax = plt.subplots()
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
    action = np.array([1, MAX_STEER])
    pos0 = np.array([0, 0, -np.pi/2])
    ob = env.reset(pos=pos0)
    print(f"Initial observation : {ob}")
    history = []
    x = []
    y = []
    for i in range(n_steps):
        ob, *_ = env.step(action)
        history.append(ob)
        current_pos = ob['robot_0']['joint_state']['position']
        x.append(current_pos[0])
        y.append(current_pos[1])
    ax.plot(x, y)
    ax.set_aspect("equal")
    fig.savefig("./examples/circle_car")
    print(f"Final observations: {history[-1]}")
    env.close()
    return history


if __name__ == "__main__":
    history = run_prius(n_steps=1000, render=False)