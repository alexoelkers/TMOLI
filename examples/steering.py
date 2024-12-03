import numpy as np
import matplotlib.pyplot as plt

from urdfenvs.urdf_common.urdf_env import UrdfEnv
from urdfenvs.urdf_common.bicycle_model import BicycleModel

MAX_STEER = 0.87270631
L_ARROW = 0.5
R_GOAL = 0.1

def run_prius(goal_queue, n_steps=1000, render=False, goal=True, obstacles=True):
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
    action = np.array([0, 0])
    goal = goal_queue.pop(0)
    pos0 = np.array([0, 0, 0])
    ob = env.reset(pos=pos0)
    ob, *_ = env.step(action)
    print(f"Initial observation : {ob}")
    history = []
    control_action = []
    for i in range(n_steps):
        # get goal 
        if distance_metric(ob, goal) <= R_GOAL:
            if len(goal_queue) == 0:
                print(f"path complete")
                break
            goal = goal_queue.pop(0)

        # get action
        action = controller(ob, goal)
        control_action.append(action)

        # run action
        ob, *_ = env.step(action)
        history.append(ob)
    
    print(f"Final observations: {history[-1]}")
    env.close()
    return history, control_action

def distance_metric(state, goal):
    """a simple distance metric"""
    position = state['robot_0']['joint_state']['position']
    return np.hypot(goal[1] - position[1], goal[0] - position[0]) # euclidian distance

def controller(state, goal):
    """A simple controller to send the current state towards the goal"""
    # find the angle to the goal
    position = state['robot_0']['joint_state']['position']
    angle_to_goal = np.arctan2(goal[1] - position[1], goal[0] - position[0])
    steering_angle = np.arccos(np.cos(angle_to_goal) * np.cos(position[2]) 
                               + np.sin(angle_to_goal) * np.sin(position[2]))
    return np.array([1, steering_angle])

def create_plots(state_history, input_history, goal_queue):
    """a function to create plots for a test run"""
    fig, ax = plt.subplots()
    fig2, ax2, = plt.subplots()

    time = np.arange(0, 0.01*len(state_history), 0.01)
    x = []
    y = []
    steering_angle = []

    for state, control_action in zip(state_history, input_history):
        current_pos = state['robot_0']['joint_state']['position']
        x.append(current_pos[0])
        y.append(current_pos[1])
        steering_angle.append(state['robot_0']['joint_state']['steering'][0] + current_pos[2])

    ax.plot(x, y)
    # ax2.plot(np.arange(0, 100, 0.01), np.array(input_history))
    for i, x_point, y_point, theta in zip(range(len(x)), x, y, steering_angle):
        if i % 57 == 0:
            ax.arrow(x_point, y_point, L_ARROW*np.cos(theta), L_ARROW*np.sin(theta), width=0.02)

    for goal in goal_queue:
        ax.scatter(goal[0], goal[1])
    
    ax.set_aspect("equal")
    fig.savefig("./examples/circle_car.png")
    fig2.savefig("./examples/input.png")
    print(f"plotting succesful")

if __name__ == "__main__":
    goal_queue = [(-5, 5), (5, -5), (5, 5), (-5, -5)]
    history, control_action = run_prius(goal_queue, n_steps=10000, render=False)
    create_plots(history, control_action, goal_queue)