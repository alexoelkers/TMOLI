from urdfenvs.urdf_common.urdf_env import UrdfEnv
import numpy as np

def plan(env: UrdfEnv, observation: dict[str, dict]) -> np.array:
    """
    Performs the planning action
    """

    obstacles = env.get_obstacles()

    if type(observation) != dict:
        raise TypeError("obs is not a dictionary")
    
    for robot in observation:
        x, y, angle = get_state(observation[robot], 'position')
        print(f"x: {x}, y: {y}, angle: {angle}")

    action = np.array([0.6, 0.8])
    return action #Returning dummy values

def get_state(robot: dict, state: str) -> tuple[float, float, float]:
    x = robot['joint_state'][state][0]
    y = robot['joint_state'][state][0]
    theta = robot['joint_state'][state][0]
    return x, y, theta
