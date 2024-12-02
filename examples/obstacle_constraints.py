import numpy as np
from urdfenvs.urdf_common.urdf_env import UrdfEnv
from mpscenes.obstacles.collision_obstacle import CollisionObstacle
from mpscenes.obstacles.box_obstacle import BoxGeometryConfig

def get_linearized_obstacle_constraints(env: UrdfEnv, position: np.ndarray):
    """
    Create the constraints in the form of n^Tp - n^Tq < 0, where n is the normal pointing "towards" the obstacle.
    If all elements of the return array are negative, the linearized constraint is fulfilled


    Parameters
    ---
    env : UrdfEnv - The urdf environment used to simulate
    position : np.ndarray - The current position of the robot. Must be two-dimensional

    Returns
    ---

    """

    obstacles: dict[str, CollisionObstacle] = env.get_obstacles()

    if len(position) != 2:
        raise ValueError("Incorrect dimension of position, should be 2")

    constraints = []

    for obstacle in obstacles.values():
        #Get the faces of the obstacles
        #Start by ignoring everything not a BoxObstacle
        if obstacle.type() != "box":
            continue #TODO Handle non-boxed constraints

        geometry: BoxGeometryConfig = obstacle.geometry()
        norm, point_on_line = nearest_norm_and_point(geometry, position)

        n_times_p = norm * np.transpose(position)
        n_times_q = norm * np.transpose(point_on_line) #Row vectors normally
        constraints.append((n_times_p - n_times_q)[0]) #Numerical wonk due to numpy TODO prettify

    return constraints

def nearest_norm_and_point(geometry: BoxGeometryConfig, position):

    #Creates a point on the middle of each edge and compares the distance to each edge
    x_min = (geometry.position[0])
    y_min = (geometry.position[0])
    x_mid = (x_min + geometry.length) / 2
    y_mid = (y_min + geometry.height) / 2

    top = np.array([x_mid, y_min+geometry.height])
    bottom = np.array([x_mid, y_min])
    leftmost = np.array([x_min, y_mid])
    rightmost = np.array([x_min + geometry.width, y_mid])

    norm = [np.array([0, -1]), np.array([0, 1]), np.array([1, 0]), np.array([-1, 0])]

    norm_point = [(norm[0], top), (norm[1], bottom), (norm[2], leftmost), (norm[3], rightmost)]

    return min(norm_point, key=lambda x: distance(position, x[1]))

def distance(q1, q2) -> float:
    return np.hypot(q1[0]-q2[0], q1[1]-q2[1])
