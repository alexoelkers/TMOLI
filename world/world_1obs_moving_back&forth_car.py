import pybullet as p
import time
import pybullet_data
import math

# Connect to PyBullet (GUI for visual simulation)
p.connect(p.GUI)

# Set up the physics simulation environment
p.setGravity(0, 0, -9.81)  # Earth gravity
p.setTimeStep(1./240.)  # Set the timestep for simulation

# Load the plane (ground)
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # Access PyBullet's built-in data
plane_id = p.loadURDF("plane.urdf")  # Load the plane

length, width, height = 1, 1, 1
position = 0,0,0

# Create a box using a collision shape and visual shape
obstacle_id = p.createMultiBody(baseMass=0,  # Static obstacle, so no mass
                                baseCollisionShapeIndex=p.createCollisionShape(p.GEOM_BOX, halfExtents=[length/2, width/2, height/2]),
                                baseVisualShapeIndex=p.createVisualShape(p.GEOM_BOX, halfExtents=[length/2, width/2, height/2], rgbaColor=[0.8, 0.3, 0.3, 1]),
                                basePosition=position)

for step in range(1000):  # Simulate for 1000 frames
    p.stepSimulation()  # Move the simulation one step

    # Kinematic movement: Set new position for the obstacle
    # Move the obstacle in the x-direction over time (e.g., increasing x-coordinate)
    new_position = [4*math.sin(step * 0.01), 0, 0.5]  # Move along the x-axis
    p.resetBasePositionAndOrientation(obstacle_id, new_position, [0, 0, 0, 1])  # Set position and orientation

    time.sleep(1./240.)  # Sleep to control the simulation speed
    
# Disconnect the simulation when done
p.disconnect()
