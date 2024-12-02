import pybullet as p
import time
import pybullet_data

# Connect to PyBullet (GUI for visual simulation)
p.connect(p.GUI)

# Set up the physics simulation environment
p.setGravity(0, 0, -9.81)  # Earth gravity
p.setTimeStep(1./240.)  # Set the timestep for simulation

# Load the plane (ground)
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # Access PyBullet's built-in data
plane_id = p.loadURDF("plane.urdf")  # Load the plane

# Define the dimensions of rectangular obstacles (length, width, height)
obstacle_dimensions = [(1, 1, 1), (2, 0.5, 0.5), (1.5, 1.5, 0.2)]  # Different sizes
positions = [(0, 2, 0.5), (2, -2, 0.25), (-2, 0, 0.75)]  # Different positions for obstacles

# Load rectangular obstacles as boxes (collision shape: box)
obstacle_ids = []
for i, dim in enumerate(obstacle_dimensions):
    length, width, height = dim
    position = positions[i]
    
    # Create a box using a collision shape and visual shape
    obstacle_id = p.createMultiBody(baseMass=0,  # Static obstacle, so no mass
                                    baseCollisionShapeIndex=p.createCollisionShape(p.GEOM_BOX, halfExtents=[length/2, width/2, height/2]),
                                    baseVisualShapeIndex=p.createVisualShape(p.GEOM_BOX, halfExtents=[length/2, width/2, height/2], rgbaColor=[0.8, 0.3, 0.3, 1]),
                                    basePosition=position)
    obstacle_ids.append(obstacle_id)

# Simulate for some time to visualize the obstacles
for _ in range(1000):  # Simulate for 1000 frames
    p.stepSimulation()  # Move the simulation one step
    time.sleep(1./240.)  # Sleep to control the simulation speed

# Disconnect the simulation when done
p.disconnect()
