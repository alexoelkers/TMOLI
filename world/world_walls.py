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

# Create walls to limit the world
wall_thickness = 0.1  # Thickness of the walls
wall_height = 2  # Height of the walls

# Define world boundaries (in x and y directions)
boundary_limits = {
    "x_min": -5,
    "x_max": 5,
    "y_min": -5,
    "y_max": 5
}

# Create walls along the boundaries
walls = []
walls.append(p.createMultiBody(baseMass=0,
                                baseCollisionShapeIndex=p.createCollisionShape(p.GEOM_BOX, halfExtents=[(boundary_limits['x_max'] - boundary_limits['x_min']) / 2, wall_thickness / 2, wall_height / 2]),
                                baseVisualShapeIndex=p.createVisualShape(p.GEOM_BOX, halfExtents=[(boundary_limits['x_max'] - boundary_limits['x_min']) / 2, wall_thickness / 2, wall_height / 2], rgbaColor=[0.3, 0.3, 0.8, 1]),
                                basePosition=[0, boundary_limits['y_min'] - wall_thickness / 2, wall_height / 2]))  # Bottom wall

walls.append(p.createMultiBody(baseMass=0,
                                baseCollisionShapeIndex=p.createCollisionShape(p.GEOM_BOX, halfExtents=[(boundary_limits['x_max'] - boundary_limits['x_min']) / 2, wall_thickness / 2, wall_height / 2]),
                                baseVisualShapeIndex=p.createVisualShape(p.GEOM_BOX, halfExtents=[(boundary_limits['x_max'] - boundary_limits['x_min']) / 2, wall_thickness / 2, wall_height / 2], rgbaColor=[0.3, 0.3, 0.8, 1]),
                                basePosition=[0, boundary_limits['y_max'] + wall_thickness / 2, wall_height / 2]))  # Top wall

walls.append(p.createMultiBody(baseMass=0,
                                baseCollisionShapeIndex=p.createCollisionShape(p.GEOM_BOX, halfExtents=[wall_thickness / 2, (boundary_limits['y_max'] - boundary_limits['y_min']) / 2, wall_height / 2]),
                                baseVisualShapeIndex=p.createVisualShape(p.GEOM_BOX, halfExtents=[wall_thickness / 2, (boundary_limits['y_max'] - boundary_limits['y_min']) / 2, wall_height / 2], rgbaColor=[0.3, 0.3, 0.8, 1]),
                                basePosition=[boundary_limits['x_min'] - wall_thickness / 2, 0, wall_height / 2]))  # Left wall

walls.append(p.createMultiBody(baseMass=0,
                                baseCollisionShapeIndex=p.createCollisionShape(p.GEOM_BOX, halfExtents=[wall_thickness / 2, (boundary_limits['y_max'] - boundary_limits['y_min']) / 2, wall_height / 2]),
                                baseVisualShapeIndex=p.createVisualShape(p.GEOM_BOX, halfExtents=[wall_thickness / 2, (boundary_limits['y_max'] - boundary_limits['y_min']) / 2, wall_height / 2], rgbaColor=[0.3, 0.3, 0.8, 1]),
                                basePosition=[boundary_limits['x_max'] + wall_thickness / 2, 0, wall_height / 2]))  # Right wall


# Simulate for some time to visualize the obstacles and walls
for _ in range(1000):  # Simulate for 1000 frames
    p.stepSimulation()  # Move the simulation one step
    time.sleep(1./240.)  # Sleep to control the simulation speed

# Disconnect the simulation when done
p.disconnect()