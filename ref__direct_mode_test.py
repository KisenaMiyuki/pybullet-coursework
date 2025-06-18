import pybullet as p
import pybullet_data
import time
import random
from rich.console import Console

# Initialize rich console
console = Console()

# Connect to PyBullet in DIRECT mode
physicsClient = p.connect(p.DIRECT)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)

# Load a plane as the floor
planeId = p.loadURDF("plane.urdf")

# Define cube start position and random orientation
start_pos = [0, 0, 1]
random_euler = [random.uniform(0, 3.14) for _ in range(3)]
start_orientation = p.getQuaternionFromEuler(random_euler)

# Load a cube URDF (using the built-in 'cube.urdf' if available)
cubeId = p.loadURDF("cube.urdf", start_pos, start_orientation, globalScaling=0.2)

# Simulate and print cube position in real time
for step in range(240):
    p.stepSimulation()
    pos, orn = p.getBasePositionAndOrientation(cubeId)
    console.print(f"Step {step:03d} | Cube position: {pos}", style="bold green")
    time.sleep(1.0 / 60.0)  # Simulate at ~60Hz

p.disconnect()
