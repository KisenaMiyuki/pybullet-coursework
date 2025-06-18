import pybullet as p
import pybullet_data
import time
import random
import population
import simulation_v2
import creature
from rich.console import Console
from rich.traceback import install
from rich.panel import Panel
from rich import inspect

# Initialize rich console
console = Console()

# Install rich traceback
install(show_locals=True)

# Hyper parameters
ENVIRONMENT_CONFIG = {
    "CREATURE_SPAWN_POSITION": [7, 0, 3],
    "CREATURE_SPAWN_ORIENTATION_QUATERNION": [0, 0, 0, 1],
    "MOUNTAIN_SPAWN_POSITION": [0, 0, -1],
    "MOUNTAIN_SPAWN_ORIENTATION_QUATERNION": p.getQuaternionFromEuler((0, 0, 0)),
    "ARENA_SIZE": 20  # length of edge
}
POPULATION_SIZE = 10
GENE_COUNT = 3
CREATURE_SIMULATION_ITERATIONS = 2400
EVOLVE_GENERATIONS = 5



# Initialize the first generation of creature population
# pop = population.Population(pop_size=POPULATION_SIZE, gene_count=GENE_COUNT)
# my_creature = creature.Creature(gene_count=3)
# inspect(my_creature)

console.print(Panel("BEFORE CONNECTING TO DIRECT MODE CLIENT"))

# Connect to PyBullet in DIRECT mode
physicsClient = p.connect(p.DIRECT)
inspect(physicsClient)
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
for step in range(60):
    p.stepSimulation()
    pos, orn = p.getBasePositionAndOrientation(cubeId)
    console.print(f"Step {step:03d} | Cube position: {pos}", style="bold green")
    time.sleep(1.0 / 60.0)  # Simulate at ~60Hz

p.disconnect()
