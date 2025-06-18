import os

from rich import live

import genome
import sys
import creature
import pybullet as p
import time 
import random
import numpy as np
from cw_envt_mod import make_arena
from rich.traceback import install
from rich.live import Live
from rich.table import Table
from rich.console import Group
from rich.spinner import Spinner

# rich output setup
install(show_locals=True)
def generate_table(elapsed_time, dist_moved, current_pos) -> Table:
    table = Table()
    table.add_column('Elapsed Time', justify='center')
    table.add_column('Distance Moved', justify='center')
    table.add_column('Current Position', justify='center')
    table.add_row(f"{elapsed_time:.2f}", f"{dist_moved:.2f}", f"{current_pos}")
    return table
spinner = Spinner("dots", text="Simulating...")

# Hyper parameters
CREATURE_SPAWN_POSITION = [7, 0, 3]
CREATURE_SPAWN_ORIENTATION_QUATERNION = [0, 0, 0, 1]
MOUNTAIN_SPAWN_POSITION = [0, 0, -1]
MOUNTAIN_SPAWN_ORIENTATION_QUATERNION = p.getQuaternionFromEuler((0, 0, 0))
SIMULATION_TIME_SECONDS = 10
ARENA_SIZE = 20  # length of edge


## ... usual starter code to create a sim and floor
def main(csv_file, environment_type):
    assert os.path.exists(csv_file), "Tried to load " + csv_file + " but it does not exists"

    # Prepare engine
    p.disconnect()
    p.connect(p.GUI)
    p.setPhysicsEngineParameter(enableFileCaching=0)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)

    # Creating the scene
    if environment_type == "0":
        # demo plane scene
        plane_shape = p.createCollisionShape(p.GEOM_PLANE)
        floor = p.createMultiBody(0, plane_shape)
    elif environment_type == "1":
        # mountain scene
        make_arena(arena_size=ARENA_SIZE)
        p.setAdditionalSearchPath('shapes/')
        mountain = p.loadURDF("gaussian_pyramid.urdf", MOUNTAIN_SPAWN_POSITION, MOUNTAIN_SPAWN_ORIENTATION_QUATERNION, useFixedBase=1)

    p.setGravity(0, 0, -10)
#   p.setRealTimeSimulation(1)


    # generate a random creature
    cr = creature.Creature(gene_count=5)
    # dna = genome.Genome.from_csv(csv_file)
    # cr.update_dna(dna)

    # save it to XML
    with open('test.urdf', 'w') as f:
        f.write(cr.to_xml())
    # load it into the sim
    rob1 = p.loadURDF('test.urdf')
    # air drop it
    p.resetBasePositionAndOrientation(rob1, CREATURE_SPAWN_POSITION, CREATURE_SPAWN_ORIENTATION_QUATERNION)
    start_pos, orn = p.getBasePositionAndOrientation(rob1)

    # iterate 
    elapsed_time = 0
    wait_time = 1.0/240 # seconds
    step = 0
    dist_moved = 0
    current_pos = CREATURE_SPAWN_POSITION
    with Live(Group(generate_table(elapsed_time, dist_moved, current_pos), spinner), refresh_per_second=4) as live:
        while True:
            p.stepSimulation()
            step += 1
            if step % 24 == 0:
                motors = cr.get_motors()
                assert len(motors) == p.getNumJoints(rob1), "Something went wrong"
                for jid in range(p.getNumJoints(rob1)):
                    mode = p.VELOCITY_CONTROL
                    vel = motors[jid].get_output()
                    p.setJointMotorControl2(rob1,
                                jid,
                                controlMode=mode,
                                targetVelocity=vel)
                new_pos, orn = p.getBasePositionAndOrientation(rob1)
                #print(new_pos)
                current_pos = new_pos
                dist_moved = np.linalg.norm(np.asarray(start_pos) - np.asarray(new_pos))
                # print(dist_moved)
            time.sleep(wait_time)
            elapsed_time += wait_time
            live.update(generate_table(elapsed_time, dist_moved, current_pos))
            if elapsed_time > SIMULATION_TIME_SECONDS:
                break

    print("TOTAL DISTANCE MOVED:", dist_moved)

    p.disconnect()



if __name__ == "__main__":
    assert len(sys.argv) == 3, "Usage: python playback_test.py csv_filename environment_type(0-plane, 1-mountain)"
    main(sys.argv[1], sys.argv[2])

