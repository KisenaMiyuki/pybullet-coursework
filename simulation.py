import pybullet as p
from multiprocessing import Pool
from cw_envt_mod import make_arena
from rich import print
import time

class Simulation:
    def __init__(self, sim_id=0):
        self.physicsClientId = None
        self.sim_id = sim_id

    def connect(self):
        if self.physicsClientId is None:
            p.disconnect()
            self.physicsClientId = p.connect(p.DIRECT)

    def disconnect(self):
        """
        Properly disconnect from the PyBullet physics server
        """
        if hasattr(self, 'physicsClientId') and self.physicsClientId is not None:
            try:
                p.disconnect(physicsClientId=self.physicsClientId)
                self.physicsClientId = None
            except Exception as e:
                # Handle case where connection was already closed
                print(f"Warning: Could not disconnect physics client {self.physicsClientId}: {e}")
                self.physicsClientId = None
        else:
            # Disconnect any existing connection without specifying client ID
            try:
                p.disconnect()
            except:
                pass  # Ignore if no connection exists

    def run_creature(self, cr, env_id=0, environment_config=None, iterations=2400):
        # Prepare physics engine for simulation
        if environment_config is None:
            environment_config = {}
        physics_client_id = self.physicsClientId
        p.resetSimulation(physicsClientId=physics_client_id)
        p.setPhysicsEngineParameter(enableFileCaching=0, physicsClientId=physics_client_id)

        # 3D Sandbox Environment setup
        if env_id == 0:
            p.setGravity(0, 0, -10, physicsClientId=physics_client_id)
            plane_shape = p.createCollisionShape(p.GEOM_PLANE, physicsClientId=physics_client_id)
            floor = p.createMultiBody(plane_shape, plane_shape, physicsClientId=physics_client_id)
        elif env_id == 1:
            p.setGravity(0, 0, -10, physicsClientId=physics_client_id)
            make_arena(arena_size=environment_config["ARENA_SIZE"])
            p.setAdditionalSearchPath('shapes/')
            mountain = p.loadURDF("gaussian_pyramid.urdf",
                                  environment_config["MOUNTAIN_SPAWN_POSITION"],
                                  environment_config["MOUNTAIN_SPAWN_ORIENTATION_QUATERNION"],
                                  useFixedBase=1)
            # print(f"{mountain}, {p.getNumJoints(mountain)}, {p.getContactPoints(mountain)}")

        # Generate the urdf definition file for the creature (build the creature) to import into pybullet
        xml_file = 'temp' + str(self.sim_id) + '.urdf'
        xml_str = cr.to_xml()
        with open(xml_file, 'w') as f:
            f.write(xml_str)

        # Load the creature into pybullet through the generated urdf file
        creature_id = p.loadURDF(xml_file, physicsClientId=physics_client_id)

        # Adjust the position and orientation of it
        p.resetBasePositionAndOrientation(creature_id,
                                          environment_config["CREATURE_SPAWN_POSITION"],
                                          environment_config["CREATURE_SPAWN_ORIENTATION_QUATERNION"],
                                          physicsClientId=physics_client_id)


        # Let the simulation begin
        for step in range(iterations):
            p.stepSimulation(physicsClientId=physics_client_id)
            if step % 24 == 0:
                self.update_motors(cid=creature_id, cr=cr)

            pos, orn = p.getBasePositionAndOrientation(creature_id, physicsClientId=physics_client_id)
            cr.update_position(pos)
            # time.sleep(1.0/240)
            #print(pos[2])
            #print(cr.get_distance_travelled())
        
    
    def update_motors(self, cid, cr):
        """
        cid is the id in the physics engine
        cr is a creature object
        """
        for jid in range(p.getNumJoints(cid,
                                        physicsClientId=self.physicsClientId)):
            m = cr.get_motors()[jid]

            p.setJointMotorControl2(cid, jid, 
                    controlMode=p.VELOCITY_CONTROL, 
                    targetVelocity=m.get_output(), 
                    force = 5, 
                    physicsClientId=self.physicsClientId)
        

    # You can add this to the Simulation class:
    def eval_population(self, pop, iterations):
        for cr in pop.creatures:
            self.run_creature(cr, 2400) 


class ThreadedSim():
    def __init__(self, pool_size):
        self.sims = [Simulation(i) for i in range(pool_size)]

    @staticmethod
    def static_run_creature(sim, cr, iterations):
        sim.run_creature(cr, iterations)
        return cr
    
    def eval_population(self, pop, iterations):
        """
        pop is a Population object
        iterations is frames in pybullet to run for at 240fps
        """
        pool_args = [] 
        start_ind = 0
        pool_size = len(self.sims)
        while start_ind < len(pop.creatures):
            this_pool_args = []
            for i in range(start_ind, start_ind + pool_size):
                if i == len(pop.creatures):# the end
                    break
                # work out the sim ind
                sim_ind = i % len(self.sims)
                this_pool_args.append([
                            self.sims[sim_ind], 
                            pop.creatures[i], 
                            iterations]   
                )
            pool_args.append(this_pool_args)
            start_ind = start_ind + pool_size

        new_creatures = []
        for pool_argset in pool_args:
            with Pool(pool_size) as p:
                # it works on a copy of the creatures, so receive them
                creatures = p.starmap(ThreadedSim.static_run_creature, pool_argset)
                # and now put those creatures back into the main 
                # self.creatures array
                new_creatures.extend(creatures)
        pop.creatures = new_creatures
