# If you on a Windows machine with any Python version 
# or an M1 mac with any Python version
# or an Intel Mac with Python > 3.7
# the multi-threaded version does not work
# so instead, you can use this version. 

import unittest
import population
import simulation 
import genome 
import creature as crlib
import numpy as np
import pybullet as p
from rich.traceback import install

# rich output
install(show_locals=True)

# Hyper parameters
environment_config = {
    "CREATURE_SPAWN_POSITION": [7, 0, 3],
    "CREATURE_SPAWN_ORIENTATION_QUATERNION": [0, 0, 0, 1],
    "MOUNTAIN_SPAWN_POSITION": [0, 0, -1],
    "MOUNTAIN_SPAWN_ORIENTATION_QUATERNION": p.getQuaternionFromEuler((0, 0, 0)),
    "ARENA_SIZE": 20  # length of edge
}

class TestGA(unittest.TestCase):
    def testBasicGA(self):
        pop = population.Population(pop_size=10, 
                                    gene_count=3)
        #sim = simulation.ThreadedSim(pool_size=1)
        sim = simulation.Simulation()

        for iteration in range(1000):
            # this is a non-threaded version 
            # where we just call run_creature instead
            # of eval_population
            for creature in pop.creatures:
                sim.run_creature(creature, env_id=1, environment_config=environment_config, iterations=2400)

            #sim.eval_population(pop, 2400)
            fits = [cr.get_fitness() for cr in pop.creatures]
            links = [len(cr.get_expanded_links()) for cr in pop.creatures]

            print(iteration, "fittest:", np.round(np.max(fits), 3), 
                  "mean:", np.round(np.mean(fits), 3), "mean links", np.round(np.mean(links)), "max links", np.round(np.max(links)))       
            fit_map = population.Population.get_fitness_map(fits)

            new_creatures = []

            for i in range(len(pop.creatures)):
                p1_ind = population.Population.select_parent(fit_map)
                p2_ind = population.Population.select_parent(fit_map)
                p1 = pop.creatures[p1_ind]
                p2 = pop.creatures[p2_ind]
                # now we have the parents!
                dna = genome.Genome.crossover(p1.dna, p2.dna)
                dna = genome.Genome.point_mutate(dna, rate=0.1, amount=0.25)
                dna = genome.Genome.shrink_mutate(dna, rate=0.25)
                dna = genome.Genome.grow_mutate(dna, rate=0.1)
                offspring = crlib.Creature(1)  # create a temporary offspring with 1 gene
                offspring.update_dna(dna)  # and replace its gene with the mutated one
                new_creatures.append(offspring)

            # elitism
            max_fit = np.max(fits)
            for creature in pop.creatures:
                if creature.get_distance_travelled() == max_fit:
                    # Find the creature of maximum fit
                    new_cr = crlib.Creature(1)
                    new_cr.update_dna(creature.dna)
                    # Place it in new generation
                    new_creatures[0] = new_cr
                    filename = "cw/elite_csvs/elite_"+str(iteration)+".csv"
                    genome.Genome.to_csv(creature.dna, filename)
                    break
            
            pop.creatures = new_creatures
                            
        self.assertNotEqual(fits[0], 0)

unittest.main()
