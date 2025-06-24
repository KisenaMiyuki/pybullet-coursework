# If you on a Windows machine with any Python version 
# or an M1 mac with any Python version
# or an Intel Mac with Python > 3.7
# the multi-threaded version does not work
# so instead, you can use this version. 

import population
import simulation_v2
import genome 
import creature as crlib
import numpy as np
import pybullet as p
import sys
from rich import print
from rich.traceback import install
from rich.progress import track
from rich.panel import Panel
from rich import inspect

# rich output
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
EVOLVE_GENERATIONS = 10



def main(sim_id, sim_mode):
    pop = population.Population(pop_size=POPULATION_SIZE, gene_count=GENE_COUNT)
    #sim = simulation.ThreadedSim(pool_size=1)
    sim = simulation_v2.Simulation(sim_id=int(sim_id), sim_mode=sim_mode)
    sim.connect()

    max_fit_on_previous_generation = 0

    for generation in track(range(EVOLVE_GENERATIONS)):
        # this is a non-threaded version
        # where we just call run_creature instead
        # of eval_population
        for creature in pop.creatures:
            sim.run_creature(creature, env_id=1, environment_config=ENVIRONMENT_CONFIG, iterations=CREATURE_SIMULATION_ITERATIONS)
            print(f"{creature}: {creature.get_fitness()}")

        #sim.eval_population(pop, 2400)
        fits = [cr.get_fitness(verbose=False) for cr in pop.creatures]
        links = [len(cr.get_expanded_links()) for cr in pop.creatures]

        print(Panel(f"[green]Generation {generation}[/green]\n"
                    f"Fittest: [blue]{np.round(np.max(fits), 3)}[/blue], "
                    f"Mean: [blue]{np.round(np.mean(fits), 3)}[/blue], "
                    f"Mean Links: [purple]{np.round(np.mean(links), 3)}[/purple], "
                    f"Max Links: [purple]{np.round(np.max(links))}[/purple]"))
        fit_map = population.Population.get_fitness_map(fits)

        new_creatures = []

        for i in range(len(pop.creatures)):
            # inspect(pop.creatures[i])
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
        print(f"Max fit on Generation {generation}: [blue]{max_fit}[/blue]")
        for creature in pop.creatures:
            if creature.get_fitness(verbose=False) == max_fit:
                # Find the creature of maximum fit
                new_cr = crlib.Creature(1)
                new_cr.update_dna(creature.dna)
                # Place it in new generation
                new_creatures[0] = new_cr
                # only save csv when it's a new high
                if max_fit > max_fit_on_previous_generation:
                    filename = f"elite_csvs/sim{sim.sim_id}/elite_gen{str(generation)}_{round(max_fit, 2)}.csv"
                    genome.Genome.to_csv(creature.dna, filename)
                    max_fit_on_previous_generation = max_fit
                break

        print("Swapping population with new creatures ...")
        pop.creatures = new_creatures

    sim.disconnect()


if __name__ == "__main__":
    assert len(sys.argv) == 3, "Usage: python run_ga_no_threads_cw.py sim_id sim_mode"
    main(sys.argv[1], sys.argv[2])