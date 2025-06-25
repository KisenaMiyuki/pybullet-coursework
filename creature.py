import genome 
from xml.dom.minidom import getDOMImplementation
from enum import Enum
import numpy as np
from rich import inspect

class MotorType(Enum):
    PULSE = 1
    SINE = 2



class Motor:
    def __init__(self, control_waveform, control_amp, control_freq):
        if control_waveform <= 0.5:
            self.motor_type = MotorType.PULSE
        else:
            self.motor_type = MotorType.SINE
        self.amp = control_amp
        self.freq = control_freq
        self.phase = 0
    

    def get_output(self):
        self.phase = (self.phase + self.freq) % (np.pi * 2)
        if self.motor_type == MotorType.PULSE:
            if self.phase < np.pi:
                output = 1
            else:
                output = -1
            
        if self.motor_type == MotorType.SINE:
            output = np.sin(self.phase)
        
        return output 



class Creature:
    time_spent_above_ascent_threshold: int

    def __init__(self, gene_count):
        self.spec = genome.Genome.get_gene_spec()
        self.dna = genome.Genome.get_random_genome(len(self.spec), gene_count)
        self.flat_links = None
        self.exp_links = None
        self.motors = None
        self.start_position = None
        self.last_position = None
        self.minimum_horizontal_distance_to_peak = None
        self.ascent_history = []
        self.ascent_threshold = 2.5
        self.time_spent_above_ascent_threshold = 0


    def get_flat_links(self):
        if self.flat_links is None:
            gdicts = genome.Genome.get_genome_dicts(self.dna, self.spec)
            self.flat_links = genome.Genome.genome_to_links(gdicts)
        return self.flat_links


    def get_expanded_links(self):
        self.get_flat_links()
        if self.exp_links is not None:
            return self.exp_links
        
        exp_links = [self.flat_links[0]]
        genome.Genome.expandLinks(self.flat_links[0], 
                                self.flat_links[0].name, 
                                self.flat_links, 
                                exp_links)
        self.exp_links = exp_links
        return self.exp_links


    def to_xml(self):
        self.get_expanded_links()
        domimpl = getDOMImplementation()
        adom = domimpl.createDocument(None, "start", None)
        robot_tag = adom.createElement("robot")
        for link in self.exp_links:
            robot_tag.appendChild(link.to_link_element(adom))
        first = True
        for link in self.exp_links:
            if first:# skip the root node! 
                first = False
                continue
            robot_tag.appendChild(link.to_joint_element(adom))
        robot_tag.setAttribute("name", "pepe") #  choose a name!
        return '<?xml version="1.0"?>' + robot_tag.toprettyxml()


    def get_motors(self):
        self.get_expanded_links()
        if self.motors is None:
            motors = []
            for i in range(1, len(self.exp_links)):
                l = self.exp_links[i]
                m = Motor(l.control_waveform, l.control_amp,  l.control_freq)
                motors.append(m)
            self.motors = motors 
        return self.motors


    def update_position(self, pos):
        # inspect(self)

        # Start / End Position
        if self.start_position is None:
            self.start_position = pos
        else:
            self.last_position = pos

        # Minimum horizontal distance to peak
        # location of peak is currently hard-coded
        current_horizontal_distance_to_peak = None if self.last_position is None else np.linalg.norm(self.last_position[:2] - np.array([0,0]))
        if self.minimum_horizontal_distance_to_peak is None:
            self.minimum_horizontal_distance_to_peak = current_horizontal_distance_to_peak
        else:
            self.minimum_horizontal_distance_to_peak = current_horizontal_distance_to_peak if current_horizontal_distance_to_peak < self.minimum_horizontal_distance_to_peak else self.minimum_horizontal_distance_to_peak

        # TODO: Find a way to calculate fitness without negative values, and ignore initial drop section

        # Height change
        height_change = pos[2] - self.last_position[2]

        # Ascent history
        if height_change >= 0:
            self.ascent_history.append(height_change)

        # Time spent above the ascent threshold
        # What is ascent threshold?
        if current_ascent > self.ascent_threshold:
            self.time_spent_above_ascent_threshold += 1


    def get_distance_travelled(self):
        if self.start_position is None or self.last_position is None:
            return 0
        p1 = np.asarray(self.start_position)
        p2 = np.asarray(self.last_position)
        dist = np.linalg.norm(p1-p2)
        return dist


    def get_fitness(self, verbose=True):
        """
        "The fitness function"
        """
        # Component 1: 1 / horizontal distance to peak
        #                               how to locate peak? (0,0) hardcoded
        # Component 2: average height over time (penalizes single jump or brief ascent, rewards stable climb)
        # Component 3: time spent above certain altitude
        # Component 4: energy used
        one_over_closest_horizontal_distance_to_peak = 1 / self.minimum_horizontal_distance_to_peak  # greater the better
        average_ascent_over_time = sum(self.ascent_history) / len(self.ascent_history)
        if verbose:
            print(f"┏━━━━━ Fitness function called")
            print(f"┣━ Minimum horizontal distance to peak: {self.minimum_horizontal_distance_to_peak} | one-over~: {one_over_closest_horizontal_distance_to_peak}")
            print(f"┣━ Average ascent amount over time: {average_ascent_over_time}")
            print(f"┣━ Time spent above ascent threshold: {self.time_spent_above_ascent_threshold}")
            print(f"┗━━━━━ Fitness: {one_over_closest_horizontal_distance_to_peak + average_ascent_over_time + self.time_spent_above_ascent_threshold}")
        return one_over_closest_horizontal_distance_to_peak + average_ascent_over_time + self.time_spent_above_ascent_threshold


    def update_dna(self, dna):
        self.dna = dna
        self.flat_links = None
        self.exp_links = None
        self.motors = None
        self.start_position = None
        self.last_position = None