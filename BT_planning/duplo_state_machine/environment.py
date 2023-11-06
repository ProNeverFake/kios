"""
A simple simulation environment for testing duplo handling.
All environments must contain a get_fitness(individual) function
that returns a fitness value and a plot_individual() function that
returns nothing but saves a graphical representation of the individual
"""

#Imports that define the environment
from behavior_tree_learning.py_trees_interface import PyTree
import duplo_state_machine.behaviors as behaviors
import duplo_state_machine.state_machine as sm
import duplo_simulation.fitness_function as fitness_function

class Environment():
    """ Class defining the environment in which the individual operates """
    def __init__(self, start_positions, targets, static_tree=None, \
                 verbose=False, sm_pars=None, mode=0, fitness_coeff=None):
        # pylint: disable=too-many-arguments
        self.start_positions = start_positions
        self.targets = targets
        self.static_tree = static_tree
        self.verbose = verbose
        self.sm_pars = sm_pars
        self.mode = mode
        self.fitness_coeff = fitness_coeff
        self.random_events = False

    def set_random_events(self, random_events):
        """ Sets the random events flag """
        self.random_events = random_events

    def get_fitness(self, individual):
        """ Run the simulation and return the fitness """
        state_machine = sm.StateMachine(self.start_positions, self.random_events, self.sm_pars, self.mode)
        pytree = PyTree(individual[:], behaviors=behaviors, world_interface=state_machine, verbose=self.verbose)

        # run the Behavior Tree
        ticks, _ = pytree.run_bt()

        return fitness_function.compute_fitness(state_machine, pytree, ticks, self.targets, self.fitness_coeff)

    def plot_individual(self, path, plot_name, individual):
        """ Saves a graphical representation of the individual """
        if self.static_tree is not None:
            pytree = PyTree(self.add_to_static_tree(individual), behaviors=behaviors)
        else:
            pytree = PyTree(individual[:], behaviors=behaviors)
        pytree.save_fig(path, name=plot_name)

    def add_to_static_tree(self, individual):
        """ Add invididual to the static part of the tree in the front """
        new_individual = self.static_tree[:]
        new_individual[-2:-2] = individual
        return new_individual

class Environment1(Environment):
    """ Test class for only running first target in list  """
    def __init__(self, targets, static_tree, verbose=False):
        super().__init__(targets, static_tree, verbose)
        self.targets = [self.targets[0]] #Only first target

    def get_fitness(self, individual):
        return super().get_fitness(self.add_to_static_tree(individual))

class Environment12(Environment):
    """ Test class for only running first two targets in list  """
    def __init__(self, targets, static_tree, verbose=False):
        super().__init__(targets, static_tree, verbose)
        self.targets = self.targets[:2] #Only first two targets

    def get_fitness(self, individual):
        return super().get_fitness(self.add_to_static_tree(individual))

class Environment123(Environment):
    """ Test class for only running first three targets in list  """
    def __init__(self, targets, static_tree, verbose=False):
        super().__init__(targets, static_tree, verbose)
        self.targets = self.targets[:3] #Only first three targets

    def get_fitness(self, individual):
        return super().get_fitness(self.add_to_static_tree(individual))
