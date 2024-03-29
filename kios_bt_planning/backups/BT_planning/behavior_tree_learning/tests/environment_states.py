"""
A simple simulation environment for test purposes only.
All environments must contain a get_fitness(individual) function
that returns a fitness value and a plot_individual() function that
returns nothing but saves a graphical representation of the individual
"""

#Imports that define the environment
from behavior_tree_learning.py_trees_interface import PyTree
import behavior_tree_learning.tests.behaviors_states as behaviors
import behavior_tree_learning.state_machine as sm
import behavior_tree_learning.fitness_function as fitness_function

def get_fitness(string):
    """ Run the simulation and return the fitness """
    state_machine = sm.StateMachine()
    behavior_tree = PyTree(string[:], behaviors=behaviors, world_interface=state_machine)

    # run the Behavior Tree
    behavior_tree.run_bt()

    return fitness_function.compute_fitness(state_machine, behavior_tree)

def plot_individual(path, plot_name, individual):
    """ Saves a graphical representation of the individual """
    pytree = PyTree(individual[:], behaviors=behaviors)
    pytree.save_fig(path, name=plot_name)
