"""
Task dependent cost function
"""

from dataclasses import dataclass

@dataclass
class Coefficients:
    """
    Coefficients for tuning the cost function
    """
    depth: int = 0
    length: int = 1
    task_completion: int = 10


def compute_fitness(state_machine, behavior_tree):
    """ Retrieve values and compute cost """

    coeff = Coefficients()

    depth = behavior_tree.depth
    length = behavior_tree.length

    cost = coeff.length * length + \
           coeff.depth * depth

    for state in state_machine.state:
        if state:
            cost += coeff.task_completion * -1

    fitness = -cost
    return fitness
