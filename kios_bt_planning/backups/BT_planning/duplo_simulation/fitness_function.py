"""
Task dependent cost function
"""

from dataclasses import dataclass

@dataclass
class Coefficients:
    """
    Coefficients for tuning the cost function
    """
    task_completion: float = 1000.0
    pos_acc: float = 0.0004
    depth: float = 0.0
    length: float = 0.1
    ticks: float = 0.0
    failed: float = 50.0
    timeout: float = 10.0
    hand_not_empty: float = 0.0

def compute_fitness(world_interface, behavior_tree, ticks, targets, coeff=None, verbose=False):
    # pylint: disable=too-many-arguments
    """ Retrieve values and compute cost """

    if coeff is None:
        coeff = Coefficients()

    depth = behavior_tree.depth
    length = behavior_tree.length

    cost = coeff.length * length + \
           coeff.depth * depth + \
           coeff.ticks * ticks
    if verbose:
        print("Cost from length:", cost)
    for i in range(len(targets)):
        cost += coeff.task_completion * max(0, world_interface.distance(i, targets[i]) - coeff.pos_acc)
        if verbose:
            print("Cost:", cost)
            print(i, ": ", world_interface.get_brick_position(i))

    if behavior_tree.failed:
        cost += coeff.failed
        if verbose:
            print("Failed: ", cost)
    if behavior_tree.timeout:
        cost += coeff.timeout
        if verbose:
            print("Timed out: ", cost)
    if world_interface.get_picked() is not None:
        cost += coeff.hand_not_empty
        if verbose:
            print("Hand not empty: ", cost)
    fitness = -cost
    return fitness
