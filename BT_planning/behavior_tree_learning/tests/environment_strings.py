"""
A simple simulation environment for test purposes only.
All environments must contain a get_fitness(individual) function
that returns a fitness value and a plot_individual() function that
returns nothing but saves a graphical representation of the individual
"""

def get_fitness(individual):
    """
    Returns fitness
    Currently values shortest possible string that contains certain nodes in sequence
    """
    fitness = 0
    stop = False
    index = 0
    index, fitness, stop = check_for_gene(individual, 'c0', index, fitness, stop)
    index, fitness, stop = check_for_gene(individual, 'c1', index, fitness, stop)
    index, fitness, stop = check_for_gene(individual, 'a2', index, fitness, stop)
    index, fitness, stop = check_for_gene(individual, 'a3', index, fitness, stop)

    fitness -= len(individual) * 0.1
    return fitness

def plot_individual(_path, _plot_name, individual):
    """ Saves a graphical representation of the individual """
    print(individual)

def check_for_gene(individual, gene, index, fitness, stop):
    """
    Checks for a gene in individual, starting at index. Ugly code but works.
    """
    index_t = -1
    if not stop:
        if gene in individual[index:]:
            index_t = individual[index:].index(gene)
            index += index_t
            fitness += 1
        else:
            stop = True


    return index, fitness, stop
