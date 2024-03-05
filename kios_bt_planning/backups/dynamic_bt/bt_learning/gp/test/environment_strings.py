"""
A simple simulation environment for test purposes only.

All environments must contain a get_fitness(individual) function
that returns a fitness value and a plot_individual() function that
returns nothing but saves a graphical representation of the individual
"""

# Copyright (c) 2022, ABB
# All rights reserved.
#
# Redistribution and use in source and binary forms, with
# or without modification, are permitted provided that
# the following conditions are met:
#
#   * Redistributions of source code must retain the
#     above copyright notice, this list of conditions
#     and the following disclaimer.
#   * Redistributions in binary form must reproduce the
#     above copyright notice, this list of conditions
#     and the following disclaimer in the documentation
#     and/or other materials provided with the
#     distribution.
#   * Neither the name of ABB nor the names of its
#     contributors may be used to endorse or promote
#     products derived from this software without
#     specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
# THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import random


def get_fitness(individual, _seed=None):
    """
    Return fitness.

    Currently values shortest possible string that contains certain nodes in sequence
    """
    string_individual = str(individual)
    fitness = 0
    stop = False
    index = 0
    index, fitness, stop = check_for_gene(string_individual, 'b?', index, fitness, stop)
    index, fitness, stop = check_for_gene(string_individual, 'c?', index, fitness, stop)
    index, fitness, stop = check_for_gene(string_individual, 'ad!', index, fitness, stop)
    index, fitness, stop = check_for_gene(string_individual, 'ae!', index, fitness, stop)
    if 'r' in individual:
        fitness += random.random()

    fitness -= len(individual) * 0.1
    return fitness


def plot_individual(_path, _plot_name, individual):
    """Save a graphical representation of the individual."""
    print(individual)


def check_for_gene(individual, gene, index, fitness, stop):
    """Check for a gene in individual, starting at index. Ugly code but works."""
    index_t = -1
    if not stop:
        if gene in individual[index:]:
            index_t = individual[index:].index(gene)
            index += index_t
            fitness += 1
        else:
            stop = True

    return index, fitness, stop
