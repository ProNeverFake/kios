"""Unit test for distributed_island.py module."""

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

import time

import bt_learning.gp.distributed_island as gp_dim

from .test_gp_instance import behavior_lists
from .test_gp_parallel import TestEnvironment


def test_coordinate_conversion():
    """Test two tool functions for conversion between coordinates and indexes."""
    # test grid:  height=3, width=4
    # 0, 1, 2, 3
    # 4, 5, 6, 7
    # 8, 9, 10, 11
    assert gp_dim.coordinate2idx(height=1, width=2, grid_width=4) == 6
    assert gp_dim.coordinate2idx(height=0, width=0, grid_width=4) == 0
    assert gp_dim.coordinate2idx(height=0, width=3, grid_width=4) == 3

    assert gp_dim.idx2coordinate(index=9, grid_width=4) == (2, 1)
    assert gp_dim.idx2coordinate(index=11, grid_width=4) == (2, 3)
    assert gp_dim.idx2coordinate(index=0, grid_width=4) == (0, 0)


def test_neighbor_loop():
    """Test the loop topology among the islands."""
    dim_par = gp_dim.DimParameters()
    dim_par.behavior_lists = behavior_lists
    dim_par.ind_start_length = 3
    dim_par.n_population = 100
    dim_par.f_crossover = 0.5
    dim_par.n_offspring_crossover = 10
    dim_par.f_mutation = 0.25
    dim_par.n_offspring_mutation = 10
    dim_par.f_elites = 0.1
    dim_par.f_parents = 1
    dim_par.plot = False
    dim_par.n_generations = 20
    dim_par.verbose = True
    dim_par.fig_best = False

    dim_par.topology = gp_dim.IslandTopology.LOOP
    dim_par.num_island = 3
    dim_par.num_migrants = 2
    dim_par.migration_frequency = 1

    t_environment = TestEnvironment()
    dim = gp_dim.DistributedIslandModel(t_environment, dim_par)
    target_neighbor_list = [[1], [2], [0]]
    assert dim.neighbors == target_neighbor_list


def test_neighbor_grid():
    """Test the grid topology among the islands."""
    dim_par = gp_dim.DimParameters()
    dim_par.behavior_lists = behavior_lists
    dim_par.ind_start_length = 3
    dim_par.n_population = 100
    dim_par.f_crossover = 0.5
    dim_par.n_offspring_crossover = 10
    dim_par.f_mutation = 0.25
    dim_par.n_offspring_mutation = 10
    dim_par.f_elites = 0.1
    dim_par.f_parents = 1
    dim_par.plot = False
    dim_par.n_generations = 20
    dim_par.verbose = True
    dim_par.fig_best = False
    # first grid to test
    # 0, 1
    # 2, 3
    dim_par.topology = gp_dim.IslandTopology.GRID
    dim_par.grid_height_size = 2
    dim_par.grid_width_size = 2
    dim_par.num_island = dim_par.grid_width_size * dim_par.grid_height_size
    dim_par.num_migrants = 2
    dim_par.migration_frequency = 1

    t_environment = TestEnvironment()
    dim = gp_dim.DistributedIslandModel(t_environment, dim_par)
    target_neighbor_list = [[2, 1], [0, 3], [0, 3], [1, 2]]
    generated_neighbor_list = dim.neighbors
    for i in range(dim_par.num_island):
        assert set(target_neighbor_list[i]) == set(generated_neighbor_list[i])

    # second grid to test
    # 0, 1, 2
    # 3, 4, 5
    # 6, 7, 8
    dim_par.topology = gp_dim.IslandTopology.GRID
    dim_par.grid_height_size = 3
    dim_par.grid_width_size = 3
    dim_par.num_island = dim_par.grid_width_size * dim_par.grid_height_size
    dim_par.num_migrants = 2
    dim_par.migration_frequency = 1

    t_environment = TestEnvironment()
    dim = gp_dim.DistributedIslandModel(t_environment, dim_par)
    target_neighbor_list = [[1, 3, 6, 2], [0, 2, 4, 7], [1, 5, 0, 8],
                            [0, 4, 6, 5], [1, 3, 5, 7], [2, 4, 8, 3],
                            [3, 7, 8, 0], [4, 6, 8, 1], [5, 7, 6, 2]]
    generated_neighbor_list = dim.neighbors
    for i in range(dim_par.num_island):
        assert set(target_neighbor_list[i]) == set(generated_neighbor_list[i])


def test_run_dim_grid():
    """
    Test running the whole distributed island model.

    with grid topology
    grid size: 2x2, and 3x3
    """
    dim_par = gp_dim.DimParameters()
    dim_par.behavior_lists = behavior_lists
    dim_par.ind_start_length = 3
    dim_par.n_population = 400
    dim_par.f_crossover = 0.5
    dim_par.n_offspring_crossover = 10
    dim_par.f_mutation = 0.25
    dim_par.n_offspring_mutation = 10
    dim_par.f_elites = 0.1
    dim_par.f_parents = 1
    dim_par.plot = False
    dim_par.n_generations = 10
    dim_par.verbose = True
    dim_par.fig_best = False
    # grid to test
    # 0, 1
    # 2, 3
    # mp with island_proc: 55s
    # seq:186s   16-18s per update
    dim_par.topology = gp_dim.IslandTopology.GRID
    dim_par.grid_height_size = 2
    dim_par.grid_width_size = 2
    dim_par.num_island = dim_par.grid_width_size * dim_par.grid_height_size
    dim_par.num_migrants = 2
    dim_par.migration_frequency = 1

    t_environment = TestEnvironment()

    t1 = time.time()
    dim = gp_dim.DistributedIslandModel(t_environment, dim_par)
    dim.run()
    individual, fitness = dim.get_best_individual()
    print(individual, fitness)
    assert fitness == 3.4
    t2 = time.time()
    print('2x2 grid running time:', t2-t1)
    # grid to test
    # 0, 1, 2
    # 3, 4, 5
    # 6, 7, 8
    # mp with island_proc: 107s
    dim_par.topology = gp_dim.IslandTopology.GRID
    dim_par.grid_height_size = 3
    dim_par.grid_width_size = 3
    dim_par.num_island = dim_par.grid_width_size * dim_par.grid_height_size
    t3 = time.time()
    dim = gp_dim.DistributedIslandModel(t_environment, dim_par)
    dim.run()
    individual, fitness = dim.get_best_individual()
    print(individual, fitness)
    assert fitness == 3.4
    t4 = time.time()
    print('3x3 grid running time:', t4-t3)


def test_run_dim_loop():
    """
    Test running the whole distributed island model.

    with loop topology
    loop len: 4, and 9
    """
    dim_par = gp_dim.DimParameters()
    dim_par.behavior_lists = behavior_lists
    dim_par.ind_start_length = 3
    dim_par.n_population = 400
    dim_par.f_crossover = 0.5
    dim_par.n_offspring_crossover = 10
    dim_par.f_mutation = 0.25
    dim_par.n_offspring_mutation = 10
    dim_par.f_elites = 0.1
    dim_par.f_parents = 1
    dim_par.plot = False
    dim_par.n_generations = 10
    dim_par.verbose = True
    dim_par.fig_best = False
    # len(loop) = 4
    # mp with island_proc: 54s
    dim_par.topology = gp_dim.IslandTopology.LOOP
    dim_par.num_island = 4
    dim_par.num_migrants = 2
    dim_par.migration_frequency = 1

    t_environment = TestEnvironment()

    t1 = time.time()
    dim = gp_dim.DistributedIslandModel(t_environment, dim_par)
    dim.run()
    individual, fitness = dim.get_best_individual()
    print(individual, fitness)
    assert fitness == 3.4
    t2 = time.time()
    print('loop with 4 islands running time:', t2-t1)
    # len(loop) = 9
    # mp with island_proc: 104s
    dim_par.num_island = 9

    t3 = time.time()
    dim = gp_dim.DistributedIslandModel(t_environment, dim_par)
    dim.run()
    individual, fitness = dim.get_best_individual()
    print(individual, fitness)
    assert fitness == 3.4
    t4 = time.time()
    print('loop with 9 islands running time:', t4-t3)
