"""Implement the Distributed Island Model for Genetic Programming."""

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

from dataclasses import dataclass
from enum import auto, Enum
import multiprocessing as mp
from multiprocessing.connection import Connection
from typing import Any, Tuple

from bt_learning.gp.genetic_programming import GpParameters
from bt_learning.gp.gp_instance import GPInstance


class IslandTopology(Enum):
    """Enum class for island topology."""

    LOOP = auto()
    GRID = auto()


class IslandInfoType(Enum):
    """Enum class for supported calls between DIM and islands."""

    STEP_GP = auto()
    GET_MIGRANT = auto()
    MERGE_MIGRANT = auto()
    RECV_MIGRANT = auto()
    GET_BEST = auto()
    LOG_FINAL_INFO = auto()
    OK = auto()
    UNKNOWN = auto()


@dataclass
class DimParameters(GpParameters):
    """Parameters for distributed island model."""

    num_island: int = 3
    migration_frequency: int = 1
    num_migrants: int = 1
    topology: IslandTopology = IslandTopology.LOOP
    grid_width_size: int = -1
    grid_height_size: int = -1
    batch_size = 100


@dataclass
class IslandInfo:
    """Information between DIM process and island processes."""

    type: IslandInfoType = IslandInfoType.UNKNOWN
    args: Any = None


def island_process(
    pipe: Connection,
    name: str,
    environment: Any,
    gp_par: GpParameters,
    num_core: int,
    batch_size: int
) -> None:
    """Create a process for the DIM to communicate, with Pipe and IslandInfo."""
    gp_instance = GPInstance(
        name, environment=environment, gp_par=gp_par, num_core=num_core, batch_size=batch_size)
    info_ok = IslandInfo(IslandInfoType.OK)
    while True:
        info = pipe.recv()
        if info.type == IslandInfoType.STEP_GP:
            gp_instance.step_gp()
            pipe.send(info_ok)
        elif info.type == IslandInfoType.GET_MIGRANT:
            info.args = gp_instance.get_migrants(gp_par.num_migrants)
            pipe.send(info)
        elif info.type == IslandInfoType.RECV_MIGRANT:
            gp_instance.receive_migrants(info.args[0], info.args[1])
            pipe.send(info_ok)
        elif info.type == IslandInfoType.MERGE_MIGRANT:
            gp_instance.merge_migrants()
            pipe.send(info_ok)
        elif info.type == IslandInfoType.GET_BEST:
            info.args = gp_instance.get_best_individual()
            pipe.send(info)
        elif info.type == IslandInfoType.LOG_FINAL_INFO:
            gp_instance.get_final_info()
            pipe.send(info_ok)
        else:
            print('Unsupported info:', info)


class DistributedIslandModel:
    """
    Implement the Distributed Island model.

    migrant selection: best, replacement strategy: worst
    migration frequency, number of migrants are set in DimParameters
    """

    def __init__(self, environment: Any, params: DimParameters):
        self.environment = environment
        self.params = params
        self.island_list = []
        self.island_process = []
        self.core_per_instance = max(
            (mp.cpu_count()-2) // self.params.num_island, 1)
        self.neighbors = []
        # initialize all the island instances
        for idx in range(self.params.num_island):
            self.init_island(name=str(idx))
        self.init_neighbors()

    def __del__(self):
        """
        Terminate the island processes manually.

        The island processes are infinite loops therefore we need to terminate them manually
        when we try to delete the DIM.
        """
        for idx in range(self.params.num_island):
            self.island_process[idx].terminate()

    def init_island(self, name):
        """
        Initialize the islands.

        The general dim object controls the islands via sending information using pipe().
        """
        con1, con2 = mp.Pipe()
        self.island_list.append(con1)
        self.island_process.append(mp.Process(
            target=island_process,
            args=(
                con2,
                name,
                self.environment,
                self.params,
                self.core_per_instance,
                self.params.batch_size
            )
        ))
        self.island_process[-1].start()

    def init_neighbors(self):
        """Initialize the neighbor list based on the topology."""
        if self.params.topology == IslandTopology.LOOP:
            for idx in range(self.params.num_island):
                # the next island in the island list
                # is considered as the neighbor
                idx_neighbor = (idx+1) % self.params.num_island
                if idx_neighbor != idx:
                    self.neighbors.append([idx_neighbor])
        elif self.params.topology == IslandTopology.GRID:
            # check if the params are correct
            assert self.params.grid_height_size >= 1
            assert self.params.grid_width_size >= 1
            assert self.params.grid_width_size * \
                self.params.grid_height_size == self.params.num_island
            for idx in range(self.params.num_island):
                my_neighbors = []
                my_pos = idx2coordinate(idx, self.params.grid_width_size)
                # up
                idx_neighbor = coordinate2idx(
                    (my_pos[0] - 1) % self.params.grid_height_size,
                    my_pos[1],
                    self.params.grid_width_size
                )
                my_neighbors.append(idx_neighbor)
                # down
                idx_neighbor = coordinate2idx(
                    (my_pos[0] + 1) % self.params.grid_height_size,
                    my_pos[1],
                    self.params.grid_width_size
                )
                my_neighbors.append(idx_neighbor)
                # left
                idx_neighbor = coordinate2idx(
                    my_pos[0],
                    (my_pos[1] - 1) % self.params.grid_width_size,
                    self.params.grid_width_size
                )
                my_neighbors.append(idx_neighbor)
                # right
                idx_neighbor = coordinate2idx(
                    my_pos[0],
                    (my_pos[1] + 1) % self.params.grid_width_size,
                    self.params.grid_width_size
                )
                my_neighbors.append(idx_neighbor)
                # remove repeated neighbors
                my_neighbors = list(set(my_neighbors))
                # remove the island itself
                if idx in my_neighbors:
                    my_neighbors.remove(idx)
                self.neighbors.append(my_neighbors)
        else:
            raise ValueError('Unexpected type of topology in distributed island model')

    def update_all_islands(self):
        """Have all the islands perform a step of GP update."""
        self.__broadcast_info(IslandInfo(IslandInfoType.STEP_GP))
        self.__check_island_reply()

    def exchange_migrants(self):
        """
        Broadcast migrants to the neighbors.

        The islands merge the pending migrants to their populations.
        The worse individuals in the population are replaced with the pending migrants.
        The purpose the pending list is to wait until the island receive.
        All migrants from different island. In this case, only one replacement will happen.
        """
        for idx in range(self.params.num_island):
            # select best individuals
            get_info = IslandInfo(IslandInfoType.GET_MIGRANT)
            self.island_list[idx].send(get_info)
            migrants, migrant_fitness = self.island_list[idx].recv().args
            send_info = IslandInfo(IslandInfoType.RECV_MIGRANT, args=[migrants, migrant_fitness])
            for idx_neighbor in self.neighbors[idx]:
                # the neighbors receive the migrants
                # and put it in a pending list
                self.island_list[idx_neighbor].send(send_info)
            for idx_neighbor in self.neighbors[idx]:
                assert self.island_list[idx_neighbor].recv().type == IslandInfoType.OK

        # the islands merge the pending migrants to the population
        self.__broadcast_info(IslandInfo(IslandInfoType.MERGE_MIGRANT))
        self.__check_island_reply()

    def run(self):
        """Run the Distributed Island Model."""
        for gen in range(self.params.n_generations):
            self.update_all_islands()
            if (gen + 1) % self.params.migration_frequency == 0:
                self.exchange_migrants()
        self.log_final_info()

    def log_final_info(self):
        """Make logs for each island at the end of the run."""
        self.__broadcast_info(IslandInfo(IslandInfoType.LOG_FINAL_INFO))
        self.__check_island_reply()

    def __broadcast_info(self, info: IslandInfoType):
        """Send the information to all the islands."""
        for idx in range(self.params.num_island):
            self.island_list[idx].send(info)

    def __check_island_reply(self):
        """Check the OK info from the islands to make sure the instructions are completed."""
        for idx in range(self.params.num_island):
            assert self.island_list[idx].recv().type == IslandInfoType.OK

    def get_best_individual(self) -> Tuple[Any, float]:
        """Return the best individual among all islands."""
        population = []
        fitness = []
        self.__broadcast_info(IslandInfo(IslandInfoType.GET_BEST))
        for idx in range(self.params.num_island):
            individual, score = self.island_list[idx].recv().args
            population.append(individual)
            fitness.append(score)
        best_one_fitness = max(fitness)
        best_idx = fitness.index(best_one_fitness)
        return population[best_idx], best_one_fitness


def idx2coordinate(index: int, grid_width: int) -> Tuple[int, int]:
    """Convert index in the island list to coordinates on the grid topology."""
    height = index // grid_width
    width = index % grid_width
    return height, width


def coordinate2idx(height: int, width: int, grid_width: int) -> int:
    """Convert coordinates on the grid topology to index in the list."""
    return height * grid_width + width
