"""Handle logs and plots for learning."""

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
import os
import pickle
import platform
import shutil
from typing import Any, Dict, List, Tuple

import numpy as np

if platform.python_implementation() == 'PyPy':
    import pypy_matplotlib as matplotlib  # pragma: no cover # pylint: disable=import-error
else:
    import matplotlib.pyplot as plt
    import matplotlib
    from scipy import interpolate

matplotlib.rcParams['pdf.fonttype'] = 42


def open_file(path: str, mode: Any) -> Any:
    """
    Attempt to open file at path.

    Tried up to max_attempts times because of intermittent permission errors on windows.
    """
    max_attempts = 100
    f = None
    for _ in range(max_attempts):  # pragma: no branch
        try:
            f = open(path, mode)  # pylint: disable=unspecified-encoding
        except PermissionError:  # pragma: no cover
            continue
        break
    return f


def make_directory(path: str) -> None:
    """
    Attempt to create directory at path.

    Tried up to max_attempts times because of intermittent permission errors on windows.
    """
    max_attempts = 100
    for _ in range(max_attempts):  # pragma: no branch
        try:
            os.mkdir(path)
        except PermissionError:  # pragma: no cover
            continue
        break


def get_log_folder(log_name: str) -> str:
    """Return log folder as string."""
    if not os.path.exists('logs'):
        os.mkdir('logs')  # pragma: no cover
    return 'logs/log_' + log_name


def trim_logs(logs: Any) -> None:
    """Trim a list of logs so that all logs have the same number of entries/generations."""
    min_rowlength = 9999999
    for row in logs:
        rowlen = len(row)
        if rowlen < min_rowlength:
            min_rowlength = rowlen

    for row in logs:
        del row[min_rowlength:]


def clear_logs(log_name: str) -> None:
    """Clear previous log folders of same same."""
    log_folder = get_log_folder(log_name)
    try:
        shutil.rmtree(log_folder)
    except FileNotFoundError:  # pragma: no cover
        pass

    make_directory(log_folder)
    fitness_log_path = log_folder + '/fitness_log.txt'
    population_log_path = log_folder + '/population_log.txt'
    open(fitness_log_path, 'x', encoding='utf-8')
    open(population_log_path, 'x', encoding='utf-8')


def clear_after_generation(log_name: str, generation: int) -> None:
    """Clear fitness and population logs after given generation."""
    with open_file(get_log_folder(log_name) + '/fitness_log.txt', 'r') as f:
        lines = f.readlines()
    with open_file(get_log_folder(log_name) + '/fitness_log.txt', 'w') as f:
        for i in range(generation + 1):
            f.write(lines[i])
    with open_file(get_log_folder(log_name) + '/population_log.txt', 'r') as f:
        lines = f.readlines()
    with open_file(get_log_folder(log_name) + '/population_log.txt', 'w') as f:
        for i in range(generation + 1):
            f.write(lines[i])


def log_best_individual(log_name: str, best_individual: Any):
    """Save the best individual."""
    with open_file(get_log_folder(log_name) + '/best_individual.pickle', 'wb') as f:
        pickle.dump(best_individual, f)


def log_fitness(log_name: str, fitness: float) -> None:
    """Log fitness of all individuals."""
    with open_file(get_log_folder(log_name) + '/fitness_log.txt', 'a') as f:
        f.write(f'{fitness}\n')


def log_diversity(
    log_name: str,
    diversity: float,
    n_gen: int,
    exchanged: bool
) -> None:
    """
    Log diversity of all individuals.

    Records the current generation count with n_gen.
    exchanged == True when the diversity is measured after exchanging
    """
    with open_file(get_log_folder(log_name) + '/diversity_log.txt', 'a') as f:
        f.write(f'generation: {n_gen}, exchanged: {exchanged}, {diversity}\n')


def log_best_fitness_episodes(log_name: str, fitness: float, n_episodes: int) -> None:
    """Log fitness of the best individual and the corresponding episodes."""
    with open_file(get_log_folder(log_name) + '/fitness_log_episodes.txt', 'a') as f:
        f.write(f'{n_episodes}\n{fitness}\n')


def log_tracking(
    log_name: str,
    num_exchange: int,
    num_from_exchange: int,
    num_from_replication: int,
    num_from_crossover: int,
    num_from_mutation: int
):
    """Log the counters that keep tracks of the methods that generate the new best individual."""
    with open_file(get_log_folder(log_name) + '/tracking_log.txt', 'a') as f:
        f.write(f'Exchange count: {num_from_exchange} / {num_exchange}\n')
        f.write(
            f'GP count: replication: {num_from_replication} crossover: {num_from_crossover}'
            f' mutation: {num_from_mutation}\n'
        )


def log_fitness_exchange(
    log_name: str,
    fitness: float,
    n_gen: int,
    exchanged: bool
) -> None:
    """
    Log fitness of all individuals.

    The function records the current generation count with n_gen.
    Set exchanged to True when the diversity is measured after exchanging.
    """
    with open_file(get_log_folder(log_name) + '/fitness_exchange_log.txt', 'a') as f:
        f.write(f'generation: {n_gen}, exchanged: {exchanged}, {fitness}\n')


def log_best_fitness(log_name: str, best_fitness: float) -> None:
    """Log best fitness of each generation."""
    with open_file(get_log_folder(log_name) + '/best_fitness_log.pickle', 'wb') as f:
        pickle.dump(best_fitness, f)


def log_n_episodes(log_name: str, n_episodes: int) -> None:
    """Log number of episodes."""
    with open_file(get_log_folder(log_name) + '/n_episodes_log.pickle', 'wb') as f:
        pickle.dump(n_episodes, f)


def log_population(log_name: str, population: List[Any]) -> None:
    """Log full population of the generation."""
    with open_file(get_log_folder(log_name) + '/population_log.txt', 'a') as f:
        f.write(f'{population}\n')


def log_last_population(log_name: str, population: List[Any]) -> None:
    """Log current population as pickle object."""
    with open_file(get_log_folder(log_name) + '/population.pickle', 'wb') as f:
        pickle.dump(population, f)


def log_settings(log_name: str, settings: Dict, baseline: Any) -> None:
    """Log settings used for the run."""
    with open_file(get_log_folder(log_name) + '/settings.txt', 'w') as f:
        for key, value in vars(settings).items():
            f.write(key + ' ' + str(value) + '\n')
        f.write('Baseline: ' + str(baseline) + '\n')


def log_state(
    log_name: str,
    randomstate: Any,
    np_randomstate: float,
    generation: int
) -> None:
    """Log the current random state and generation number."""
    with open_file(get_log_folder(log_name) + '/states.pickle', 'wb') as f:
        pickle.dump(randomstate, f)
        pickle.dump(np_randomstate, f)
        pickle.dump(generation, f)


def get_best_fitness(log_name: str) -> List[float]:
    """Get the best fitness list from the given log."""
    with open_file(get_log_folder(log_name) + '/best_fitness_log.pickle', 'rb') as f:
        best_fitness = pickle.load(f)
    return best_fitness


def get_n_episodes(log_name: str) -> int:
    """Get the list of n_episodes from the given log."""
    with open_file(get_log_folder(log_name) + '/n_episodes_log.pickle', 'rb') as f:
        n_episodes = pickle.load(f)
    return n_episodes


def get_state(log_name: str) -> Tuple[Any, float, int]:
    """Get the random state and generation number."""
    with open_file(get_log_folder(log_name) + '/states.pickle', 'rb') as f:
        randomstate = pickle.load(f)
        np_randomstate = pickle.load(f)
        generation = pickle.load(f)
    return randomstate, np_randomstate, generation


def get_last_population(log_name: str) -> List[Any]:
    """Get the last population list from the given log."""
    with open_file(get_log_folder(log_name) + '/population.pickle', 'rb') as f:
        population = pickle.load(f)
    return population


def get_best_individual(log_name: str) -> Any:
    """Return the best individual from the given log."""
    with open_file(get_log_folder(log_name) + '/best_individual.pickle', 'rb') as f:
        best_individual = pickle.load(f)
    return best_individual


def plot_fitness(log_name: str, fitness: float, n_episodes: int = None) -> None:
    """Plot fitness over iterations or individuals."""
    if n_episodes is not None:
        plt.plot(n_episodes, fitness)
        plt.xlabel('Episodes')
    else:
        plt.plot(fitness)
        plt.xlabel('Generation')
    plt.ylabel('Fitness')
    plt.savefig(get_log_folder(log_name) + '/Fitness.png')
    plt.savefig(get_log_folder(log_name) + '/Fitness.svg')
    plt.close()


# pylint: disable=too-many-instance-attributes
@dataclass
class PlotParameters:
    """Data class for parameters for plotting."""

    plot_mean: bool = True                # Plot the mean of the logs
    mean_color: str = 'b'                 # Color for mean curve
    plot_std: bool = True                 # Plot the standard deviation
    std_color: str = 'b'                  # Color of the std fill
    plot_minmax: bool = False             # Plots minmax instead of std, should not be combined
    plot_ind: bool = False                # Plot each individual log
    ind_color: str = 'aquamarine'         # Ind color
    label: str = ''                       # Label name
    title: str = ''                       # Plot title
    title_fontsize: int = 24              # Font size for the title
    xlabel: str = ''                      # Label of x axis
    xlabel_fontsize: int = 18             # Font size for the x axis
    fig_width: int = 6                    # Width of figure
    fig_height: int = 4                   # Height of figure
    x_max: int = 0                        # Upper limit of x axis
    y_max: int = -1000                    # Upper limit of y axis
    y_min: int = 1000                     # Lower limit of y axis
    extend_gens: int = 0                  # Extend until this minimum number of gens
    ylabel: str = ''                      # Label of y axis
    ylabel_fontsize: int = 18             # Font size for the y axis
    extrapolate_y: bool = False           # Extrapolate y as constant to x_max
    logarithmic_y: bool = False           # Logarithmic y scale
    plot_horizontal: bool = True          # Plot thin horizontal line
    horizontal: float = 0                 # Horizontal value to plot
    horizontal_label: str = ''            # Label of horizontal line
    horizontal_linestyle: str = 'dashed'  # Style of horizontal line
    horizontal_color: str = 'b'           # Color of horizontal line
    legend_position: str = 'lower right'  # Position of legend
    legend_fontsize: int = 18             # Font size for the legend
    save_fig: bool = True                 # Save figure. If false, more plots is possible.
    path: str = 'logs/plot.svg'           # Path to save log


def plot_learning_curves(logs: List[str], parameters: PlotParameters) -> None:
    # pylint: disable=too-many-branches, too-many-statements, too-many-locals
    """Plot mean and standard deviation of a number of logs in the same figure."""
    plt.rcParams['figure.figsize'] = (parameters.fig_width, parameters.fig_height)
    fitness = []
    n_episodes = []
    for log_name in logs:
        fitness.append(get_best_fitness(log_name))
        n_episodes.append(get_n_episodes(log_name))

    n_logs = len(logs)

    if parameters.extend_gens > 0:
        # Extend until this minimum number of gens, assuming shorter logs are stopped because
        # they have converged there is no difference to end result
        for i in range(n_logs):
            if len(fitness[i]) < parameters.extend_gens:
                last_fitness = fitness[i][-1]

                while len(fitness[i]) < parameters.extend_gens:
                    fitness[i].append(last_fitness)
                    n_episodes[i].append(parameters.x_max)

    trim_logs(fitness)
    trim_logs(n_episodes)

    fitness = np.array(fitness)
    n_episodes = np.array(n_episodes)

    startx = np.max(n_episodes[:, 0])
    endx = np.min(n_episodes[:, -1])
    if parameters.extrapolate_y:
        x = np.arange(startx, parameters.x_max + 1)
    else:
        x = np.arange(startx, endx + 1)

    if parameters.plot_horizontal:
        plt.plot([0, parameters.x_max],
                 [parameters.horizontal, parameters.horizontal],
                 color=parameters.horizontal_color, linestyle=parameters.horizontal_linestyle,
                 linewidth=1, label=parameters.horizontal_label)

    y = np.zeros((len(x), n_logs))
    for i in range(0, n_logs):
        f = interpolate.interp1d(n_episodes[i, :], fitness[i, :], bounds_error=False)
        y[:, i] = f(x)
        if parameters.extrapolate_y:
            n_extrapolated = int(parameters.x_max - n_episodes[i, -1])
            if n_extrapolated > 0:
                left = y[:n_episodes[i, -1] - n_episodes[i, 0] + 1, i]
                y[:, i] = np.concatenate((left, np.full(n_extrapolated, left[-1])))
        if parameters.plot_ind:
            plt.plot(x, y[:, i], color=parameters.ind_color, linestyle='dashed', linewidth=1)

    y_mean = np.mean(y, axis=1)
    if parameters.plot_mean:
        plt.plot(x, y_mean, color=parameters.mean_color, label=parameters.label)

    if parameters.plot_std:
        y_std = np.std(y, axis=1)
        plt.fill_between(x, y_mean - y_std, y_mean + y_std, alpha=.1, color=parameters.std_color)
    if parameters.plot_minmax:
        maxcurve = np.max(y, axis=1)
        mincurve = np.min(y, axis=1)
        plt.fill_between(x, mincurve, maxcurve, alpha=.1, color=parameters.std_color)

    plt.legend(loc=parameters.legend_position, fontsize=parameters.legend_fontsize)
    plt.xlabel(parameters.xlabel, fontsize=parameters.xlabel_fontsize)
    if parameters.x_max > 0:
        plt.xlim(0, parameters.x_max)
    if parameters.y_max > -1000:
        plt.ylim(top=parameters.y_max)
    if parameters.y_min < 1000:
        plt.ylim(bottom=parameters.y_min)

    if parameters.logarithmic_y:
        plt.yscale('symlog')
        plt.yticks([0, -1, -10, -100], ('0', '-1', '-10', '-100'))
    plt.ylabel(parameters.ylabel, fontsize=parameters.ylabel_fontsize)
    plt.title(parameters.title, fontsize=parameters.title_fontsize)
    if parameters.save_fig:
        plt.tight_layout(pad=0.05)
        plt.savefig(parameters.path, format='svg', dpi=300)
        plt.close()
