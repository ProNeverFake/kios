# pylint: disable=too-many-instance-attributes
"""
Handling of logs and plots for learning
"""
import os
import shutil
import pickle
from dataclasses import dataclass
import numpy as np
import matplotlib.pyplot as plt
import matplotlib
from scipy import interpolate

matplotlib.rcParams['pdf.fonttype'] = 42

def open_file(path, mode):
    """
    Attempts to open file at path.
    Tried up to max_attempts times because of intermittent permission errors on windows
    """
    max_attempts = 100
    f = None
    for _ in range(max_attempts): # pragma: no branch
        try:
            f = open(path, mode)
        except PermissionError: # pragma: no cover
            continue
        break
    return f

def make_directory(path):
    """
    Attempts to create directory at path.
    Tried up to max_attempts times because of intermittent permission errors on windows
    """
    max_attempts = 100
    for _ in range(max_attempts): # pragma: no branch
        try:
            os.mkdir(path)
        except PermissionError: # pragma: no cover
            continue
        break

def get_log_folder(log_name):
    """ Returns log folder as string """
    if not os.path.exists('logs'):
        os.mkdir('logs')
    return 'logs/log_' + log_name

def trim_logs(logs):
    """ Trims a list of logs so that all logs have the same number of entries/generations """
    min_rowlength = 9999999
    for row in logs:
        rowlen = len(row)
        if rowlen < min_rowlength:
            min_rowlength = rowlen

    for row in logs:
        del row[min_rowlength:]

def clear_logs(log_name):
    """ Clears previous log folders of same same """
    log_folder = get_log_folder(log_name)
    try:
        shutil.rmtree(log_folder)
    except FileNotFoundError: # pragma: no cover
        pass

    make_directory(log_folder)
    fitness_log_path = log_folder + '/fitness_log.txt'
    population_log_path = log_folder + '/population_log.txt'
    open(fitness_log_path, "x")
    open(population_log_path, "x")

def clear_after_generation(log_name, generation):
    """ Clears fitness and population logs after given generation """
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

def log_best_individual(log_name, best_individual):
    """ Saves the best individual """
    with open_file(get_log_folder(log_name) + '/best_individual.pickle', 'wb') as f:
        pickle.dump(best_individual, f)

def log_fitness(log_name, fitness):
    """ Logs fitness of all individuals """
    with open_file(get_log_folder(log_name) + '/fitness_log.txt', 'a') as f:
        f.write("%s\n" % fitness)

def log_best_fitness(log_name, best_fitness):
    """ Logs best fitness of each generation """
    with open_file(get_log_folder(log_name) + '/best_fitness_log.pickle', 'wb') as f:
        pickle.dump(best_fitness, f)

def log_n_episodes(log_name, n_episodes):
    """ Logs number of episodes """
    with open_file(get_log_folder(log_name) + '/n_episodes_log.pickle', 'wb') as f:
        pickle.dump(n_episodes, f)

def log_population(log_name, population):
    """ Logs full population of the generation"""
    with open_file(get_log_folder(log_name) + '/population_log.txt', 'a') as f:
        f.write("%s\n" % population)

def log_last_population(log_name, population):
    """ Logs current population as pickle object """
    with open_file(get_log_folder(log_name) + '/population.pickle', 'wb') as f:
        pickle.dump(population, f)

def log_settings(log_name, settings, baseline):
    """ Logs settings used for the run """
    with open_file(get_log_folder(log_name) + '/settings.txt', 'w') as f:
        for key, value in vars(settings).items():
            f.write(key + ' ' + str(value) + '\n')
        f.write('Baseline: ' + str(baseline) + '\n')

def log_state(log_name, randomstate, np_randomstate, generation):
    """ Logs the current random state and generation number """
    with open_file(get_log_folder(log_name) + '/states.pickle', 'wb') as f:
        pickle.dump(randomstate, f)
        pickle.dump(np_randomstate, f)
        pickle.dump(generation, f)

def get_best_fitness(log_name):
    """ Gets the best fitness list from the given log """
    with open_file(get_log_folder(log_name) + '/best_fitness_log.pickle', 'rb') as f:
        best_fitness = pickle.load(f)
    return best_fitness

def get_n_episodes(log_name):
    """ Gets the list of n_episodes from the given log """
    with open_file(get_log_folder(log_name) + '/n_episodes_log.pickle', 'rb') as f:
        n_episodes = pickle.load(f)
    return n_episodes

def get_state(log_name):
    """ Gets the random state and generation number """
    with open_file(get_log_folder(log_name) + '/states.pickle', 'rb') as f:
        randomstate = pickle.load(f)
        np_randomstate = pickle.load(f)
        generation = pickle.load(f)
    return randomstate, np_randomstate, generation

def get_last_population(log_name):
    """ Gets the last population list from the given log """
    with open_file(get_log_folder(log_name) + '/population.pickle', 'rb') as f:
        population = pickle.load(f)
    return population

def get_best_individual(log_name):
    """ Return the best individual from the given log """
    with open_file(get_log_folder(log_name) + '/best_individual.pickle', 'rb') as f:
        best_individual = pickle.load(f)
    return best_individual

def plot_fitness(log_name, fitness, n_episodes=None):
    """
    Plots fitness over iterations or individuals
    """
    if n_episodes is not None:
        plt.plot(n_episodes, fitness)
        plt.xlabel("Episodes")
    else:
        plt.plot(fitness)
        plt.xlabel("Generation")
    plt.ylabel("Fitness")
    plt.savefig(get_log_folder(log_name) + '/Fitness.png')
    plt.close()

@dataclass
class PlotParameters:
    """ Data class for parameters for plotting """
    plot_mean: bool = True               #Plot the mean of the logs
    mean_color: str = 'b'                #Color for mean curve
    plot_std: bool = True                #Plot the standard deviation
    std_color: str = 'b'                 #Color of the std fill
    plot_minmax: bool = False            #Plots minmax instead of std, should not be combined
    plot_ind: bool = False               #Plot each individual log
    ind_color: str = 'aquamarine'        #Ind color
    label: str = ''                      #Label name
    title: str = ''                      #Plot title
    xlabel: str = ''                     #Label of x axis
    x_max: int = 0                       #Upper limit of x axis
    extend_gens: int = 0                 #Extend until this minimum number of gens
    ylabel: str = ''                     #Label of y axis
    extrapolate_y: bool = False          #Extrapolate y as constant to x_max
    logarithmic_y: bool = False          #Logarithmic y scale
    plot_horizontal: bool = True         #Plot thin horizontal line
    horizontal: float = 0                #Horizontal value to plot
    horizontal_label: str = ''           #Label of horizontal line
    horizontal_linestyle: str = 'dashed' #Style of horizontal line
    legend_position: str = 'lower right'  #Position of legend
    save_fig: bool = True                #Save figure. If false, more plots is possible.
    path: str = 'logs/plot.svg'          #Path to save log

def plot_learning_curves(logs, parameters):
    # pylint: disable=too-many-branches, too-many-statements, too-many-locals
    """
    Plots mean and standard deviation of a number of logs in the same figure
    """
    fitness = []
    n_episodes = []
    for log_name in logs:
        fitness.append(get_best_fitness(log_name))
        n_episodes.append(get_n_episodes(log_name))

    n_logs = len(logs)

    if parameters.extend_gens > 0:
        #Extend until this minimum number of gens, assuming shorter logs are stopped because
        #they have converged there is no difference to end result
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
        plt.plot([0, parameters.x_max], \
                 [parameters.horizontal, parameters.horizontal], \
                 color='k', linestyle=parameters.horizontal_linestyle, linewidth=1, label=parameters.horizontal_label)

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

    plt.legend(loc=parameters.legend_position)
    plt.xlabel(parameters.xlabel)
    if parameters.x_max > 0:
        plt.xlim(0, parameters.x_max)

    if parameters.logarithmic_y:
        plt.yscale('symlog')
        plt.yticks([0, -1, -10, -100], ('0', '-1', '-10', '-100'))
    plt.ylabel(parameters.ylabel)
    plt.title(parameters.title)
    if parameters.save_fig:

        plt.savefig(parameters.path, format='pdf', dpi=300)
        plt.close()
