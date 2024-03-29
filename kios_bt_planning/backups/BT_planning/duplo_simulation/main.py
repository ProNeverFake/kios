# pylint: disable=line-too-long
"""
Main script for running duplo simulations
"""
import sys
import time

import behavior_tree_learning.behavior_tree as behavior_tree
import behavior_tree_learning.genetic_programming as gp
import behavior_tree_learning.logplot as logplot
import duplo_simulation.agx_interface as agx_interface
from duplo_simulation.environment import Environment
from duplo_simulation.fitness_function import Coefficients

def run_individual():
    """ This is just for testing """
    paper_figures()

def paper_run():
    # pylint: disable=too-many-branches, too-many-statements
    """ Run this to produce results from the paper 'Combining Planning and Learning of Behavior Trees for Robotic Assembly' """
    rosid = "1"
    if len(sys.argv) > 1:
        rosid = sys.argv[1]

    behavior_tree.load_settings_from_file('BT_SETTINGS_TOWER.yaml')
    gp_par = gp.GpParameters()
    gp_par.ind_start_length = 8
    gp_par.n_population = 16
    gp_par.f_crossover = 0.5
    gp_par.n_offspring_crossover = 2
    gp_par.replace_crossover = False
    gp_par.f_mutation = 0.5
    gp_par.n_offspring_mutation = 2
    gp_par.parent_selection = gp.SelectionMethods.RANK
    gp_par.survivor_selection = gp.SelectionMethods.RANK
    gp_par.f_elites = 0.1
    gp_par.f_parents = gp_par.f_elites
    gp_par.mutate_co_offspring = False
    gp_par.mutate_co_parents = True
    gp_par.mutation_p_add = 0.4
    gp_par.mutation_p_delete = 0.3
    gp_par.allow_identical = False
    gp_par.plot = True
    gp_par.n_generations = 200
    gp_par.verbose = False
    gp_par.fig_last_gen = False

    world_interface = agx_interface.AgxInterface(rosid)

    ########################################################################################
    # Tower
    ########################################################################################
    targets = []
    targets.append(agx_interface.Pos(0.0, 0.05, 0))
    targets.append(agx_interface.Pos(0.0, 0.05, 0.0192))
    targets.append(agx_interface.Pos(0.0, 0.05, 2*0.0192))
    environment = Environment(world_interface, targets, verbose=False)

    planner_baseline = ['s(', 'f(', '0 at pos (0.0, 0.05, 0.0)?', 's(', 'f(', 'picked 0?', 'pick 0!', ')', 'place at (0.0, 0.05, 0.0)!', ')', ')', \
                        'f(', '1 at pos (0.0, 0.05, 0.0192)?', 's(', 'f(', '1 on 0?', 's(', 'f(', 'picked 1?', 'pick 1!', ')', 'place on 0!', ')', ')', 'apply force 1!', ')', ')',  \
                        'f(', '2 at pos (0.0, 0.05, 0.0384)?', 's(', 'f(', '2 on 1?', 's(', 'f(', 'picked 2?', 'pick 2!', ')', 'place on 1!', ')', ')', 'apply force 2!', ')', ')', ')'] #-3.0109... 0.0109

    #Best from experiments
    #planner_baseline = ['s(', 'f(', '0 at pos (0.0, 0.05, 0.0)?', 's(', 'pick 0!', 'place at (0.0, 0.05, 0.0)!', ')', ')', \
    #                    'f(', '1 at pos (0.0, 0.05, 0.0192)?', 's(', 'f(', '1 on 0?', 's(', 'pick 1!', 'place on 0!', ')', ')', 'apply force 1!', ')', ')',  \
    #                    'f(', '2 at pos (0.0, 0.05, 0.0384)?', 's(', 'f(', '2 on 1?', 's(', 'pick 2!', 'place on 1!', ')', ')', 'apply force 2!', ')', ')', ')'] #-2.4109819330136357 0.0109

    #planner_baseline = ['s(', 'f(', '0 at pos (0.0, 0.05, 0.0)?', 's(', 'pick 0!', 'place at (0.0, 0.05, 0.0)!', ')', ')', \
    #                    'f(', '1 on 0?', 's(', 'pick 1!', 'place on 0!', ')', ')', \
    #                    'f(', '1 at pos (0.0, 0.05, 0.0192)?', 'apply force 1!', ')',  \
    #                    'f(', '2 on 1?', 's(', 'pick 2!', 'place on 1!', ')', ')', \
    #                    'f(', '2 at pos (0.0, 0.05, 0.0384)?', 'apply force 2!', ')',')'] #-2.2109819330136357 0.0109

    n_logs = 10
    for i in range(1, n_logs + 1):
        gp_par.log_name = 'tower_no_baseline_' + str(i)
        gp.set_seeds(i)
        gp.run(environment, gp_par)

    for i in range(1, n_logs + 1):
        gp_par.log_name = 'tower_planner_baseline_' + str(i)
        gp.set_seeds(i)
        gp.run(environment, gp_par, baseline=planner_baseline)

    for i in range(0):
        start = time.time()
        fitness = environment.get_fitness(planner_baseline)
        print(str(i) + ": " + str(time.time() - start))
        print(fitness)

    ########################################################################################
    # Croissant
    ########################################################################################
    behavior_tree.load_settings_from_file('BT_SETTINGS_CROISSANT.yaml')
    planner_baseline = ['s(', 'f(', '0 at pos (0.0, 0.0, 0.0)?', 's(', 'f(', 'picked 0?', 'pick 0!', ')', 'place at (0.0, 0.0, 0.0)!', ')', ')', \
                        'f(', '1 at pos (0.0, 0.0, 0.0192)?', 's(', 'f(', '1 on 0?', 's(', 'f(', 'picked 1?', 'pick 1!', ')', 'place on 0!', ')', ')', 'apply force 1!', ')', ')', \
                        'f(', '2 at pos (0.016, -0.032, 0.0)?', 's(', 'f(', 'picked 2?', 'pick 2!', ')', 'place at (0.016, -0.032, 0.0)!', ')', ')', \
                        'f(', '3 at pos (0.016, 0.032, 0.0)?', 's(', 'f(', 'picked 3?', 'pick 3!', ')', 'place at (0.016, 0.032, 0.0)!', ')', ')', ')'] #-237...

    #best from experiments
    #planner_baseline = ['s(', 'f(', '0 at pos (0.0, 0.0, 0.0)?', 's(', 'pick 0!', 'place at (0.0, 0.0, 0.0)!', ')', ')', \
    #                    'f(', '2 at pos (0.016, -0.032, 0.0)?', 's(', 'pick 2!', 'place at (0.016, -0.032, 0.0)!', ')', ')', \
    #                    'f(', '3 at pos (0.016, 0.032, 0.0)?', 's(', 'pick 3!', 'place at (0.016, 0.032, 0.0)!', ')', ')', \
    #                    'f(', '1 at pos (0.0, 0.0, 0.0192)?', 's(', 'f(', '1 on 0?', 's(', 'pick 1!', 'place on 0!', ')', ')', 'apply force 1!', ')', ')', ')'] #-2.5
    #planner_baseline = ['s(', 'f(', '0 at pos (0.0, 0.0, 0.0)?', 's(', 'pick 0!', 'place at (0.0, 0.0, 0.0)!', ')', ')', \
    #                    'f(', '3 at pos (0.016, 0.032, 0.0)?', 's(', 'pick 3!', 'place at (0.016, 0.032, 0.0)!', ')', ')', \
    #                    'f(', '2 at pos (0.016, -0.032, 0.0)?', 's(', 'pick 2!', 'place at (0.016, -0.032, 0.0)!', ')', ')', \
    #                    'f(', '1 on 0?', 's(', 'pick 1!', 'place on 0!', ')', ')',
    #                    'f(', '1 at pos (0.0, 0.0, 0.0192)?', 'apply force 1!', ')', ')'] #-2.4
    #planner_baseline = ['s(', 'f(', '0 at pos (0.0, 0.0, 0.0)?', 's(', 'pick 0!', 'place at (0.0, 0.0, 0.0)!', ')', ')', \
    #                    'f(', '1 at pos (0.0, 0.0, 0.0192)?', 's(', 'f(', '1 on 0?', 's(', 'f(', '2 at pos (0.016, -0.032, 0.0)?', 's(', '3 at pos (0.016, 0.032, 0.0)?', 'pick 2!', 'place at (0.016, -0.032, 0.0)!', ')', 'place at (0.016, 0.032, 0.0)!', ')', \
    #                    'pick 1!', 'place on 0!', ')', ')', 'apply force 1!', ')', 'pick 3!', ')', ')'] #-2.33... Very clever with brick 3..

    targets = []
    targets.append(agx_interface.Pos(0.0, 0.0, 0.0))
    targets.append(agx_interface.Pos(0.0, 0.0, 0.0192))
    targets.append(agx_interface.Pos(0.016, -0.032, 0.0))
    targets.append(agx_interface.Pos(0.016, 0.032, 0.0))
    environment = Environment(world_interface, targets, verbose=False)
    gp_par.n_generations = 1000

    n_logs = 5

    for i in range(1, n_logs + 1):
        gp_par.log_name = 'cro_planner_baseline_' + str(i)
        gp.set_seeds(i)
        #gp.run(environment, gp_par, baseline=planner_baseline)

    for i in range(1, n_logs + 1):
        gp_par.log_name = 'cro_planner_baseline_boost_co_' + str(i)
        gp_par.boost_baseline = True
        gp_par.boost_baseline_only_co = True
        gp.set_seeds(i)
        gp.run(environment, gp_par, baseline=planner_baseline, hotstart=False)
        gp_par.boost_baseline = False

    for i in range(1, n_logs + 1):
        gp_par.log_name = 'cro_planner_baseline_boost_all_' + str(i)
        gp_par.boost_baseline = True
        gp_par.boost_baseline_only_co = False
        gp.set_seeds(i)
        gp.run(environment, gp_par, baseline=planner_baseline, hotstart=False)
        gp_par.boost_baseline = False
        gp_par.boost_baseline_only_co = True

    for i in range(1, n_logs + 1):
        gp_par.log_name = 'cro_no_baseline_' + str(i)
        gp.set_seeds(i)
        gp.run(environment, gp_par)

    for i in range(0):
        start = time.time()
        fitness = environment.get_fitness(planner_baseline)
        print(str(i) + ": " + str(time.time() - start))
        print(fitness)

    ########################################################################################
    # Balance
    ########################################################################################
    behavior_tree.load_settings_from_file('BT_SETTINGS_BALANCE.yaml')
    planner_baseline = ['s(', 'f(', '0 at pos (0.0, 0.0, 0.0192)?', 's(', 'f(', '1 at pos (0.0, 0.0, 0.0)?', 'put 1 at (0.0, 0.0, 0.0)!', ')', 'f(', '0 on 1?', 'put 0 on 1!', ')', 'apply force 0!', ')', ')', ')'] #-117.7... 6.6...

    #Solved
    #planner_baseline = ['f(', '0 at pos (0.0, 0.0, 0.0192)?', 's(', 'put 2 at (0.0, 0.0, 0.0)!', 'put 0 on 2!', 'apply force 0!', ')', ')'] #-0.6...

    targets = []
    targets.append(agx_interface.Pos(0.0, 0.0, 0.0192))
    fitness_coeff = Coefficients()
    fitness_coeff.hand_not_empty = 100.0

    environment = Environment(world_interface, targets, verbose=False, fitness_coeff=fitness_coeff)
    gp_par.n_generations = 800

    n_logs = 10

    for i in range(1, n_logs + 1):
        gp_par.log_name = 'balance_planner_baseline_' + str(i)
        gp.set_seeds(i)
        gp.run(environment, gp_par, baseline=planner_baseline, hotstart=False)

    for i in range(1, n_logs + 1):
        gp_par.log_name = 'balance_no_baseline_' + str(i)
        gp.set_seeds(i)
        gp.run(environment, gp_par, hotstart=False)

    for i in range(1, n_logs + 1):
        gp_par.log_name = 'balance_planner_baseline_boost_all_' + str(i)
        gp_par.boost_baseline = True
        gp_par.boost_baseline_only_co = False
        gp.set_seeds(i)
        gp.run(environment, gp_par, baseline=planner_baseline, hotstart=False)
        gp_par.boost_baseline = False
        gp_par.boost_baseline_only_co = True

    for i in range(1, n_logs + 1):
        gp_par.log_name = 'balance_planner_baseline_boost_co_' + str(i)
        gp_par.boost_baseline = True
        gp_par.boost_baseline_only_co = True
        gp.set_seeds(i)
        gp.run(environment, gp_par, baseline=planner_baseline, hotstart=False)
        gp_par.boost_baseline = False

    for i in range(0):
        start = time.time()
        fitness = environment.get_fitness(planner_baseline)
        print(str(i) + ": " + str(time.time() - start))
        print(fitness)

    ########################################################################################
    # Blocking
    ########################################################################################
    behavior_tree.load_settings_from_file('BT_SETTINGS_BLOCKING.yaml')
    planner_baseline = ['s(', 'f(', '0 at pos (-0.1, 0.0, 0.0)?', 'put 0 at (-0.1, 0.0, 0.0)!', ')', \
                        'f(', '1 at pos (-0.1, 0.0, 0.0192)?', 's(', 'f(', '1 on 0?', 'put 1 on 0!', ')', 'apply force 1!', ')', ')', \
                        'f(', '2 at pos (0.0, 0.0, 0.0)?', 'put 2 at (0.0, 0.0, 0.0)!', ')', ')'] #-288...

    #Solved
    #planner_baseline = ['s(', 'put 1 on 0!', 'f(', '0 at pos (-0.1, 0.0, 0.0)?', 'put 2 at (0.0, 0.05, 0.0)!', ')', \
    #                    'put 0 at (-0.1, 0.0, 0.0)!', \
    #                    'f(', '1 at pos (-0.1, 0.0, 0.0192)?', 'apply force 1!', ')', \
    #                    'put 2 at (0.0, 0.0, 0.0)!', ')'] #-1.0

    #planner_baseline = ['s(', 'f(', '1 at pos (-0.1, 0.0, 0.0192)?', \
    #                                's(', 'put 1 on 0!', 'put 2 at (0.0, 0.05, 0.0)!', 'put 0 at (-0.1, 0.0, 0.0)!', 'apply force 1!', ')', ')',
    #                          'put 2 at (0.0, 0.0, 0.0)!', ')'] #-0.9

    targets = []
    targets.append(agx_interface.Pos(-0.1, 0.0, 0.0))
    targets.append(agx_interface.Pos(-0.1, 0.0, 0.0192))
    targets.append(agx_interface.Pos(0.0, 0.0, 0.0))
    fitness_coeff = Coefficients()
    fitness_coeff.hand_not_empty = 0.0

    environment = Environment(world_interface, targets, verbose=False, fitness_coeff=fitness_coeff)
    gp_par.n_generations = 300

    n_logs = 5
    for i in range(1, n_logs + 1):
        gp_par.log_name = 'block_planner_baseline_' + str(i)
        gp.set_seeds(i)
        #gp.run(environment, gp_par, baseline=planner_baseline)


    for i in range(1, n_logs + 1):
        gp_par.log_name = 'block_no_baseline_' + str(i)
        gp.set_seeds(i)
        gp.run(environment, gp_par, hotstart=False)

    for i in range(1, n_logs + 1):
        gp_par.log_name = 'block_planner_baseline_boost_co_' + str(i)
        gp_par.boost_baseline = True
        gp_par.boost_baseline_only_co = True
        gp.set_seeds(i)
        gp.run(environment, gp_par, baseline=planner_baseline, hotstart=False)
        gp_par.boost_baseline = False

    for i in range(1, n_logs + 1):
        gp_par.log_name = 'block_planner_baseline_boost_all_' + str(22)
        gp_par.boost_baseline = True
        gp_par.boost_baseline_only_co = False
        gp.set_seeds(i)
        gp.run(environment, gp_par, baseline=planner_baseline, hotstart=False)
        gp_par.boost_baseline = False
        gp_par.boost_baseline_only_co = True

    for i in range(0):
        start = time.time()
        fitness = environment.get_fitness(planner_baseline)
        print(str(i) + " time: " + str(time.time() - start))
        print(fitness)

    world_interface.shutdown()
    paper_plots()

def paper_plots():
    # pylint: disable=too-many-branches, too-many-statements
    """ Creates plots given that paper_run has been executed before to produce logs """
    plotpars = logplot.PlotParameters()
    plotpars.plot_std = True
    plotpars.xlabel = 'Episodes'
    plotpars.ylabel = 'Fitness'

    ########################################################################################
    # Tower figure
    ########################################################################################
    n_logs = 10
    logs = []
    for i in range(1, n_logs + 1):
        logs.append('tower_planner_baseline_' + str(i))
    plotpars.title = ''
    plotpars.mean_color = 'b'
    plotpars.std_color = 'b'
    plotpars.x_max = 5000
    plotpars.logarithmic_y = False
    plotpars.horizontal = -2.2
    plotpars.horizontal_label = 'Best'
    plotpars.save_fig = False
    plotpars.label = 'From planner baseline'
    plotpars.legend_position = 'lower right'
    logplot.plot_learning_curves(logs, plotpars)

    logs = []
    for i in range(1, n_logs + 1):
        logs.append('tower_no_baseline_' + str(i))
    plotpars.mean_color = 'r'
    plotpars.std_color = 'r'
    plotpars.label = 'No baseline'
    plotpars.horizontal = -3.0
    plotpars.horizontal_label = 'Planner baseline'
    plotpars.horizontal_linestyle = 'dotted'
    plotpars.save_fig = True
    plotpars.save_fig = True
    plotpars.path = 'logs/tower.pdf'
    logplot.plot_learning_curves(logs, plotpars)

    ########################################################################################
    # Croissant figure
    ########################################################################################
    n_logs = 5
    logs = []
    for i in range(1, n_logs + 1):
        logs.append('cro_planner_baseline_' + str(i))
    plotpars.title = ''#'Learning curves for task 2'
    plotpars.mean_color = 'b'
    plotpars.std_color = 'b'
    plotpars.label = 'From planner baseline'
    plotpars.save_fig = False
    plotpars.horizontal = -2.33
    plotpars.horizontal_label = 'Best'
    plotpars.horizontal_linestyle = 'dashed'
    plotpars.extend_gens = 1000
    plotpars.x_max = 28000
    plotpars.logarithmic_y = False
    logplot.plot_learning_curves(logs, plotpars)

    logs = []
    for i in range(1, n_logs + 1):
        logs.append('cro_no_baseline_' + str(i))
    plotpars.mean_color = 'r'
    plotpars.std_color = 'r'
    plotpars.label = 'No baseline'
    plotpars.horizontal = -237.2
    plotpars.horizontal_label = 'Planner baseline'
    plotpars.horizontal_linestyle = 'dotted'
    logplot.plot_learning_curves(logs, plotpars)

    logs = []
    for i in range(1, n_logs + 1):
        logs.append('cro_planner_baseline_boost_co_' + str(i))
    plotpars.mean_color = 'g'
    plotpars.std_color = 'g'
    plotpars.extend_gens = 0
    plotpars.plot_horizontal = False
    plotpars.label = 'Baseline boosted for crossover'
    logplot.plot_learning_curves(logs, plotpars)

    logs = []
    for i in range(1, n_logs + 1):
        logs.append('cro_planner_baseline_boost_all_' + str(i))
    plotpars.mean_color = 'k'
    plotpars.std_color = 'k'
    plotpars.label = 'Baseline boosted for mutation and crossover'
    plotpars.save_fig = True
    plotpars.path = 'logs/cro.pdf'
    logplot.plot_learning_curves(logs, plotpars)

    ########################################################################################
    # Balance figure
    ########################################################################################
    n_logs = 10
    logs = []
    for i in range(1, n_logs + 1):
        logs.append('balance_planner_baseline_' + str(i))
    plotpars.title = ''#'Learning curves for task 3'
    plotpars.mean_color = 'b'
    plotpars.std_color = 'b'
    plotpars.label = 'From planner baseline'
    plotpars.save_fig = False
    plotpars.extend_gens = 1200
    plotpars.horizontal = -0.6
    plotpars.horizontal_label = 'Best'
    plotpars.horizontal_linestyle = 'dashed'
    plotpars.plot_horizontal = True
    plotpars.legend_position = (0.25, 0.1)
    plotpars.x_max = 14000
    plotpars.logarithmic_y = False
    logplot.plot_learning_curves(logs, plotpars)

    logs = []
    for i in range(1, n_logs + 1):
        logs.append('balance_no_baseline_' + str(i))
    plotpars.mean_color = 'r'
    plotpars.std_color = 'r'
    plotpars.label = 'No baseline'
    plotpars.horizontal = -117.7
    plotpars.horizontal_label = 'Planner baseline'
    plotpars.horizontal_linestyle = 'dotted'
    plotpars.plot_horizontal = True
    logplot.plot_learning_curves(logs, plotpars)

    logs = []
    for i in range(1, n_logs + 1):
        logs.append('balance_planner_baseline_boost_co_' + str(i))
    plotpars.mean_color = 'g'
    plotpars.std_color = 'g'
    plotpars.label = 'Baseline boosted for crossover'
    plotpars.plot_horizontal = False
    logplot.plot_learning_curves(logs, plotpars)

    logs = []
    for i in range(1, n_logs + 1):
        logs.append('balance_planner_baseline_boost_all_' + str(i))
    plotpars.mean_color = 'k'
    plotpars.std_color = 'k'
    plotpars.label = 'Baseline boosted for mutation and crossover'
    plotpars.save_fig = True
    plotpars.path = 'logs/balance.pdf'
    logplot.plot_learning_curves(logs, plotpars)
    plotpars.extend_gens = 0

    ########################################################################################
    # Blocking figure
    ########################################################################################
    n_logs = 5
    logs = []
    for i in range(1, n_logs + 1):
        logs.append('block_planner_baseline_' + str(i))
    plotpars.title = ''#'Learning curves for task 4'
    plotpars.mean_color = 'b'
    plotpars.std_color = 'b'
    plotpars.label = 'From planner baseline'
    plotpars.save_fig = False
    plotpars.plot_horizontal = True
    plotpars.horizontal = -1.0
    plotpars.horizontal_label = 'Best'
    plotpars.horizontal_linestyle = 'dashed'
    plotpars.legend_position = (0.25, 0.1)
    plotpars.extend_gens = 300
    plotpars.x_max = 8000
    plotpars.logarithmic_y = False
    logplot.plot_learning_curves(logs, plotpars)

    logs = []
    for i in range(1, n_logs + 1):
        logs.append('block_no_baseline_' + str(i))
    plotpars.mean_color = 'r'
    plotpars.std_color = 'r'
    plotpars.label = 'No baseline'
    plotpars.horizontal = -288
    plotpars.horizontal_label = 'Planner baseline'
    plotpars.horizontal_linestyle = 'dotted'
    logplot.plot_learning_curves(logs, plotpars)

    logs = []
    for i in range(1, n_logs + 1):
        logs.append('block_planner_baseline_boost_co_' + str(i))
    plotpars.mean_color = 'g'
    plotpars.std_color = 'g'
    plotpars.label = 'Baseline boosted for crossover'
    plotpars.plot_horizontal = False
    logplot.plot_learning_curves(logs, plotpars)
    plotpars.extend_gens = 0

    logs = []
    for i in range(1, n_logs + 1):
        logs.append('block_planner_baseline_boost_all_' + str(i))
    plotpars.extrapolate_y = True
    plotpars.mean_color = 'k'
    plotpars.std_color = 'k'
    plotpars.label = 'Baseline boosted for mutation and crossover'
    plotpars.save_fig = True
    plotpars.path = 'logs/blocking.pdf'
    logplot.plot_learning_curves(logs, plotpars)
    plotpars.extend_gens = 0
    plotpars.extrapolate_y = False

    ########################################################################################
    # Blocking baseline tests in sm
    ########################################################################################
    n_logs = 10
    logs = []
    for i in range(1, n_logs + 1):
        logs.append('test_baseline_no_keep' + str(i))
    plotpars.title = ''#'Baseline tests'
    plotpars.mean_color = 'b'
    plotpars.std_color = 'b'
    plotpars.plot_std = False
    plotpars.label = 'Not keeping baseline'
    plotpars.save_fig = False
    plotpars.plot_horizontal = True
    plotpars.horizontal = -0.9
    plotpars.horizontal_label = 'Optimal'
    plotpars.x_max = 12000
    plotpars.logarithmic_y = False
    logplot.plot_learning_curves(logs, plotpars)

    logs = []
    for i in range(1, n_logs + 1):
        logs.append('test_baseline' + str(i))
    plotpars.mean_color = 'r'
    plotpars.std_color = 'r'
    plotpars.plot_horizontal = False
    plotpars.label = 'Always baseline'
    logplot.plot_learning_curves(logs, plotpars)

    logs = []
    for i in range(1, n_logs + 1):
        logs.append('test_baseline_boost' + str(i))
    plotpars.mean_color = 'g'
    plotpars.std_color = 'g'
    plotpars.label = 'Boost baseline fitness for both mutation and crossover'
    logplot.plot_learning_curves(logs, plotpars)

    logs = []
    for i in range(1, n_logs + 1):
        logs.append('test_baseline_boost_only_co' + str(i))
    plotpars.mean_color = 'k'
    plotpars.std_color = 'k'
    plotpars.label = 'Boost baseline fitness only for crossover'
    plotpars.save_fig = True
    plotpars.path = 'logs/baseline.pdf'
    logplot.plot_learning_curves(logs, plotpars)

def paper_figures():
    # pylint: disable=import-outside-toplevel
    """ Creates the bt figures used in the paper """
    from behavior_tree_learning.py_trees_interface import PyTree
    import behavior_tree_learning.behaviors_figures as behaviors

    behavior_tree.load_settings_from_file('BT_SETTINGS_TOWER.yaml')
    # Tower planned
    tree = ['s(', 'f(', 'Green at A?', 's(', 'f(', 'picked Green?', 'pick Green!', ')', 'place at A!', ')', ')', \
            'f(', 'Blue at pos B?', 's(', 'f(', 'Blue on Green?', 's(', 'f(', 'picked Blue?', 'pick Blue!', ')', 'place on Green!', ')', ')', 'apply force Blue!', ')', ')',  \
            'f(', 'Red at pos C?', 's(', 'f(', 'Red on Blue?', 's(', 'f(', 'picked Red?', 'pick Red!', ')', 'place on Blue!', ')', ')', 'apply force Red!', ')', ')', ')']
    pytree = PyTree(tree[:], behaviors=behaviors)
    pytree.save_fig('logs/paperfigures/', name='tower_planned', svg=True)

    # Tower solved
    tree = ['s(', 'f(', 'Green at pos A?', 's(', 'pick Green!', 'place at A!', ')', ')', \
            'f(', 'Blue at pos B?', 's(', 'f(', 'Blue on Green?', 's(', 'pick Blue!', 'place on Green!', ')', ')', 'apply force Blue!', ')', ')',  \
            'f(', 'Red at pos C?', 's(', 'f(', 'Red on Blue?', 's(', 'pick Red!', 'place on Blue!', ')', ')', 'apply force Red!', ')', ')', ')']
    pytree = PyTree(tree[:], behaviors=behaviors)
    pytree.save_fig('logs/paperfigures/', name='tower_solved', svg=True)

    behavior_tree.load_settings_from_file('BT_SETTINGS_CROISSANT.yaml')
    # Croissant planned
    tree = ['s(', 'f(', 'Green at pos B?', 's(', 'f(', 'picked Green?', 'pick Green!', ')', 'place at B!', ')', ')', \
            'f(', 'Blue at pos D?', 's(', 'f(', 'Blue on Green?', 's(', 'f(', 'picked Blue?', 'pick Blue!', ')', 'place on Green!', ')', ')', 'apply force Blue!', ')', ')', \
            'f(', 'Red at pos A?', 's(', 'f(', 'picked Red?', 'pick Red!', ')', 'place at A!', ')', ')', \
            'f(', 'Yellow at pos C?', 's(', 'f(', 'picked Yellow?', 'pick Yellow!', ')', 'place at C!', ')', ')', ')']
    pytree = PyTree(tree[:], behaviors=behaviors)
    pytree.save_fig('logs/paperfigures/', name='croissant_planned', svg=True)

    # Croissant solved
    tree = ['s(', 'f(', 'Green at pos B?', 's(', 'pick Green!', 'place at B!', ')', ')', \
            'f(', 'Red at pos A?', 's(', 'pick Red!', 'place at A!', ')', ')', \
            'f(', 'Yellow at pos C?', 's(', 'pick Yellow!', 'place at C!', ')', ')', \
            'f(', 'Blue at pos D?', 's(', 'f(', 'Blue on Green?', 's(', 'pick Blue!', 'place on Green!', ')', ')', 'apply force Blue!', ')', ')', ')']
    pytree = PyTree(tree[:], behaviors=behaviors)
    pytree.save_fig('logs/paperfigures/', name='croissant_solved', svg=True)

    behavior_tree.load_settings_from_file('BT_SETTINGS_BALANCE.yaml')
    # Balance planned
    tree = ['s(', 'f(', 'Green at pos B?', 's(', 'f(', 'Blue at pos A?', 'put Blue at A!', ')', 'f(', 'Green on Blue?', 'put Green on Blue!', ')', 'apply force Green!', ')', ')', ')']
    pytree = PyTree(tree[:], behaviors=behaviors)
    pytree.save_fig('logs/paperfigures/', name='balance_planned', svg=True)

    # Balance solved
    tree = ['f(', 'Green at pos B?', 's(', 'put Red at A!', 'put Green on Red!', 'apply force Green!', ')', ')']
    pytree = PyTree(tree[:], behaviors=behaviors)
    pytree.save_fig('logs/paperfigures/', name='balance_solved', svg=True)

    behavior_tree.load_settings_from_file('BT_SETTINGS_BLOCKING.yaml')
    # Blocking planned
    tree = ['s(', 'f(', 'Green at A?', 'put Green at A!', ')', \
            'f(', 'Blue at E?', 's(', 'f(', 'Blue on Green?', 'put Blue on Green!', ')', 'apply force Blue!', ')', ')', \
            'f(', 'Red at C?', 'put Red at C!', ')', ')']

    pytree = PyTree(tree[:], behaviors=behaviors)
    pytree.save_fig('logs/paperfigures/', name='blocking_planned', svg=True)

    # Blocking solved
    tree = ['s(', 'put Blue on Green!', 'f(', 'Green at A?', 'put Red at D!', ')', \
            'put Green at A!', \
            'f(', 'Blue at E?', 'apply force Blue!', ')', \
            'put Red at C!', ')']
    pytree = PyTree(tree[:], behaviors=behaviors)
    pytree.save_fig('logs/paperfigures/', name='blocking_solved', svg=True)

if __name__ == "__main__":
    run_individual()
