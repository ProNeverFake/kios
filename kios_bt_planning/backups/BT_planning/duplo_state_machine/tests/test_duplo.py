# pylint: disable=too-many-statements, import-outside-toplevel
"""
Test duplo setup
"""
import random
import pytest
import behavior_tree_learning.genetic_programming as gp
import behavior_tree_learning.behavior_tree as behavior_tree
import duplo_state_machine.environment
from duplo_state_machine.state_machine import Pos

def test_duplo_tower():
    """ Tests state machine for duplo tower scenario """
    behavior_tree.load_settings_from_file('duplo_state_machine/BT_SETTINGS_TOWER.yaml')
    start_positions = []
    start_positions.append(Pos(-0.05, -0.1, 0))
    start_positions.append(Pos(0, -0.1, 0))
    start_positions.append(Pos(0.05, -0.1, 0))
    targets = []
    targets.append(Pos(0.0, 0.05, 0))
    targets.append(Pos(0.0, 0.05, 0.0192))
    targets.append(Pos(0.0, 0.05, 2*0.0192))
    environment = duplo_state_machine.environment.Environment(start_positions, targets)

    print(environment.get_fitness( \
        ['s(', 'f(', '0 at pos (0.0, 0.05, 0.0)?', \
                     's(', 'pick 0!', 'place at (0.0, 0.05, 0.0)!', ')', ')', \
               'f(', '1 at pos (0.0, 0.05, 0.0192)?', \
                     's(', 'pick 1!', 'place on 0!', ')', ')', \
         ')']))

    print(environment.get_fitness(
        ['s(', 'f(', '0 at pos (0.0, 0.05, 0.0)?', \
                     's(', 'pick 0!', 'place at (0.0, 0.05, 0.0)!', ')', ')', \
               'f(', '1 at pos (0.0, 0.05, 0.0192)?', \
                     's(', 'f(', '1 on 0?', 's(', 'pick 1!', 'place on 0!', ')', ')', 'apply force 1!', ')', ')', \
         ')']))

    environment.plot_individual('', 'test', \
                                ['s(', 'f(', '0 at pos (0.0, 0.05, 0.0)?', \
                                             's(', 'pick 0!', 'place at (0.0, 0.05, 0.0)!', ')', ')', \
                                       'f(', '1 at pos (0.0, 0.05, 0.0192)?', \
                                             's(', 'f(', '1 on 0?', 's(', 'pick 1!', 'place on 0!', ')', ')', \
                                                   'apply force 1!', ')', ')', \
                                 ')'])

    environment.plot_individual('', 'test2', \
                                ['s(', 'f(', '0 at pos (0.0, 0.05, 0.0)?', \
                                             's(', 'f(', 'picked 0?', 'pick 0!', ')', \
                                                   'place at (0.0, 0.05, 0.0)!', ')', ')', \
                                       'f(', '1 at pos (0.0, 0.05, 0.0192)?', \
                                             's(', 'f(', 'picked 1?', 'pick 1!', ')', \
                                                   'place on 0!', ')', ')', \
                                       'f(', '2 at pos (0.0, 0.05, 0.0384)?', \
                                             's(', 'f(', 'picked 2?', 'pick 2!', ')', \
                                                   'place on 1!', ')', ')', ')'])

    ind = ['s(', 'f(', '0 at pos (0.0, 0.05, 0.0)?', \
                       's(', 'f(', 'picked 0?', 'pick 0!', ')', 'place at (0.0, 0.05, 0.0)!', ')', ')', \
                 'f(', '1 at pos (0.0, 0.05, 0.0192)?', \
                       's(', 'f(', 'picked 1?', 'pick 1!', ')', 'place on 0!', ')', ')', \
                 'f(', '2 at pos (0.0, 0.05, 0.0384)?', \
                       's(', 'f(', 'picked 2?', 'pick 2!', ')', 'place on 1!', ')', ')', ')']
    print_and_plot(environment, ind, 'at no force')

    ind = ['s(', 'f(', '0 at pos (0.0, 0.05, 0.0)?', \
                       's(', 'f(', 'picked 0?', 'pick 0!', ')', 'place at (0.0, 0.05, 0.0)!', ')', ')', \
                 'f(', '1 on 0?', 's(', 'f(', 'picked 1?', 'pick 1!', ')', 'place on 0!', ')', ')', \
                 'f(', '2 on 1', 's(', 'f(', 'picked 2?', 'pick 2!', ')', 'place on 1!', ')', ')', ')']
    print_and_plot(environment, ind, 'on no force')

    ind = ['s(', 'f(', '0 at pos (0.0, 0.05, 0.0)?', \
                       's(', 'f(', 'picked 0?', 'pick 0!', ')', 'place at (0.0, 0.05, 0.0)!', ')', ')', \
                 'f(', '1 at pos (0.0, 0.05, 0.0192)?', \
                       's(', 'f(', '1 on 0?', 's(', 'f(', 'picked 1?', 'pick 1!', ')', 'place on 0!', ')', ')', \
                             'apply force 1!', ')', ')', \
                 'f(', '2 at pos (0.0, 0.05, 0.0384)?', \
                       's(', 'f(', '2 on 1?', 's(', 'f(', 'picked 2?', 'pick 2!', ')', 'place on 1!', ')', ')', \
                             'apply force 2!', ')', ')', ')']
    print_and_plot(environment, ind, 'on with force')

    for i in range(10):
        random.seed(i)
        ind = ['s(', 'f(', '0 at pos (0.0, 0.05, 0.0)?', 's(', 'pick 0!', 'place at (0.0, 0.05, 0.0)!', ')', ')', \
                     'f(', '1 at pos (0.0, 0.05, 0.0192)?', \
                           's(', 'f(', '1 on 0?', 's(', 'pick 1!', 'place on 0!', ')', ')', \
                                 'apply force 1!', ')', ')', \
                     'f(', '2 on 1?', 's(', 'pick 2!', 'place on 1!', ')', ')', 'apply force 2!', ')']
        print_and_plot(environment, ind, 'optimal')

    ind = ['s(', 'f(', '0 at pos (0.0, 0.05, 0.0)?', \
                       's(', 'f(', 'picked 0?', 'pick 0!', ')', 'place at (0.0, 0.05, 0.0)!', ')', ')', \
                 'f(', '1 at pos (0.0, 0.05, 0.0192)?', \
                       's(', 'f(', '1 on 0?', 's(', 'f(', 'picked 1?', 'pick 1!', ')', 'place on 0!', ')', ')', \
                             'apply force 1!', ')', ')', \
                 'f(', '2 at pos (0.0, 0.05, 0.0384)?', \
                       's(', 'f(', '2 on 1?', 's(', 'f(', 'picked 2?', 'pick 2!', ')', 'place on 1!', ')', ')', \
                             'apply force 2!', ')', ')', ')']
    print_and_plot(environment, ind, 'full planned')

def test_duplo_croissant():
    """ Tests state machine for duplo croissant scenario """
    behavior_tree.load_settings_from_file('duplo_state_machine/BT_SETTINGS_CROISSANT.yaml')
    start_positions = []
    start_positions.append(Pos(-0.05, -0.1, 0))
    start_positions.append(Pos(0, -0.1, 0))
    start_positions.append(Pos(0.05, -0.1, 0))
    start_positions.append(Pos(0.1, -0.1, 0))
    targets = []
    targets.append(Pos(0.0, 0.0, 0.0))
    targets.append(Pos(0.0, 0.0, 0.0192))
    targets.append(Pos(0.016, -0.032, 0.0))
    targets.append(Pos(0.016, 0.032, 0.0))
    environment = duplo_state_machine.environment.Environment(start_positions, targets)

    best = ['s(', 'f(', '0 at pos (0.0, 0.0, 0.0)?', 's(', 'pick 0!', 'place at (0.0, 0.0, 0.0)!', ')', ')', \
                  'f(', '2 at pos (0.016, -0.032, 0.0)?', 's(', 'pick 2!', 'place at (0.016, -0.032, 0.0)!', ')', ')', \
                  'f(', '3 at pos (0.016, 0.032, 0.0)?', 's(', 'pick 3!', 'place at (0.016, 0.032, 0.0)!', ')', ')', \
                  'f(', '1 at pos (0.0, 0.0, 0.0192)?', \
                        's(', 'f(', '1 on 0?', 's(', 'pick 1!', 'place on 0!', ')', ')', \
                              'apply force 1!', ')', ')', ')']

    print(environment.get_fitness(best))

    planned = ['s(', 'f(', '0 at pos (0.0, 0.0, 0.0)?', 's(', 'pick 0!', 'place at (0.0, 0.0, 0.0)!', ')', ')', \
                     'f(', '1 at pos (0.0, 0.0, 0.0192)?', \
                           's(', 'f(', '1 on 0?', 's(', 'pick 1!', 'place on 0!', ')', ')', \
                                 'apply force 1!', ')', ')', \
                     'f(', '2 at pos (0.016, -0.032, 0.0)?', \
                           's(', 'pick 2!', 'place at (0.016, -0.032, 0.0)!', ')', ')', \
                     'f(', '3 at pos (0.016, 0.032, 0.0)?', \
                           's(', 'pick 3!', 'place at (0.016, 0.032, 0.0)!', ')', ')', ')']

    planned = ['s(', 'f(', '0 at pos (0.0, 0.0, 0.0)?', \
                           's(', 'f(', 'picked 0?', 'pick 0!', ')', 'place at (0.0, 0.0, 0.0)!', ')', ')', \
                     'f(', '1 at pos (0.0, 0.0, 0.0192)?', \
                           's(', 'f(', '1 on 0?', 's(', 'f(', 'picked 1?', 'pick 1!', ')', 'place on 0!', ')', ')', \
                                 'apply force 1!', ')', ')', \
                     'f(', '2 at pos (0.016, -0.032, 0.0)?', \
                           's(', 'f(', 'picked 2?', 'pick 2!', ')', 'place at (0.016, -0.032, 0.0)!', ')', ')', \
                     'f(', '3 at pos (0.016, 0.032, 0.0)?', \
                           's(', 'f(', 'picked 3?', 'pick 3!', ')', 'place at (0.016, 0.032, 0.0)!', ')', ')', ')']

    print(environment.get_fitness(planned))

    gp_par = gp.GpParameters()
    gp_par.ind_start_length = 8
    gp_par.n_population = 16
    gp_par.f_crossover = 0.5
    gp_par.n_offspring_crossover = 2
    gp_par.f_mutation = 0.5
    gp_par.n_offspring_mutation = 2
    gp_par.parent_selection = gp.SelectionMethods.RANK
    gp_par.survivor_selection = gp.SelectionMethods.RANK
    gp_par.f_elites = 0.1
    gp_par.f_parents = 1
    gp_par.mutate_co_offspring = False
    gp_par.mutate_co_parents = True
    gp_par.mutation_p_add = 0.4
    gp_par.mutation_p_delete = 0.3
    gp_par.allow_identical = False
    gp_par.plot = True
    gp_par.n_generations = 50
    gp_par.verbose = False
    gp_par.fig_last_gen = False

    n_logs = 3
    for i in range(1, n_logs + 1):
        gp_par.log_name = 'croissant_baseline_sm_' + str(i)
        gp.set_seeds(i)
        gp.run(environment, gp_par, baseline=planned)

def print_and_plot(environment, bt, name):
    """ Help function to print and plot bt """
    environment.plot_individual('logs/', name, bt)

    print(name + ' ' + str(environment.get_fitness(bt)))

@pytest.mark.skip
def test_check_profile():
    """
    Profiles the state machine environment
    """
    import cProfile
    cProfile.runctx('test_other()', globals=globals(), locals=locals())
