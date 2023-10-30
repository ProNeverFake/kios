"""
Unit test for logplot.py
"""
import os
import shutil
import behavior_tree_learning.logplot as logplot

def test_trim_logs():
    """ Tests trim_logs function """
    logs = []
    logs.append([1, 2, 3])
    logs.append([1, 2, 3, 4])
    logs.append([1, 2, 3, 4, 5])

    logplot.trim_logs(logs)

    assert logs == [[1, 2, 3], [1, 2, 3], [1, 2, 3]]

def test_plot_fitness():
    """ Tests plot_fitness function """
    logplot.clear_logs('test')
    logplot.plot_fitness('test', [0, 1, 2])
    assert os.path.isfile(logplot.get_log_folder('test') +  '/Fitness.png')
    try:
        shutil.rmtree(logplot.get_log_folder('test'))
    except FileNotFoundError:
        pass

def test_extend_gens():
    """ Tests plot of learning curves with extended gens """
    logplot.clear_logs('test1')
    logplot.log_best_fitness('test1', [1, 2, 3, 4, 5])
    logplot.log_n_episodes('test1', [5, 10, 15, 20, 25])
    logplot.clear_logs('test2')
    logplot.log_best_fitness('test2', [1, 2, 5])
    logplot.log_n_episodes('test2', [5, 10, 15])
    parameters = logplot.PlotParameters()
    parameters.path = 'behavior_tree_learning/tests/test.pdf'
    parameters.extend_gens = 5
    parameters.save_fig = True
    parameters.x_max = 30
    logplot.plot_learning_curves(['test1', 'test2'], parameters)

def test_plot_learning_curves():
    """ Tests plot_learning_curves function """
    try:
        os.remove('behavior_tree_learning/tests/test.pdf')
    except FileNotFoundError:
        pass

    logplot.clear_logs('test')
    logplot.log_best_fitness('test', [1, 2, 3, 4, 5])
    logplot.log_n_episodes('test', [5, 10, 15, 20, 25])

    parameters = logplot.PlotParameters()
    parameters.path = 'behavior_tree_learning/tests/test.pdf'
    parameters.extrapolate_y = False
    parameters.plot_mean = False
    parameters.plot_std = False
    parameters.plot_ind = False
    parameters.save_fig = False
    parameters.x_max = 0
    parameters.plot_horizontal = True
    logplot.plot_learning_curves(['test'], parameters)
    assert not os.path.isfile('behavior_tree_learning/tests/test.pdf')

    parameters.extrapolate_y = True
    parameters.plot_mean = True
    parameters.plot_std = True
    parameters.plot_ind = True
    parameters.save_fig = True
    parameters.x_max = 100
    parameters.plot_horizontal = True
    parameters.save_fig = True
    logplot.plot_learning_curves(['test'], parameters)
    assert os.path.isfile('behavior_tree_learning/tests/test.pdf')
    os.remove('behavior_tree_learning/tests/test.pdf')

    parameters.x_max = 10
    parameters.plot_horizontal = False
    logplot.plot_learning_curves(['test'], parameters)
    assert os.path.isfile('behavior_tree_learning/tests/test.pdf')

    os.remove('behavior_tree_learning/tests/test.pdf')
    try:
        shutil.rmtree(logplot.get_log_folder('test'))
    except FileNotFoundError:
        pass
