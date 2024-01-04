"""Unit test for logplot.py module."""

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

import os
import shutil

from bt_learning.gp import logplot


DIR_NAME = os.path.abspath(os.path.dirname(__file__))


def test_trim_logs():
    """Test trim_logs function."""
    logs = []
    logs.append([1, 2, 3])
    logs.append([1, 2, 3, 4])
    logs.append([1, 2, 3, 4, 5])

    logplot.trim_logs(logs)

    assert logs == [[1, 2, 3], [1, 2, 3], [1, 2, 3]]


def test_plot_fitness():
    """Test plot_fitness function."""
    logplot.clear_logs('test')
    logplot.plot_fitness('test', [0, 1, 2])
    assert os.path.isfile(logplot.get_log_folder('test') + '/Fitness.png')
    try:
        shutil.rmtree(logplot.get_log_folder('test'))
    except FileNotFoundError:
        pass


def test_extend_gens():
    """Test plot of learning curves with extended gens."""
    file_name = os.path.join(DIR_NAME, 'test.pdf')
    logplot.clear_logs('test1')
    logplot.log_best_fitness('test1', [1, 2, 3, 4, 5])
    logplot.log_n_episodes('test1', [5, 10, 15, 20, 25])
    logplot.clear_logs('test2')
    logplot.log_best_fitness('test2', [1, 2, 5])
    logplot.log_n_episodes('test2', [5, 10, 15])
    parameters = logplot.PlotParameters()
    parameters.path = file_name
    parameters.extend_gens = 5
    parameters.save_fig = True
    parameters.x_max = 30
    logplot.plot_learning_curves(['test1', 'test2'], parameters)


def test_plot_learning_curves():
    """Test plot_learning_curves function."""
    file_name = os.path.join(DIR_NAME, 'test.pdf')
    try:
        os.remove(file_name)
    except FileNotFoundError:
        pass

    logplot.clear_logs('test')
    logplot.log_best_fitness('test', [1, 2, 3, 4, 5])
    logplot.log_n_episodes('test', [5, 10, 15, 20, 25])

    parameters = logplot.PlotParameters()
    parameters.path = file_name
    parameters.extrapolate_y = False
    parameters.plot_mean = False
    parameters.plot_std = False
    parameters.plot_ind = False
    parameters.save_fig = False
    parameters.x_max = 0
    parameters.plot_horizontal = True
    logplot.plot_learning_curves(['test'], parameters)
    assert not os.path.isfile(file_name)

    parameters.extrapolate_y = True
    parameters.plot_mean = True
    parameters.plot_std = True
    parameters.plot_ind = True
    parameters.save_fig = True
    parameters.plot_minmax = True
    parameters.x_max = 100
    parameters.y_max = 100
    parameters.y_min = 10
    parameters.logarithmic_y = True
    parameters.plot_horizontal = True
    parameters.save_fig = True
    logplot.plot_learning_curves(['test'], parameters)
    assert os.path.isfile(file_name)
    os.remove(file_name)

    parameters.x_max = 10
    parameters.plot_horizontal = False
    logplot.plot_learning_curves(['test'], parameters)
    assert os.path.isfile(file_name)

    os.remove(file_name)
    try:
        shutil.rmtree(logplot.get_log_folder('test'))
    except FileNotFoundError:
        pass


def test_best_individual():
    """Test logging and loading a best individual."""
    best_individual = ['a']
    logplot.clear_logs('test')
    logplot.log_best_individual('test', best_individual)

    loaded_best_individual = logplot.get_best_individual('test')
    assert best_individual == loaded_best_individual
