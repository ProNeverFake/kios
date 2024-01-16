"""Plot logs from the GP approach."""

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

from bt_learning.gp import logplot


def main() -> None:
    batch_size = 10

    # first plot
    params = logplot.PlotParameters()
    params.label = 'GP with LfD solution'
    params.xlabel_fontsize = 12
    params.save_fig = False
    params.mean_color = 'b'
    params.std_color = 'b'
    params.x_max = 2300
    params.extrapolate_y = True
    params.plot_horizontal = False
    logs = [str(x + 1) for x in range(batch_size)]
    logplot.plot_learning_curves(logs, params)

    # second plot
    params = logplot.PlotParameters()
    params.title = 'Fitness score for Experiment 2'
    params.title_fontsize = 18
    params.label = 'GP standalone'
    params.xlabel_fontsize = 12
    params.xlabel = 'Learning Episodes'
    params.legend_fontsize = 14
    params.save_fig = True
    params.mean_color = 'r'
    params.std_color = 'r'
    params.x_max = 2300
    params.extrapolate_y = True
    params.plot_horizontal = False
    logs = [str(x + 1 + batch_size) for x in range(batch_size)]
    logplot.plot_learning_curves(logs, params)


if __name__ == '__main__':
    main()
