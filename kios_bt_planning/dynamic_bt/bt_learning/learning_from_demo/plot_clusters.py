"""This module contains a function to plot the clusters generated for frame inference."""

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

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import hsv_to_rgb
import math
import sys
from typing import Any

import bt_learning.learning_from_demo.clustering as clustering
# Import this to allow 3D plot
from mpl_toolkits.mplot3d import axes3d  # noqa: F401

import logging
logging.getLogger('matplotlib').setLevel(logging.WARNING)


def plot_clusters(demo_dir: str, demonstration: Any, show: bool = False) -> None:
    """Plot the clusters of the learning framework."""
    try:
        partition = clustering.initial_partition(demonstration.all_actions())
    except AttributeError:
        print('The given <demonstration> object has no method "all_actions()".')
        sys.exit(1)

    try:
        n_frames = len(demonstration.frames)
    except AttributeError:
        print('The given <demonstration> object has no attribute "frames".')
        sys.exit(1)
    n_rows = math.floor(math.sqrt(n_frames))
    for i, group in enumerate(partition):
        fig = plt.figure(i)
        fig.suptitle(f'Action group {i}: {group[0].type}({group[0].parameters})')

        try:
            equivalent = clustering.cluster(
                group, demonstration.frames, 0, len(demonstration.demonstrations()))
        except AttributeError:
            print('The given <demonstration> object has no method "demonstrations()".')
            sys.exit(1)

        for j, frame in enumerate(demonstration.frames):
            ax = fig.add_subplot(n_rows, math.ceil(n_frames/n_rows), j+1, projection='3d')
            ax.set_title(frame)
            for action in group:
                # Find action
                equivalent_grouping = next(x for x in equivalent if action in x)
                idx = equivalent.index(equivalent_grouping)
                color = hsv_to_rgb(
                    [idx/len(equivalent), 1, idx/len(equivalent)+0.1]).reshape((1, 3))

                if frame == action.frame[0]:
                    marker = 'x'
                else:
                    marker = 'o'

                position = action.target_position(frame)
                scaled_position = [x*100 if x < 1 else x for x in position]
                unit = [' [cm]' if x < 1 else ' [m]' for x in position]
                # format the position values
                formatted_val = []
                for val in scaled_position:
                    formatted_val.append(float(np.format_float_positional(
                        val, precision=3, unique=False, fractional=False, trim='k')))
                # plot
                ax.scatter(scaled_position[0], scaled_position[1],
                           scaled_position[2], c=color, marker=marker)
                # Uncomment if it is desired to display axis labels. Not recommended.
                ax.set_xlabel('x' + unit[0], fontsize=9, labelpad=-18, color='r')
                ax.set_ylabel('y' + unit[0], fontsize=9, labelpad=-18, color='r')
                ax.set_zlabel('z' + unit[0], fontsize=9, labelpad=-18, color='r')
                # Uncomment if it is desired to remove the values of the samples
                # ax.set_xticklabels([])
                # ax.set_yticklabels([])
                # ax.set_zticklabels([])

        # set the spacing between subplots
        plt.subplots_adjust(left=0.1, bottom=0.1, right=0.9, top=0.9, wspace=0.6, hspace=0.4)

        format = 'svg'
        name = demo_dir + '/clusters_' + str(i) + '.' + format
        plt.savefig(name, format=format)

        if show:
            plt.show()

        plt.close()
