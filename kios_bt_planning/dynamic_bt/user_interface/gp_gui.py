"""
Main node launching the GUI for the LfD framework.

In this node:
 - the Mobile Base is NOT enabled.
 - the perception uses MaskRCNN for object detection.
 - it is possible to enable HRI to disambiguate the scene.
"""

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
from typing import List

from user_interface.simple_gui import SimpleGUI

# OS dependent import
if os.name == 'nt':  # Windows
    import PySimpleGUIQt as gui
elif os.name == 'posix':  # Linux Ubuntu
    import PySimpleGUI as gui


class GPGUILayout(SimpleGUI):
    """Meta class to handle more easily the Simple GUI layout."""

    def __init__(self, title: str):
        """Initialize some internal parameters."""
        super().__init__(title)

    def initialize_layout(self):
        """Initialize the layout with the standard LfD functionalities."""
        self.layout = [
            [
                gui.Input(
                    'No folder selected',
                    key='__folder_display__',
                    enable_events=True,
                    disabled=True,
                    size=(self.size_parameter, None)
                ),
                gui.FolderBrowse(
                    'Select log folder',
                    target='__folder_display__',
                    key='__log_folder__'
                )
            ],
            [gui.Text(
                'Current generation:', key='__gen__', size=(self.size_parameter, None)
            )],
            [gui.Text('Best fitness score:', key='__fitness__', size=(self.size_parameter, None))],
            [gui.ProgressBar(20, size=(52, 3), key='__progress__', visible=False)],
            [
                gui.Button(
                    'Start GP',
                    tooltip='Start the evolution process with Genetic Programming.',
                    size=(None, None),
                    key='__start__'
                ),
                gui.Button(
                    'Run tree',
                    tooltip='Run the best Behavior Tree.',
                    disabled=True,
                    size=(None, None),
                    key='__run__'
                ),
                gui.Button(
                    'Show tree',
                    tooltip='Display the best Behavior Tree.',
                    disabled=True,
                    size=(None, None),
                    key='__show_tree__'
                ),
            ]
        ]

        self.reshape([3])

    def reshape(self, shape: List[int]) -> None:
        return super().reshape(shape, 4)
