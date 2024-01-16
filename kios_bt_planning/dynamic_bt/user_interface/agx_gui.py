"""Main node launching the GUI for the GP+LfD framework."""

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


class AGXGUILayout(SimpleGUI):
    """Meta class to handle more easily the Simple GUI layout."""

    def __init__(self, title: str, root_folder: str = None) -> None:
        """Initialize some internal parameters."""
        super().__init__(title)

        self.root_folder = root_folder
        self.keys = []

    def initialize_layout(self) -> None:
        """Initialize the layout with the standard LfD functionalities."""
        self.layout = [
            [
                gui.Input(
                    'Select a folder to load past runs...',
                    key='__folder_display__',
                    enable_events=True,
                    disabled=True,
                    size=(self.size_parameter, None)
                ),
                gui.FolderBrowse(
                    'Select log folder',
                    target='__folder_display__',
                    initial_folder=self.root_folder,
                    key='__log_folder__'
                )
            ],
            [
                gui.Frame(
                    ' Simulation Environment ',
                    self.get_agx_layout(),
                    expand_x=True,
                    expand_y=True,
                    title_location=gui.TITLE_LOCATION_TOP
                )
            ],
            [
                gui.Frame(
                    ' Behavior Tree ',
                    self.get_bt_layout(),
                    expand_x=True,
                    expand_y=True,
                    title_location=gui.TITLE_LOCATION_TOP
                )
            ],
            [
                gui.Frame(
                    ' Genetic Programming ',
                    self.get_gp_layout(),
                    expand_x=True,
                    expand_y=True,
                    title_location=gui.TITLE_LOCATION_TOP
                )
            ],
            [
                gui.Frame(
                    ' Learning from Demonstration ',
                    self.get_lfd_layout(),
                    expand_x=True,
                    expand_y=True,
                    title_location=gui.TITLE_LOCATION_TOP
                )
            ]
        ]

        self.keys += ['__log_folder__']

    def get_agx_layout(self) -> List[List[gui.Element]]:
        """Layout for BT related stuff."""
        agx_layout = [
            [
                gui.Button(
                    'Initial Configuration',
                    tooltip='Show the initial configuration of the simulation environment.',
                    disabled=False,
                    size=(None, None),
                    key='__initial__'
                ),
                gui.Button(
                    'Target Configuration',
                    tooltip='Show the final configuration of the simulation environment.',
                    disabled=False,
                    size=(None, None),
                    key='__target__'
                ),
            ]
        ]
        self.keys += ['__initial__', '__target__']
        self.reshape(agx_layout, [2])
        return agx_layout

    def get_bt_layout(self) -> List[List[gui.Element]]:
        """Layout for BT related stuff."""
        bt_layout = [
            [
                gui.Button(
                    'Run',
                    tooltip='Run the loaded Behavior Tree and execute the learnt task.',
                    disabled=True,
                    size=(None, None),
                    key='__run_tree__'
                ),
                gui.Button(
                    'Show',
                    tooltip='Display the Behavior Tree.',
                    disabled=True,
                    size=(None, None),
                    key='__show_tree__'
                ),
                gui.Button(
                    'Save',
                    tooltip='Save the Behavior Tree for future reference.',
                    disabled=True,
                    size=(None, None),
                    key='__save_tree__'
                ),
            ]
        ]
        self.keys += ['__run_tree__', '__show_tree__', '__save_tree__']
        self.reshape(bt_layout, [3])
        return bt_layout

    def get_gp_layout(self) -> List[List[gui.Element]]:
        """Layout for GP related stuff."""
        gp_layout = [
            [gui.Text(
                'Current generation:', key='__gen__', size=(self.size_parameter, None)
            )],
            [gui.Text('Best fitness score:', key='__fitness__', size=(self.size_parameter, None))],
            [
                gui.Button(
                    'Start',
                    tooltip='Start the evolution process with Genetic Programming.',
                    size=(None, None),
                    key='__start__'
                ),
                gui.Button(
                    'Plot Fitness',
                    tooltip='Plot the fitness function.',
                    disabled=True,
                    size=(None, None),
                    key='__plot_fitness__'
                ),
                gui.Button(
                    'Increase Generations',
                    tooltip='Increase the number of generations by the default value.',
                    disabled=True,
                    size=(None, None),
                    key='__increase__'
                ),
            ]
        ]
        self.keys += ['__start__', '__plot_fitness__', '__increase__']
        self.reshape(gp_layout[2:], [3])
        return gp_layout

    def get_lfd_layout(self) -> List[List[gui.Element]]:
        """Layout for LfD related stuff."""
        lfd_layout = [
            [gui.Text(
                'Number of demonstrations:', key='__n_demos__', size=(self.size_parameter, None)
            )],
            [gui.Text('Number of nodes:', key='__n_nodes__', size=(self.size_parameter, None))],
            [gui.ProgressBar(20, size=(52, 3), key='__progress__', visible=False)],
            [
                gui.Button(
                    'Add Demonstration',
                    tooltip='Teach the robot a new task by guiding its arm.',
                    disabled=False,
                    size=(None, None),
                    key='__new_demo__'
                ),
                gui.Button(
                    'Add Target',
                    tooltip='Demonstrate a new step in the task from the current taget configuration.',
                    disabled=False,
                    size=(None, None),
                    key='__new_target__'
                ),
                gui.Button(
                    'Plot Clusters',
                    tooltip='Plot the clusters for frame inference.',
                    disabled=True,
                    size=(None, None),
                    key='__cluster__'
                ),
            ]
        ]
        self.keys += ['__new_demo__', '__new_target__', '__cluster__']
        self.reshape(lfd_layout[3:], [3])
        return lfd_layout

    def reshape(self, layout: List[List[gui.Element]], shape: List[int]) -> List[List[gui.Element]]:
        """
        Reshape the buttons as in the given input shape.

        Args
        ----
            - shape: list where its size indicates the number of rows and every element is
                     the number of buttons in that row.
        """
        if type(sum(shape)) != int:
            raise TypeError('The shape has not the correct data type: it requires INT.')

        buttons = []
        for row in layout:
            for item in row:
                if type(item) is gui.Button:
                    buttons.append(item)

        new_layout = []
        rows = len(shape)
        for i in range(rows):
            new_layout.append([])
            button_size = (self.button_space - (shape[i] - 1)*self.space)//(shape[i])
            for item in buttons[:shape[i]]:
                item.Size = (button_size, None)
                new_layout[-1].append(item)
            # Remove appended buttons to from the list of buttons.
            buttons = buttons[shape[i]:]

        return new_layout

    def disable_all_but(self, window: gui.Window, key: str) -> None:
        """Disable some buttons if the GUI is run offline."""
        window.read(timeout=0)
        for val in self.keys:
            if val == key:
                continue
            window.find_element(val).update(disabled=True)

    def enable_all_but(self, window: gui.Window, key: str = None) -> None:
        """Disable some buttons if the GUI is run offline."""
        window.read(timeout=0)
        for val in self.keys:
            if val == key:
                continue
            window.find_element(val).update(disabled=False)

    def enable_from_flags(
        self,
        window: gui.Window,
        has_BT: bool,
        has_GP: bool,
        has_Demo: bool
    ) -> None:
        """Disable some buttons if the GUI is run offline."""
        window.read(timeout=0)
        self.enable_all_but(window)
        if not has_BT:
            window.find_element('__run_tree__').update(disabled=True)
            window.find_element('__show_tree__').update(disabled=True)
            window.find_element('__save_tree__').update(disabled=True)
        if not has_GP:
            window.find_element('__plot_fitness__').update(disabled=True)
            window.find_element('__increase__').update(disabled=True)
        if not has_Demo:
            window.find_element('__cluster__').update(disabled=True)
