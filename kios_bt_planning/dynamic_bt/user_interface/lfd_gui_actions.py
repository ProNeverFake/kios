"""Enable robot actions through the GUI."""

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

from copy import copy
import logging
import os
from typing import Any, List

# OS dependent import
if os.name == 'nt':  # Windows
    import PySimpleGUIQt as gui
elif os.name == 'posix':  # Linux Ubuntu
    import PySimpleGUI as gui

logging.basicConfig(level=logging.NOTSET)


class GUIActions():

    def __init__(self, title: str, action_list: List[str], button_length: int = 15):
        self.title = title
        self.action_list = action_list

        if os.name == 'nt':  # Windows
            self.button_length = None
        elif os.name == 'posix':  # Linux Ubuntu
            self.button_length = button_length

        self.layout = []
        self.window = None

    def get_layout(self) -> List[List[gui.Button]]:
        """Return the layout of this window."""
        return self.layout

    def get_window(self) -> gui.Window:
        """Return the interface as window."""
        self.window = gui.Window(self.title, self.layout)
        return self.window

    def add_action_button(self, action_name: str, tooltip: str = None) -> None:
        """Add a custom action to the GUI."""
        action_key = action_name.lower()
        action_key = '__' + action_key + '__'
        self.layout.append(
            [gui.Button(
                button_text=action_name,
                tooltip=tooltip,
                size=(self.button_length, None),
                key=str(action_key)
            )]
        )

    def initialize_layout(self) -> None:
        """Define the layout for the GUI window where the user can select the actions."""
        actions_to_add = copy(self.action_list)
        if 'pick' in actions_to_add:
            self.layout.append(
                [gui.Button(
                    'Pick',
                    tooltip='Close the grippers around the target object.',
                    size=(self.button_length, None),
                    key='__pick__'
                )]
            )
            actions_to_add.pop(actions_to_add.index('pick'))
        if 'place' in actions_to_add:
            self.layout.append(
                [gui.Button(
                    'Fine Place',
                    tooltip='Open the grippers. Use when the target position has to be precise.',
                    size=(self.button_length, None),
                    key='__place__'
                )]
            )
            actions_to_add.pop(actions_to_add.index('place'))
        if 'drop' in actions_to_add:
            self.layout.append(
                [gui.Button(
                    'Drop',
                    tooltip='Open the grippers. Use when the target position does not matter much.',
                    size=(self.button_length, None),
                    key='__drop__'
                )]
            )
            actions_to_add.pop(actions_to_add.index('drop'))

        for action in actions_to_add:
            action_key = '__' + action + '__'
            self.layout.append(
                [gui.Button(
                    action.capitalize(),
                    tooltip=f'Perform the {action} action.',
                    size=(self.button_length, None),
                    key=action_key
                )]
            )
            actions_to_add.pop(actions_to_add.index(action))

    def execute_action(self, event: str, action_function: Any, n_demos: int, *kwargs: Any) -> None:
        """Map the button to the robot action."""
        if event == '__pick__':
            action_function(n_demos, action_name='pick', gripper_state='closed')
        elif event == '__place__':
            action_function(n_demos, action_name='place', gripper_state='open')
        elif event == '__drop__':
            action_function(n_demos, action_name='drop', gripper_state='open')
        elif event is None:
            pass
        else:
            action_function(n_demos, action_name=event[2:-2], *kwargs)

        # TODO: add MoveBase buttons
