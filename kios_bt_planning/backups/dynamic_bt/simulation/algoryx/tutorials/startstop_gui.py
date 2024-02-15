"""
GUI to test start - restart - reset functionalities.

Use with: python tutorials/startstop_gui.py --model models/BTDemo.yml:CoarseGripperInLabDR --decorate --timeStep 0.05
from the /algoryx folder
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

import logging
import os
import time
from typing import Any, Dict, List

from agxBrick.brickLoaderUtils import createArgumentParser
from agxClick import AgxApplication
from simulation.algoryx.agx_application import Application as App
import yaml


# OS dependent import
if os.name == 'nt':  # Windows
    import PySimpleGUIQt as gui
elif os.name == 'posix':  # Linux Ubuntu
    import PySimpleGUI as gui


class SandS_GUI():

    def __init__(self, args: Any):
        self._logger = logging.getLogger(__file__)

        repo_path = os.path.dirname(os.path.dirname((os.path.abspath(__file__))))
        with open(os.path.join(repo_path, 'config/sim_objects.yaml')) as f:
            obj_data = yaml.safe_load(f)

        self._env = None
        self._app = None
        self.args = args
        self.objects = obj_data

        self.gui = Layout('Start and Stop the AGX Simulation')

    def show(self):
        """Define the GUI functionalities."""
        self.gui.initialize_layout()

        window = self.gui.get_window()

        # Interaction with the GUI Main menu
        ret = 0
        while ret is not None:
            ret, values = window.read()

            if ret == '__start__':
                self.bringup()

            elif ret == '__close__':
                self.shutdown()

            elif ret == '__restart__':
                self.shutdown()
                time.sleep(3)
                self.bringup()

            elif ret == '__reset__':
                self._app.reset()

    def bringup(self):
        self._app = App(self.args)
        self._app.bringup(self.objects, visual=True)

    def shutdown(self):
        self._app.shutdown()
        del self._app
        self._app = None
        time.sleep(3)
        print('Application DOWN!')


class Layout():

    def __init__(self, title: str):
        """Initialize some internal parameters."""
        self.title = title

        self.window = None
        self.layout = []

        if os.name == 'nt':  # Windows
            self.size_parameter = 500
        elif os.name == 'posix':  # Linux Ubuntu
            self.size_parameter = 60

        self.button = 36
        self.space = 5
        self.button_space = self.button*2 + self.space

    def get_window(self) -> gui.Window:
        """Return the interface as window."""
        self.window = gui.Window(self.title, self.layout)
        return self.window

    def initialize_layout(self):
        """Initialize the layout with the standard LfD functionalities."""
        self.layout = [
            [
                gui.Button(
                    'Start AGX',
                    disabled=False,
                    size=(None, None),
                    key='__start__'
                ),
                gui.Button(
                    'Close AGX',
                    disabled=False,
                    size=(None, None),
                    key='__close__'
                ),
            ],
            [
                gui.Button(
                    'Reset Simulation',
                    disabled=False,
                    size=(None, None),
                    key='__reset__'
                ),
                gui.Button(
                    'Restart AGX',
                    disabled=False,
                    size=(None, None),
                    key='__restart__'
                ),
            ]
        ]
        self.reshape([2, 2])

    def reshape(self, shape: List[int]) -> None:
        """Reshape the buttons as in the given input shape."""
        if type(sum(shape)) != int:
            raise TypeError('The shape has not the correct data type: it requires INT.')

        buttons = []
        for row in self.layout:
            for item in row:
                if type(item) is gui.Button:
                    buttons.append(item)

        self.layout = []
        rows = len(shape)
        for i in range(rows):
            self.layout.append([])
            button_size = (self.button_space - (shape[i] - 1)*self.space)//(shape[i])
            for item in buttons[:shape[i]]:
                item.Size = (button_size, None)
                self.layout[-1].append(item)
            # Remove appended buttons to from the list of buttons.
            buttons = buttons[shape[i]:]


def main():
    parser, args, leftover_args = createArgumentParser()

    gui = SandS_GUI(args)
    gui.show()


if __name__ == '__main__':
    main()
