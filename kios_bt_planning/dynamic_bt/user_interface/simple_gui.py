"""Common GUI functionalities."""

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

# OS dependent import
if os.name == 'nt':  # Windows
    import PySimpleGUIQt as gui
elif os.name == 'posix':  # Linux Ubuntu
    import PySimpleGUI as gui


class SimpleGUI():
    """Meta class to handle more easily the Simple GUI layout."""

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

    def get_layout(self) -> List[List[gui.Button]]:
        """Return the layout of this window."""
        return self.layout

    def add_button(
        self,
        label: str,
        tooltip: str,
        disabled: bool,
        key: str,
        size: int = None,
        row: int = None
    ) -> None:
        """Add a button to the layout."""
        if size is None or size > self.button_space:
            size = self.button_space

        if row == None:
            self.add_row()
            row = -1

        self.layout[row].append(
            gui.Button(
                button_text=label,
                tooltip=tooltip,
                disabled=disabled,
                size=(size, None),
                key=key
            )
        )

    def add_row(self) -> None:
        """Add a row to the layout."""
        self.layout.append([])

    def reshape(self, shape: List[int], button_idx: int) -> None:
        """
        Reshape the buttons as in the given input shape.

        Args
        ----
            - shape: list where its size indicates the number of rows and every element is
                     the number of buttons in that row.
            - button_idx: index of the layout where the buttons are placed.
        """
        if type(sum(shape)) != int:
            raise TypeError('The shape has not the correct data type: it requires INT.')

        buttons = []
        for row in self.layout[button_idx:]:
            for item in row:
                if type(item) is gui.Button:
                    buttons.append(item)

        self.layout = self.layout[:button_idx]

        rows = len(shape)
        for i in range(rows):
            self.add_row()
            button_size = (self.button_space - (shape[i] - 1)*self.space)//(shape[i])
            for item in buttons[:shape[i]]:
                item.Size = (button_size, None)
                self.layout[-1].append(item)
            # Remove appended buttons to from the list of buttons.
            buttons = buttons[shape[i]:]

    def hide_button(self, window: gui.Window, key: str) -> None:
        """Hide a button through its key."""
        window.read(timeout=0)
        window.find_element(key).update(visible=False)

    def disable_button(self, window: gui.Window, key: str) -> None:
        """Deactivate a button through its key."""
        window.read(timeout=0)
        window.find_element(key).update(disabled=True)

    def enable_button(self, window: gui.Window, key: str) -> None:
        """Activate a button through its key."""
        window.read(timeout=0)
        window.find_element(key).update(disabled=False)
