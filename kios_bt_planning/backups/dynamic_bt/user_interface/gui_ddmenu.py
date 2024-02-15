"""Define a dropdown menu in the GUI."""

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
from typing import List

# OS dependent import
if os.name == 'nt':  # Windows
    import PySimpleGUIQt as gui
elif os.name == 'posix':  # Linux Ubuntu
    import PySimpleGUI as gui

logging.basicConfig(level=logging.NOTSET)


class GUIMenu():

    def __init__(
        self,
        title: str = 'Simple Menu',
        option_list: List[str] = ['1', '2'],
        length: int = 15
    ):
        self.title = title
        self.options = option_list

        if os.name == 'nt':  # Windows
            self.length = None
        elif os.name == 'posix':  # Linux Ubuntu
            self.length = length

        self.layout = []
        self.window = None

    def set_title(self, title: str) -> None:
        """Set the title for the menu."""
        self.title = title

    def set_options(self, option_list: List[str]) -> None:
        """Set the options to choose from in the menu."""
        self.options = option_list

    def get_layout(self) -> List[List[gui.Button]]:
        """Return the layout of this window."""
        return self.layout

    def get_window(self) -> gui.Window:
        """Return the interface as window."""
        self.window = gui.Window('Options', self.layout)
        return self.window

    def initialize_layout(self) -> str:
        """Define the layout for the GUI window where the user can select some options."""
        menu_def = ['Options', [self.options]]

        key_ = '__menu__'

        self.layout.append(
            [gui.ButtonMenu(
                self.title,
                menu_def=menu_def,
                border_width=self.length,
                key=key_
            )]
        )

        return key_
