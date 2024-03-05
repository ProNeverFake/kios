"""Class that renders the BT in a web browser and allows ticking the tree manually."""

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
import tempfile
import threading
import webbrowser

from bt_learning.learning_from_demo.render_tree import dot_graph
import py_trees as pt


class BTVisualizer:
    """Render the BT in a web browser and allows ticking the tree manually."""

    # This works only in Windows!!
    CHROME_PATH = r'C:\Program Files\Google\Chrome\Application\chrome.exe'
    DISPLAY_HTML = '<!DOCTYPE html>\
                        <html>\
                        <head>\
                            <meta charset="utf-8" />\
                            <title>DEBUG BT</title>\
                            <script language="JavaScript">\
                                function refreshIt(element) {\
                                    setTimeout(function () {\
                                        element.src = element.src.split("?")[0] + "?" +\
                                        new Date().getTime();\
                                    }, 100);\
                                }\
                            </script>\
                        </head>\
                        <body>\
                            <img src="tree.svg" onload="refreshIt(this)"/>\
                        </body>\
                        </html>'

    def __init__(self, tree):
        if isinstance(tree, pt.trees.BehaviourTree):
            self.tree = tree.root
        else:
            self.tree = tree
        self.temp_dir = tempfile.mkdtemp()
        self.html_document = os.path.join(self.temp_dir, 'bt_debug.html')
        self.svg_document = os.path.join(self.temp_dir, 'tree.svg')

        dot_graph(self.tree, True).write_svg(self.svg_document, encoding='utf8')
        with open(self.html_document, 'w') as f:
            f.write(self.DISPLAY_HTML)

        self.__thread = threading.Thread(target=self.__open_browser)
        self.__thread.start()

    def __open_browser(self):
        if not webbrowser.get(f'"{self.CHROME_PATH}" %s').open('file://' + self.html_document):
            webbrowser.open('file://' + self.html_document)

    def __del__(self):
        while os.path.isdir(self.temp_dir):
            try:
                f = open(self.svg_document, encoding='utf8')
                f.close()
                shutil.rmtree(self.temp_dir)
            except IOError:
                pass

    def tick(self) -> pt.common.Status:
        """Tick the tree once and display its status."""
        self.tree.tick_once()
        self.update_graph()
        return self.tree.status

    def tick_interactive(self):
        """
        Block until the tree returns SUCCESS.

        It Lets the user tick the tree by pressing enter in the console.
        """
        while self.tree.status != pt.common.Status.SUCCESS:
            input('Press ENTER to tick tree')
            self.tick()

    def requires_manual_closing(self) -> bool:
        """Return if this object requires that the browser is closed manually."""
        return self.__thread.is_alive()

    def update_graph(self):
        """Update the visualized graph."""
        dot_graph(self.tree, True).write_svg(self.svg_document, encoding='utf8')
