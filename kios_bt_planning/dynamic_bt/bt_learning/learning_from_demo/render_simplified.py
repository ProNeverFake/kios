"""Methods to render a shrinked version of the BT."""

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
from typing import Any

from behaviors.common_behaviors import RandomSelector, RSequence
import bt_learning.learning_from_demo.render_tree as rt
import py_trees as pt


def write_simplified_svg(tree: Any, file: str):
    simplified_figure(tree, file, lambda t, f: rt.dot_graph(t).write_svg(f, encoding='utf8'))


def write_simplified_tikz(tree: Any, file: str):
    simplified_figure(tree, file, rt.write_tikz_tree)


def write_simplified_py_trees(tree: Any, file: str):
    simplified_figure(tree, file, lambda t, f: rt.py_trees_dot(t).write_svg(f))


def simplified_figure(
    tree: Any,
    file: str,
    generation_method: Any
):
    """
    Write an image of a simplified BT where only place nodes are present.

    Args:
    ----
        tree: the Behavior Tree to save as image.
        file: path with the name of the image.
        generation_method: function that creates the image of the tree and saves it in file.

    """
    if isinstance(tree, pt.trees.BehaviourTree):
        tree = tree.root

    __collapse_nodes(tree)

    if len(tree.children) == 1:
        # If only one child replace with child
        tree = tree.children[0]

    generation_method(tree, file)


def __collapse_nodes(tree: Any):
    if isinstance(tree, RSequence):
        children = copy(tree.children)
        for child in children:
            __collapse_nodes(child)
        if len(tree.children) == 1 and tree.parent is not None:
            tree.parent.replace_child(tree, tree.children[0])
    elif isinstance(tree, pt.composites.Selector) or isinstance(tree, RandomSelector):
        name = tree.children[0].name
        if not (name.startswith('object_at') or name.startswith('object_roughly_at')) and\
           tree.parent is not None:
            tree.parent.remove_child(tree)
        else:
            for child in tree.children:
                __collapse_nodes(child)
    elif not (tree.name.startswith('place') or
              tree.name.startswith('object_at') or
              tree.name.startswith('object_roughly_at')):
        tree.parent.remove_child(tree)


class LeafNode(pt.behaviour.Behaviour):

    def __init__(self, name: str):
        super().__init__(name)
