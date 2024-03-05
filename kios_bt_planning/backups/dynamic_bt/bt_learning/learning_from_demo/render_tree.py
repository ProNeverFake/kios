"""Methods to render the BT."""

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

import re
from typing import Any

from behaviors.common_behaviors import RandomSelector, RSequence
from behaviors.common_behaviors import ActionBehavior
import py_trees as pt
import pydot


_NUMBER_REGEX = r'-?\d+(?:.\d+)?(?:e-?\d+)?'


class __TreeState:

    regex = re.compile(f'\\({_NUMBER_REGEX}, {_NUMBER_REGEX}, {_NUMBER_REGEX}\\)')

    def __init__(self, short_names: bool):
        self.__number = 0
        self.__position_number = 1
        self.__positions = {}
        self.__short_names = short_names

    def get_number(self):
        self.__number += 1
        return self.__number

    def get_positions(self):
        return self.__positions

    def get_name(self, name: str, latex: bool = False):
        if not self.__short_names:
            return name

        position = self.regex.search(name)
        if position is None:
            return name
        position = position[0]

        if position in self.__positions:
            pos_name = self.__positions[position]
        else:
            if latex:
                pos_name = '$p_{%d}$' % self.__position_number
            else:
                pos_name = 'p%d' % self.__position_number

            self.__positions[position] = pos_name
            self.__position_number += 1

        return self.regex.sub(pos_name, name)


def write_tikz_tree(
    tree: Any,
    file: str,
    short_names: bool = True
):
    """Write latex code to file to draw tree using tikz."""
    if isinstance(tree, pt.trees.BehaviourTree):
        root = tree.root
    else:
        root = tree

    state = __TreeState(short_names)

    with open(file, 'w') as f:
        print('% Please compile your document with LuaLaTex and add', file=f)
        print('% the following to your document preamble:', file=f)
        print(r"""% \usepackage{tikz}
                    % \usetikzlibrary{arrows,decorations.pathmorphing,backgrounds,fit,positioning,
                    calc,shapes,graphs,graphdrawing}
                    % \usegdlibrary{trees}
                    % \tikzset{control/.style={draw, inner sep=1pt, regular polygon,
                    regular polygon sides=4, minimum width=1cm, fill=blue!15}}
                    % \tikzset{action/.style={draw, rectangle, inner sep=0.2cm, fill=green!40}}
                    % \tikzset{condition/.style={draw, minimum width=1cm, ellipse, inner sep=2pt,
                    fill=yellow!80}}
                    % \tikzset{>=stealth}
                    % \tikzset{level distance=1.5cm}
                    % \tikzset{sibling distance=2.5cm}
                    % \tikzset{grow=down}""", file=f)
        print(r'\begin{tikzpicture}', file=f)
        print('\t\\graph[tree layout] {', file=f)
        __generate_tikz(root, f, 2, state)
        print('\n\t};', file=f)
        print(r'\end{tikzpicture}', file=f)

    return state.get_positions()


def dot_graph(
    tree: Any,
    include_status: bool = False,
    short_names: bool = True
) -> pydot.Dot:
    """Create and return a pydot graph from tree."""
    if isinstance(tree, pt.trees.BehaviourTree):
        root = tree.root
    else:
        root = tree

    state = __TreeState(short_names)

    graph = pydot.Dot('BT', graph_type='digraph')
    __generate_dot_tree(root, graph, state, include_status)
    return graph


def py_trees_dot(
    tree: Any,
    short_names: bool = True,
) -> Any:
    """Return a pydot graph in the style of py_trees."""
    if isinstance(tree, pt.trees.BehaviourTree):
        root = tree.root
    else:
        root = tree

    state = __TreeState(short_names)

    display_tree = __display_names(root, state)
    return pt.display.dot_tree(display_tree)


def __generate_tikz(
    root: Any,
    fp: str,
    depth: int,
    state: Any
):
    if hasattr(root, 'get_display_name'):
        name = root.get_display_name()
    else:
        name = root.name
    name = state.get_name(name, True)

    number = state.get_number()

    if isinstance(root, RSequence) or isinstance(root, pt.composites.Sequence):
        fp.write('\t'*depth + f'n{number}[as=$\\rightarrow$, control] -> ' + '{\n')
    elif isinstance(root, RandomSelector):
        fp.write('\t'*depth + f'n{number}[as=??, control] -> ' + '{\n')
    elif isinstance(root, pt.composites.Selector):
        fp.write('\t'*depth + f'n{number}[as=?, control] -> ' + '{\n')
    elif isinstance(root, ActionBehavior):
        fp.write('\t'*depth + f'n{number}[as={{{name}}}, action]')
    elif isinstance(root, pt.behaviour.Behaviour):
        fp.write('\t'*depth + f'n{number}[as={{{name}}}, condition]')

    if isinstance(root, pt.composites.Composite):
        for i, child in enumerate(root.children):
            __generate_tikz(child, fp, depth+1, state)
            if i < len(root.children)-1:
                fp.write(',')
            fp.write('\n')
        fp.write('\t'*depth + '}')

    return state.get_positions()


def __generate_dot_tree(
    root: Any,
    graph: pydot.Dot,
    state: Any,
    include_status: bool
):
    if hasattr(root, 'get_display_name'):
        name = root.get_display_name()
    else:
        name = root.name
    name = state.get_name(name)

    number = state.get_number()
    node = f'n{number}'
    node_instance = None

    if isinstance(root, RSequence) or isinstance(root, pt.composites.Sequence):
        node_instance = pydot.Node(
            node, label='→', shape='square', margin='0.1', width='0', height='0')
    elif isinstance(root, RandomSelector):
        node_instance = pydot.Node(
            node, label='??', shape='square', margin='0.1', width='0', height='0')
    elif isinstance(root, pt.composites.Selector):
        node_instance = pydot.Node(
            node, label='?', shape='square', margin='0.1', width='0', height='0')
    elif isinstance(root, ActionBehavior):
        node_instance = pydot.Node(
            node, label=name, shape='box')
    elif isinstance(root, pt.behaviour.Behaviour):
        node_instance = pydot.Node(
            node, label=name, shape='ellipse')

    if node_instance is not None:
        if include_status:
            if root.status == pt.common.Status.SUCCESS:
                node_instance.set('color', 'lightgreen')
                node_instance.set('penwidth', '2')
            elif root.status == pt.common.Status.FAILURE:
                node_instance.set('color', 'red')
                node_instance.set('penwidth', '2')
            elif root.status == pt.common.Status.RUNNING:
                node_instance.set('color', 'orange')
                node_instance.set('penwidth', '2')
        graph.add_node(node_instance)

    if isinstance(root, pt.composites.Composite):
        for i, child in enumerate(root.children):
            child_node = __generate_dot_tree(child, graph, state, include_status)
            edge = pydot.Edge(node, child_node)
            if include_status:
                color = graph.get_node(child_node)[0].get('color')
                if color is not None:
                    edge.set('color', color)
                    edge.set('penwidth', '2')
            graph.add_edge(edge)

    return node


def __display_names(
    root: Any,
    state: Any
):
    """Create a new tree with empty behaviors and display names."""
    if hasattr(root, 'get_display_name'):
        name = root.get_display_name()
    else:
        name = root.name
    name = state.get_name(name)

    if isinstance(root, pt.composites.Composite):
        if root.name == 'Sequence':
            # control = RSequence()
            control = pt.composites.Sequence('Sequence', memory=False)
        elif root.name == 'Fallback':
            control = pt.composites.Selector('Fallback')
        else:
            raise ValueError('Unknown control flow node ' + root.name)

        for child in root.children:
            copy = __display_names(child, state)
            control.add_child(copy)

        return control
    else:
        return pt.behaviour.Behaviour(name=name)
