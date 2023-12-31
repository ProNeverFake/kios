"""Test settings for behavior_lists.py module."""

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

from behaviors import behavior_lists


def get_condition_nodes():
    """Return a list of condition nodes."""
    return [
        behavior_lists.ParameterizedNode("b", None, [], True),
        behavior_lists.ParameterizedNode("c", None, [], True),
        behavior_lists.ParameterizedNode(
            "d", None, [behavior_lists.NodeParameter(["0", "100"])], True
        ),
        behavior_lists.ParameterizedNode(
            "e", None, [behavior_lists.NodeParameter([], 0, 100, 100)], True
        ),
        behavior_lists.ParameterizedNode(
            "value check", None, [behavior_lists.NodeParameter([], 0, 100, 100)], True
        ),
    ]


def get_action_nodes():
    """Return a list of action nodes."""
    return [
        behavior_lists.ParameterizedNode("ab", None, [], False),
        behavior_lists.ParameterizedNode("ac", None, [], False),
        behavior_lists.ParameterizedNode("ad", None, [], False),
        behavior_lists.ParameterizedNode("ae", None, [], False),
        behavior_lists.ParameterizedNode("af", None, [], False),
        behavior_lists.ParameterizedNode("ag", None, [], False),
    ]
