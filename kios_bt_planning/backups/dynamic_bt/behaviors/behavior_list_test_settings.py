"""Test settings for behavior_lists.py module."""

from . import behavior_lists


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

