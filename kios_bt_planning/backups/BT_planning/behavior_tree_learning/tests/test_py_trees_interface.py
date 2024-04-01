"""
Unit test for py_trees_interface.py
"""
import pytest
import py_trees as pt
import behavior_tree_learning.behavior_tree as behavior_tree
import behavior_tree_learning.py_trees_interface as interface
import behavior_tree_learning.tests.behaviors_states as behaviors

def test_pytree():
    """ Tests the PyTree class initialization """
    behavior_tree.load_settings_from_file('behavior_tree_learning/tests/BT_TEST_SETTINGS.yaml')

    bt = ['f(', 'a', 'a', ')']
    py_tree = interface.PyTree(bt, behaviors=behaviors)
    assert bt == []
    assert len(py_tree.root.children) == 2
    print(pt.display.ascii_tree(py_tree.root))

    bt = ['f(', 'f(', 'a', 'a', ')', ')']
    py_tree = interface.PyTree(bt, behaviors=behaviors)
    assert bt == []
    assert len(py_tree.root.children) == 1
    print(pt.display.ascii_tree(py_tree.root))

    bt = ['f(', 'f(', 'a', 'a', ')', 's(', 'a', 'a', ')', ')']
    py_tree = interface.PyTree(bt, behaviors=behaviors)
    assert bt == []
    assert len(py_tree.root.children) == 2
    print(pt.display.ascii_tree(py_tree.root))

    bt = ['f(', 'f(', 'a', 'a', ')', 'f(', 's(', 'a', ')', ')', ')']
    py_tree = interface.PyTree(bt, behaviors=behaviors)
    assert bt == []
    assert len(py_tree.root.children) == 2
    print(pt.display.ascii_tree(py_tree.root))

    bt = ['f(', 'f(', 'a', 'a', ')']
    py_tree = interface.PyTree(bt, behaviors=behaviors)
    assert bt == []
    assert len(py_tree.root.children) == 1
    print(pt.display.ascii_tree(py_tree.root))

    bt = ['f(', 'f(', 'a', ')', ')', 'a', ')']
    py_tree = interface.PyTree(bt, behaviors=behaviors)
    assert bt != []
    print(pt.display.ascii_tree(py_tree.root))

    with pytest.raises(Exception):
        py_tree = interface.PyTree(['nonbehavior'], behaviors=behaviors)

    with pytest.raises(Exception):
        py_tree = interface.PyTree(['f(', 'nonpytreesbehavior', ')'], behaviors=behaviors)

def test_get_bt_from_root():
    """ Specific test for get_string_from_root function """
    behavior_tree.load_settings_from_file('behavior_tree_learning/tests/BT_TEST_SETTINGS.yaml')
    bt = ['f(', 'f(', 'a', 'a', ')', 'f(', 's(', 'a', ')', ')', ')']
    py_tree = interface.PyTree(bt[:], behaviors=behaviors)
    assert py_tree.get_bt_from_root() == bt

    bt = ['f(', 'f(', 'a', ')', 'a', ')']
    py_tree = interface.PyTree(bt[:], behaviors=behaviors)
    assert py_tree.get_bt_from_root() == bt
