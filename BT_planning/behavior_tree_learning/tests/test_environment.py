"""
Tests various environment setups
"""

import behavior_tree_learning.tests.environment_states as environment_states
import behavior_tree_learning.behavior_tree as behavior_tree
behavior_tree.load_settings_from_file('behavior_tree_learning/tests/BT_TEST_SETTINGS.yaml')

def test_state_environment():
    """
    Tests a state machine environment
    """
    assert environment_states.get_fitness(['t1']) == 9

    assert environment_states.get_fitness(['s(', 't1', 't2', 't3', 't4', ')']) == 35

    assert environment_states.get_fitness(['s(', 't1', 't2', 't3', 'r1', 'r2', 'r3', 'r4', ')']) == 22
