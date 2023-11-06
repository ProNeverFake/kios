"""
Test for planner
"""
import duplo_state_machine.planner_behaviors as behaviors
import duplo_state_machine.state_machine as sm
import behavior_tree_learning.planner as planner
import behavior_tree_learning.behavior_tree as behavior_tree

def test_tower():
    """ Test run of stacking three bricks in a tower """
    behavior_tree.load_settings_from_file('duplo_state_machine/BT_SETTINGS_TOWER.yaml')
    start_positions = []
    start_positions.append(sm.Pos(-0.05, -0.1, 0))
    start_positions.append(sm.Pos(0, -0.1, 0))
    start_positions.append(sm.Pos(0.05, -0.1, 0))
    state_machine = sm.StateMachine(start_positions, False)
    goals = ['0 at pos (0.0, 0.05, 0.0)?', '1 at pos (0.0, 0.05, 0.0192)?', '2 at pos (0.0, 0.05, 0.0384)?']
    planner.plan(state_machine, behaviors, goals)

    behavior_tree.load_settings_from_file('behavior_tree_learning/tests/BT_TEST_SETTINGS.yaml')

def test_croissant():
    """ Test run of stacking four bricks in a structure looking somewhat like a croissant """
    behavior_tree.load_settings_from_file('duplo_state_machine/BT_SETTINGS_CROISSANT.yaml')
    start_positions = []
    start_positions.append(sm.Pos(-0.05, -0.1, 0))
    start_positions.append(sm.Pos(0, -0.1, 0))
    start_positions.append(sm.Pos(0.05, -0.1, 0))
    start_positions.append(sm.Pos(0.1, -0.1, 0))
    state_machine = sm.StateMachine(start_positions, False)
    goals = ['0 at pos (0.0, 0.0, 0.0)?', '1 at pos (0.0, 0.0, 0.0192)?', \
             '2 at pos (0.016, -0.032, 0.0)?', '3 at pos (0.016, 0.032, 0.0)?']
    planner.plan(state_machine, behaviors, goals)

    behavior_tree.load_settings_from_file('behavior_tree_learning/tests/BT_TEST_SETTINGS.yaml')

def test_balance():
    """ Test run of balancing with off gravity piece """
    behavior_tree.load_settings_from_file('duplo_state_machine/BT_SETTINGS_BALANCE.yaml')
    start_positions = []
    start_positions.append(sm.Pos(-0.05, -0.1, 0.0))
    start_positions.append(sm.Pos(0.0, -0.1, 0.0))
    start_positions.append(sm.Pos(0.05, -0.1, 0.0))

    state_machine = sm.StateMachine(start_positions, False)
    goals = ['0 at pos (0.0, 0.0, 0.0192)?']

    planner.plan(state_machine, behaviors, goals)

    behavior_tree.load_settings_from_file('behavior_tree_learning/tests/BT_TEST_SETTINGS.yaml')

def test_blocking():
    """ Test run of shuffling bricks to avoid blocking """
    behavior_tree.load_settings_from_file('duplo_state_machine/BT_SETTINGS_BLOCKING.yaml')
    start_positions = []
    start_positions.append(sm.Pos(0.0, -0.05, 0.0))
    start_positions.append(sm.Pos(0.0, 0.05, 0.0))
    start_positions.append(sm.Pos(-0.1, 0.0, 0.0))

    state_machine = sm.StateMachine(start_positions, False)
    goals = ['0 at pos (-0.1, 0.0, 0.0)?', '1 at pos (-0.1, 0.0, 0.0192)?', '2 at pos (0.0, 0.0, 0.0)?']

    planner.plan(state_machine, behaviors, goals)

    behavior_tree.load_settings_from_file('behavior_tree_learning/tests/BT_TEST_SETTINGS.yaml')
