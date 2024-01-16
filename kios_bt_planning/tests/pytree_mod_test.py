import py_trees

from kios_bt.behavior_nodes import ActionNode, ConditionNode

from kios_bt.data_types import (
    Action,
    Condition,
    ObjectProperty,
)

from kios_utils.pybt_test import (
    generate_bt_stewardship,
    tick_once_test,
    render_dot_tree,
)

from kios_bt.bt_factory import BehaviorTreeFactory

action = Action(
    summary="pick up apple",
    identifier=0,
    name="pick_up",
    effects=[
        ObjectProperty(
            object_name="apple",
            property_name="on_the_ground",
            property_value=None,
            status=False,
        ),
        ObjectProperty(
            object_name="apple",
            property_name="in",
            property_value="hand",
            status=True,
        ),
        ObjectProperty(
            object_name="hand",
            property_name="free",
            property_value=None,
            status=False,
        ),
    ],
)

precondition1 = Condition(
    summary="check if the apple is on the ground",
    identifier=1,
    name="apple on the ground",
    conditions=[
        ObjectProperty(
            object_name="apple",
            property_name="on_the_ground",
            property_value=None,
            status=True,
        ),
    ],
)

precondition2 = Condition(
    summary="check if the hand is free",
    identifier=2,
    name="hand free",
    conditions=[
        ObjectProperty(
            object_name="hand",
            property_name="free",
            property_value=None,
            status=True,
        ),
    ],
)

effect = Condition(
    summary="check if the apple is in hand",
    identifier=3,
    name="apple in hand",
    conditions=[
        ObjectProperty(
            object_name="apple",
            property_name="in",
            property_value="hand",
            status=True,
        ),
    ],
)

another_action = Action(
    summary="put down apple",
    identifier=4,
    name="put_down",
    effects=[
        ObjectProperty(
            object_name="apple",
            property_name="on_the_ground",
            property_value=None,
            status=True,
        ),
        ObjectProperty(
            object_name="apple",
            property_name="in",
            property_value="hand",
            status=False,
        ),
        ObjectProperty(
            object_name="hand",
            property_name="free",
            property_value=None,
            status=True,
        ),
    ],
)

bt_factory = BehaviorTreeFactory()

precondition1_node = bt_factory.generate_condition_node(precondition1)
precondition2_node = bt_factory.generate_condition_node(precondition2)
effect_node = bt_factory.generate_condition_node(effect)
action_node = bt_factory.generate_fake_action_node(action)

another_action_node = bt_factory.generate_fake_action_node(another_action)

another_action_node1 = bt_factory.generate_fake_action_node(another_action)

subtree = bt_factory.generate_subtree(
    preconditions=[precondition1_node, precondition2_node],
    action=action_node,
    effects=[effect_node],
)

behaviortree = generate_bt_stewardship(subtree)

render_dot_tree(behaviortree)

tick_once_test(behaviortree)

# try the mod methods in pytree

behaviortree.replace_subtree(action_node.id, another_action_node)

tick_once_test(behaviortree)

behaviortree.insert_subtree(
    another_action_node1,
    another_action_node.parent.id,
    index=1,
)

behaviortree.setup(timeout=15)


tick_once_test(behaviortree)
