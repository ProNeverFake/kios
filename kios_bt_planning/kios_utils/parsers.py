import re
from kios_bt.data_types import BaseNode
from functools import partial
from typing import Any


def match_type(node: dict) -> tuple[str, str]:
    """
    match the type of the node and extract the node body xxx(arg1, arg2)

    return: tuple of node type and node body in str
    raise: ValueError if the node name does not match any type
    """
    node_name = node["name"]
    match = re.search(
        r"(selector|sequence|action|precondition|condition|target|parallel):\s*(.+)",
        node_name,
    )
    if match:
        node_type = match.group(1)
        node_body = match.group(2)
        return node_type, node_body
    else:
        raise ValueError(f"the node name {node_name} does not match any type.")


def traversal_handler(
    tree_node: dict,
    predicate: object = lambda tree_node: True,
    processor: object = lambda tree_node: tree_node["name"],
) -> Any | None:
    """
    the traversal handler for node processing.
    can be partialed with the predicate and processor functions to filter and process the tree nodes.

    tree: the behavior tree in json format
    predicate: a function to filter the tree node, default to allow all nodes to pass
    processor: a function to process the tree node, default to return the tree node as it is
    return: the processed tree node, or None if the tree node is filtered out
    """
    # extract the node type and node body
    if predicate(tree_node):
        return processor(tree_node)
    else:
        return None


def postorder_traversal(
    tree_node: dict, sequence_list: list = [], node_handler: object = traversal_handler
) -> list:
    """
    this method assumes that sequence nodes are the only control flow node type in the tree.
    If a different control flow node type is used, like selector or parallel nodes, a queue or
    list is actually not the appropriate data structure to store the corresponding tree.

    tree: the behavior tree in json format
    sequence_list: the list to store the sequence of the tree
    node_handler: the handler for node, default to return the node as it is
    return: sequence_list
    """
    node_type, node_body = match_type(tree_node)

    if node_type == "sequence":
        for child in tree_node["children"]:
            postorder_traversal(child, sequence_list, node_handler)
        # sequence_list.append(tree_node) # appending the sequence might not be necessary
    elif node_type == "parallel":
        for child in tree_node["children"]:
            postorder_traversal(child, sequence_list, node_handler)
    elif node_type == "selector":
        postorder_traversal(tree_node["children"][-1], sequence_list, node_handler)
    else:
        sequence_list.append(node_handler(tree_node))

    return sequence_list


def base_traversal(
    tree_node: dict,
    node_handler: object = partial(traversal_handler, predicate=lambda x: True),
) -> BaseNode | None:
    """
    traverse the json behavior tree to construct a behavior tree in the BaseNode format
    """
    raise NotImplementedError


def test_traversal():
    example_bt = {
        "summary": "sequence to perform the task of manipulating the glass and water bottle with left and right arms",
        "name": "sequence: main_task",
        "children": [
            {
                "summary": "parallel to perform initial reaching and grasping tasks",
                "name": "parallel: initial_reach_and_grasp",
                "children": [
                    {
                        "summary": "sequence for left arm to reach and grasp the glass on table1",
                        "name": "sequence: left_arm_reach_and_grasp_glass",
                        "children": [
                            {
                                "summary": "left arm reaches the glass on table1",
                                "name": "action: REACH(left_arm, table1)",
                            },
                            {
                                "summary": "left arm grasps the glass",
                                "name": "action: GRASP(left_arm, glass)",
                            },
                            {
                                "summary": "left arm retreats with the glass",
                                "name": "action: RETREAT(left_arm)",
                            },
                        ],
                    },
                    {
                        "summary": "sequence for right arm to reach and grasp the water bottle on table2",
                        "name": "sequence: right_arm_reach_and_grasp_bottle",
                        "children": [
                            {
                                "summary": "right arm reaches the water bottle on table2",
                                "name": "action: REACH(right_arm, table2)",
                            },
                            {
                                "summary": "right arm grasps the water bottle",
                                "name": "action: GRASP(right_arm, watter_bottle)",
                            },
                            {
                                "summary": "right arm retreats with the water bottle",
                                "name": "action: RETREAT(right_arm)",
                            },
                        ],
                    },
                ],
            },
            {
                "summary": "sequence for right arm to pour water into the glass",
                "name": "sequence: right_arm_pour_water",
                "children": [
                    {
                        "summary": "right arm waits if the glass is not ready",
                        "name": "selector: wait_or_pour",
                        "children": [
                            {
                                "summary": "condition to check if the glass is ready",
                                "name": "condition: is_ready(glass)",
                            },
                            {
                                "summary": "right arm waits",
                                "name": "action: WAIT(right_arm)",
                            },
                        ],
                    },
                    {
                        "summary": "right arm pours water from the bottle into the glass",
                        "name": "action: POUR(right_arm, watter_bottle, glass)",
                    },
                ],
            },
            {
                "summary": "parallel to perform final releasing tasks",
                "name": "parallel: final_release",
                "children": [
                    {
                        "summary": "sequence for left arm to release the glass on table1",
                        "name": "sequence: left_arm_release_glass",
                        "children": [
                            {
                                "summary": "left arm reaches table1",
                                "name": "action: REACH(left_arm, table1)",
                            },
                            {
                                "summary": "left arm releases the glass",
                                "name": "action: RELEASE(left_arm, glass)",
                            },
                        ],
                    },
                    {
                        "summary": "sequence for right arm to release the water bottle on table2",
                        "name": "sequence: right_arm_release_bottle",
                        "children": [
                            {
                                "summary": "right arm reaches table2",
                                "name": "action: REACH(right_arm, table2)",
                            },
                            {
                                "summary": "right arm releases the water bottle",
                                "name": "action: RELEASE(right_arm, watter_bottle)",
                            },
                        ],
                    },
                ],
            },
        ],
    }

    from pprint import pprint

    pprint(postorder_traversal(example_bt))


if __name__ == "__main__":
    test_traversal()
