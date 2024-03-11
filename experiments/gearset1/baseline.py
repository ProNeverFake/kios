import json
import os
import re
from pprint import pprint
from typing import List, Tuple, Annotated, TypedDict
import operator
import logging
import asyncio

from kios_bt.bt_stewardship import BehaviorTreeStewardship
from kios_scene.scene_factory import SceneFactory
from kios_bt.bt_factory import BehaviorTreeFactory
from kios_robot.robot_interface import RobotInterface
from kios_world.world_interface import WorldInterface

from kios_agent.llm_supporter import KiosLLMSupporter
from kios_agent.data_types import KiosPromptSkeleton

from kios_agent.kios_tools import (
    BehaviorTreeSimulatorTool,
)

from dotenv import load_dotenv

load_dotenv()

from kios_utils.pybt_test import generate_bt_stewardship, render_dot_tree
from kios_utils.pddl_problem_parser import parse_problem_init, parse_problem_objects


def render_bt(bt_json: dict):
    test_class = BehaviorTreeFactory()
    bt = test_class.from_json_to_simple_bt(bt_json)
    # bt = test_class.from_json_to_tree_root(bt_json)
    bt_stewardship = generate_bt_stewardship(bt)
    # bt_stewardship.setup(timeout=15)
    render_dot_tree(bt_stewardship)


####################### dirs
current_dir = os.path.dirname(os.path.abspath(__file__))
scene_path = os.path.join(current_dir, "scene.json")
# bt_json_file_path = os.path.join(current_dir, "behavior_tree.json")
world_state_path = os.path.join(current_dir, "world_state.json")
problem_path = os.path.join(current_dir, "gearset.problem")
domain_knowledge_path = os.path.join(current_dir, "domain_knowledge.txt")

####################### problem
with open(problem_path, "r") as file:
    problem = file.read()

####################### scene
with open(scene_path, "r") as file:
    scene_json_object = json.load(file)

scene = SceneFactory().create_scene_from_json(scene_json_object)

####################### world
world_interface = WorldInterface()
with open(world_state_path, "r") as file:
    world_state_json = json.load(file)
    world_interface.load_world_from_json(world_state_json)

pprint(world_state_json)


####################### robot
robot_interface = RobotInterface(
    robot_address="127.0.0.1",
    robot_port=12000,
)
robot_interface.setup_scene(scene)

####################### bt_factory
bt_factory = BehaviorTreeFactory(
    world_interface=world_interface,
    robot_interface=robot_interface,
)

####################### behavior_tree_stewardship
behavior_tree_stewardship = BehaviorTreeStewardship(
    behaviortree_factory=bt_factory,
    world_interface=world_interface,
    robot_interface=robot_interface,
)


def baseline_run(tree_root: dict, world_state: dict):

    sk_json = tree_root
    initial_world_state = world_state

    from kios_plan.dynamic_planning import gearset_ut_dict

    # * first sim run
    solultion = behavior_tree_stewardship.sk_baseline(
        initial_world_state,
        sk_json,
        gearset_ut_dict,
    )
    record = {
        "target": sk_json["name"],
        "initial_world_state": initial_world_state,
        "result": solultion,
    }
    file_dir = os.path.join(current_dir, "baseline_result.jsonl")
    with open(file_dir, "a") as file:
        file.write(json.dumps(record) + "\n")


def match_type(node: dict) -> tuple[str, str]:
    node_name = node["name"]
    match = re.search(
        r"(selector|sequence|action|precondition|condition|target):\s*(.+)", node_name
    )
    if match:
        node_type = match.group(1)
        node_body = match.group(2)
        return node_type, node_body
    else:
        raise ValueError(f"the node name {node_name} does not match any type.")


def expand_nodes(
    node_list: list[dict],
    start_state: dict,
    overall_tree: list[dict] = None,
) -> dict:
    """
    in order to monitor the tree generation, the overall tree and the node list should be the same variable when being passed in.
    """
    pprint("----------check the entire tree:")
    if overall_tree is not None:
        render_bt(overall_tree[0])
    pprint("----------start to expand the node list:")
    pprint(node_list)
    pause = input("paused here! check the tree.")

    assert len(node_list) > 0
    state = start_state

    for i in range(len(node_list)):
        type_name, body = match_type(node_list[i])
        # if match_type(node_list[i]) == "action":
        if type_name == "action":
            print(f"the node {node_list[i]['name']} is an action node. skip it.")
            pause = input("paused here! check!")
        # elif match_type(node_list[i]) == "precondition" or "target":
        elif type_name in ["precondition", "target"]:
            # goal = node_list[i]["name"]
            goal = body
            plan = make_plan(state, goal)
            if len(plan) == 0:
                logging.warning(f"No action should be performed for the goal {goal}.")
                logging.warning(f'the node {node_list[i]["name"]} has been skipped.')
                pause = input("paused here! check!")
            else:
                logging.info(f"Actions have been planed for the goal {goal}.")
                pprint(f"the plan for the goal {goal} is {plan}")
                pause = input("paused here! check!")
                last_action = plan[-1]
                unit_subtree = generate_unit_subtree(last_action)
                # insert the subtree into the node_list
                node_list[i] = unit_subtree
                new_node_list = get_node_list_from_tree(unit_subtree)
                expand_nodes(
                    node_list=new_node_list,
                    start_state=state,
                    overall_tree=overall_tree,
                )
                state = estimate_state(state, plan)

    return node_list[0]


def test_expand_nodes():
    start_state = world_state_json
    node_list = [
        {
            "summary": "insert shaft1 into gearbase hole1",
            "name": "target: insert shaft1 into gearbase hole1",
        }
    ]
    # node_list = [
    #     {
    #         "summary": "insert gear2 into shaft2",
    #         "name": "target: insert gear2 into shaft2",
    #     }
    # ]
    # node_list = [
    #     {
    #         "summary": "pick up the shaft1",
    #         "name": "target: pick up the shaft1",
    #     },
    # ]
    expand_nodes(node_list, start_state, node_list)
    pprint(node_list)


def baseline_plan():
    sk = {
        # "summary": "the target is to insert the shaft1 into the gearbase_hole1",
        # "name": "target: is_inserted_to(shaft1, gearbase_hole1)",
        # "summary": "the target is to insert the shaft3 into the gearbase_hole3",
        # "name": "target: is_inserted_to(shaft3, gearbase_hole3)",
        # "summary": "the target is to insert the gear2 into the shaft2",
        # "name": "target: is_inserted_to(gear2, shaft2)",
        # "summary": "the target is to insert gear3 into shaft3",
        # "name": "target: is_inserted_to(gear3, shaft3)",
        "summary": "the target is to insert gear1 to shaft1",
        "name": "target: is_inserted_to(gear1, shaft1)",
    }
    baseline_run(sk, world_state_json)


def test_result(problem_id: int):
    file_dir = os.path.join(current_dir, "baseline_result.jsonl")
    with open(file_dir, "r") as file:
        results = file.read()
        data = [json.loads(line) for line in results.splitlines()]
    problem = data[problem_id]
    initial_world_state = problem["initial_world_state"]
    sk = problem["result"]
    result, node = behavior_tree_stewardship.sk_sim_run(
        world_state=initial_world_state, skeleton_json=sk
    )
    pprint(result.result)
    pprint(result.summary)


if __name__ == "__main__":
    pass

    # test_expand_nodes()
    # baseline_plan()
    test_result(15)
