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
# with open(problem_path, "r") as file:
#     problem = file.read()

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

    # * first sim run
    from kios_plan.dynamic_planning import chair_ut_dict

    solultion = behavior_tree_stewardship.sk_baseline(
        initial_world_state, sk_json, chair_ut_dict
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

def test_baseline():
    sk = {
        # "summary": "the target is to screw the chairnut1 into the chairseatbolt1",
        # "name": "target: is_screwed_to(chairnut1, chairseatbolt1)",
        # "summary": "the target is to screw the chairnut2 into the chairseatbolt2",
        # "name": "target: is_screwed_to(chairnut2, chairseatbolt2)",
        "summary": "the target is to screw the chairleg1 into the chairseatthread1",
        "name": "target: is_screwed_to(chairleg1, chairseatthread1)",
        # "summary": "the target is to screw the chairleg2 into the chairseatthread2",
        # "name": "target: is_screwed_to(chairleg2, chairseatthread2)",
        # "summary": "the target is to insert the chairback into the chairseatconnector",
        # "name": "target: is_inserted_to(chairback, chairseatconnector)",
    }
    baseline_run(sk, world_state_json)


if __name__ == "__main__":
    pass

    # test_expand_nodes()
    test_baseline()
