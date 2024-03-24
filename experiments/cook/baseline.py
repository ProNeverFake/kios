import json
import os
import re
from pprint import pprint
from typing import List, Tuple, Annotated, TypedDict

from kios_bt.bt_stewardship import BehaviorTreeStewardship
from kios_scene.scene_factory import SceneFactory
from kios_bt.bt_factory import BehaviorTreeFactory
from kios_robot.robot_interface import RobotInterface
from kios_world.world_interface import WorldInterface

from dotenv import load_dotenv

load_dotenv()

from kios_utils.pybt_test import generate_bt_stewardship, render_dot_tree


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
    """
    test the plan from the baseline_result.jsonl
    """
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
    # baseline_plan()
    test_result(15)
