import json
import os
from pprint import pprint
import logging


"""
iterative generation
"""

os.environ["LANGCHAIN_TRACING_V2"] = "true"
os.environ["LANGCHAIN_ENDPOINT"] = "https://api.smith.langchain.com"
# os.environ["LANGCHAIN_PROJECT"] = "iterative_generation"
os.environ["LANGCHAIN_PROJECT"] = "tryout"


from kios_bt.bt_stewardship import BehaviorTreeStewardship
from kios_scene.scene_factory import SceneFactory
from kios_bt.bt_factory import BehaviorTreeFactory
from kios_robot.robot_interface import RobotInterface
from kios_world.world_interface import WorldInterface

from kios_agent.llm_supporter import KiosLLMSupporter
from kios_agent.data_types import KiosPromptSkeleton
from kios_agent.kios_graph import iterative_generation_chain

from dotenv import load_dotenv

from langsmith import traceable

load_dotenv()

from kios_utils.pybt_test import generate_bt_stewardship, render_dot_tree

import datetime

timestamp = datetime.datetime.now().strftime("%Y%m%d%H%M%S")

current_dir = os.path.dirname(os.path.abspath(__file__))

problem_set = os.path.join(current_dir, "baseline_result.jsonl")

with open(problem_set, "r") as f:
    problem_set = f.readlines()

problem_number = 3

result_dir = os.path.join(current_dir, "iterative_record", str(problem_number))

if not os.path.exists(result_dir):
    os.makedirs(result_dir)

the_problem = json.loads(problem_set[problem_number])

metadata = {
    "method_name": "iterative_generation",
    "try_count": problem_number,
    "timestamp": timestamp,
    "usecase": "lamp",
}


def render_bt(bt_json: json, dir=None):
    test_class = BehaviorTreeFactory()
    bt = test_class.from_json_to_simple_bt(bt_json)
    # bt = test_class.from_json_to_tree_root(bt_json)
    bt_stewardship = generate_bt_stewardship(bt)
    # bt_stewardship.setup(timeout=15)
    time_stamp = datetime.datetime.now().strftime("%Y%m%d%H%M%S")
    render_dot_tree(bt_stewardship, name=time_stamp, dir=dir)


def write_result(bt_json: json, dir: str, tree_result: dict):
    result = {
        "problem": the_problem,
        "behavior_tree": bt_json,
        "tree_result": tree_result,
    }
    with open(os.path.join(dir, "result.json"), "w") as f:
        json.dump(result, f)


####################### dirs
current_dir = os.path.dirname(os.path.abspath(__file__))
scene_path = os.path.join(current_dir, "scene.json")
# bt_json_file_path = os.path.join(current_dir, "behavior_tree.json")
world_state_path = os.path.join(current_dir, "world_state.json")
domain_knowledge_path = os.path.join(current_dir, "domain_knowledge.txt")

####################### scene
with open(scene_path, "r") as file:
    scene_json_object = json.load(file)

scene = SceneFactory().create_scene_from_json(scene_json_object)

####################### world
world_interface = WorldInterface()
with open(world_state_path, "r") as file:
    world_state_json = json.load(file)
    world_interface.load_world_from_json(world_state_json)

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

# * kios data prompt skeleton dir
data_dir = os.environ.get("KIOS_DATA_DIR").format(username=os.getlogin())
print(data_dir)
prompt_sk_dir = os.path.join(data_dir, "prompt_skeletons")
prompt_dir = os.path.join(data_dir, "prompts")


@traceable(name="final_simulation", metadata=metadata)
def behavior_tree_simulation(bt_skeleton: dict, world_state: dict) -> dict:
    """
    execute the first step of the plan, append the result to the past steps
    """
    print(f"-----behavior_tree_simulation_step-----")

    try:
        behavior_tree_stewardship.set_world_state(world_state)

        behavior_tree_stewardship.generate_behavior_tree_from_skeleton(bt_skeleton)

        behavior_tree_stewardship.setup_simulation()

        behavior_tree_stewardship.setup_behavior_tree()

        behavior_tree_stewardship.tick_tree(period_msec=500)

        tree_result = behavior_tree_stewardship.tree_result

        pprint(tree_result.to_json())
        pause = input("DEBUG: please check the tree result. Press enter to continue.")

        return tree_result.to_json()
    except Exception as e:
        logging.error(f"Error occurred in the simulation: {e}")
        return {
            "result": "error",
            "summary": str(e),
            "world_state": world_state,
            "final_node": None,
        }
    except KeyboardInterrupt:
        logging.error(f"Execution has been interrupted.")
        return {
            "result": "error",
            "summary": "endless loop in execution",
            "world_state": world_state,
            "final_node": None,
        }


@traceable(name="rollout_simulation", metadata=metadata)
def rollout_simulation(bt_skeleton: dict, world_state: dict) -> dict:
    """
    execute the first step of the plan, append the result to the past steps
    """
    print(f"-----rollout_simulation_step-----")
    global behavior_tree_stewardship
    global last_bt
    global last_failed_node
    global summary
    global runtime_state
    global start_state

    try:
        tree_result, skeleton_json = behavior_tree_stewardship.sk_sim_run(
            world_state=start_state, skeleton_json=bt_skeleton
        )
        return tree_result.to_json()
    except Exception as e:
        logging.error(f"Error occurred in the simulation: {e}")
        return {
            "result": "error",
            "summary": str(e),
            "world_state": world_state,
            "final_node": None,
        }
    except KeyboardInterrupt:
        logging.error(f"Execution has been interrupted.")
        return {
            "result": "error",
            "summary": "endless loop in execution",
            "world_state": world_state,
            "final_node": None,
        }


last_bt = None
last_failed_node = None
summary = None
runtime_state = None
start_state = the_problem["initial_world_state"]


@traceable(name="iterative_generation_step", metadata=metadata)
def iterative_step() -> dict:
    print(f"-----iterative_generation-step-----")
    global last_bt
    global runtime_state
    global summary
    global last_failed_node
    start_state = the_problem["initial_world_state"]
    target = the_problem["target"]

    response = iterative_generation_chain.invoke(
        {
            "target": target,
            "initial_world_state": start_state,
            "runtime_world_state": runtime_state,
            "last_behavior_tree": last_bt,
            "last_failed_node": last_failed_node,
        }
    )

    return response


def test_iterative_generation():
    i = 0
    global last_bt
    global runtime_state
    global last_failed_node
    global summary

    while i < 5:
        response = iterative_step()
        render_bt(response.get("behavior_tree"), dir=result_dir)
        last_bt = response.get("behavior_tree")

        tree_result = rollout_simulation(last_bt, start_state)
        pause = input("DEBUG: please check the tree result. Press enter to continue.")

        if tree_result.get("result") == "success":
            write_result(last_bt, result_dir, tree_result)
            exit(0)
        else:
            runtime_state = tree_result.get("world_state")
            last_failed_node = tree_result.get("final_node")
            summary = tree_result.get("summary")
            i += 1

    # tree_result = behavior_tree_simulation(
    #     response.get("behavior_tree"), the_problem["initial_world_state"]
    # )

    write_result(last_bt, result_dir, tree_result)


if __name__ == "__main__":
    pass
    test_iterative_generation()
