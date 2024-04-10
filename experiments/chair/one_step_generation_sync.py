import json
import os
from pprint import pprint
import logging


"""
one step generation
"""

os.environ["LANGCHAIN_TRACING_V2"] = "true"
os.environ["LANGCHAIN_ENDPOINT"] = "https://api.smith.langchain.com"
os.environ["LANGCHAIN_PROJECT"] = "onestep_generation"

from kios_bt.bt_stewardship import BehaviorTreeStewardship
from kios_scene.scene_factory import SceneFactory
from kios_bt.bt_factory import BehaviorTreeFactory
from kios_robot.robot_interface import RobotInterface
from kios_world.world_interface import WorldInterface

from kios_agent.llm_supporter import KiosLLMSupporter
from kios_agent.data_types import KiosPromptSkeleton
from kios_agent.kios_graph import one_step_chain

from dotenv import load_dotenv

from langchain_openai import ChatOpenAI

from langchain.chains.openai_functions import (
    create_structured_output_runnable,
    create_openai_fn_runnable,
)
from langchain_core.prompts import ChatPromptTemplate, MessagesPlaceholder
from langchain_core.output_parsers import JsonOutputParser, StrOutputParser

from langchain.prompts.pipeline import PipelinePromptTemplate
from langchain.prompts.prompt import PromptTemplate

from langchain_community.document_loaders import TextLoader

from langsmith import traceable

load_dotenv()

from kios_utils.pybt_test import generate_bt_stewardship, render_dot_tree

import datetime

timestamp = datetime.datetime.now().strftime("%Y%m%d%H%M%S")

current_dir = os.path.dirname(os.path.abspath(__file__))

problem_set = os.path.join(current_dir, "baseline_result.jsonl")

with open(problem_set, "r") as f:
    problem_set = f.readlines()

problem_number = 2

result_dir = os.path.join(current_dir, "one_step_record", str(problem_number))

if not os.path.exists(result_dir):
    os.makedirs(result_dir)

the_problem = json.loads(problem_set[problem_number])

metadata = {
    "method_name": "one_step_generation",
    "try_count": problem_number,
    "timestamp": timestamp,
    "usecase": "chair",
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

# * end 2 end bt generation ppl
# system_file = os.path.join(prompt_dir, "end_to_end_v3/system.txt")
# task_file = os.path.join(prompt_dir, "end_to_end_v3/task.txt")
# domain_file = os.path.join(prompt_dir, "end_to_end_v3/domain.txt")
# behaviortree_file = os.path.join(prompt_dir, "end_to_end_v3/behaviortree.txt")
# example_file = os.path.join(prompt_dir, "end_to_end_v3/example.txt")
# output_format_file = os.path.join(prompt_dir, "end_to_end_v3/output_format.txt")
# state_file = os.path.join(prompt_dir, "end_to_end_v3/state.txt")
# template_file = os.path.join(prompt_dir, "end_to_end_v3/template.txt")
# with open(template_file, "r") as f:
#     template_ppt = PromptTemplate.from_template(f.read())
# with open(task_file, "r") as f:
#     task_ppt = PromptTemplate.from_template(f.read())
# with open(system_file, "r") as f:
#     system_ppt = PromptTemplate.from_template(f.read())
# with open(domain_file, "r") as f:
#     domain_ppt = PromptTemplate.from_template(f.read())
# with open(state_file, "r") as f:
#     state_ppt = PromptTemplate.from_template(f.read())
# with open(output_format_file, "r") as f:
#     output_format_ppt = PromptTemplate.from_template(f.read())
# with open(behaviortree_file, "r") as f:
#     ppt_tmp = PromptTemplate.from_template("{input}")
#     behaviortree_ppt = ppt_tmp.partial(input=f.read())
# with open(example_file, "r") as f:
#     ppt_tmp = PromptTemplate.from_template("{input}")
#     example_ppt = ppt_tmp.partial(input=f.read())

# full_template_ppt = ChatPromptTemplate.from_template(
#     """{system}

#     {task}

#     {domain}

#     {state}

#     {behaviortree}

#     {output_format}

#     {example}

#     {template}

#     {format_instructions}
#     """
# )

# parser = JsonOutputParser()

# format_instructions = PromptTemplate.from_template("""{input}""").partial(
#     input=parser.get_format_instructions()
# )

# e2e_ppt_ppl = PipelinePromptTemplate(
#     final_prompt=full_template_ppt,
#     pipeline_prompts=[
#         ("template", template_ppt),
#         ("task", task_ppt),
#         ("system", system_ppt),
#         ("domain", domain_ppt),
#         ("behaviortree", behaviortree_ppt),
#         ("output_format", output_format_ppt),
#         ("example", example_ppt),
#         ("state", state_ppt),
#         ("format_instructions", format_instructions),
#     ],
# )

# one_step_chain = (
#     e2e_ppt_ppl
#     # | ChatOpenAI(model="gpt-3.5-turbo-0125", temperature=0)
#     # | ChatOpenAI(model="ft:gpt-3.5-turbo-0125:kifabrik-mirmi::8y1cXwVw", temperature=0)
#     # | ChatOpenAI(
#     #     model="ft:gpt-3.5-turbo-0125:kifabrik-mirmi:kios-ut-gen-v2:8z2KbPsr",
#     #     temperature=0,
#     # )
#     | ChatOpenAI(model="gpt-4", temperature=0)
#     | JsonOutputParser()
# )


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


@traceable(name="one_step")
def one_step() -> dict:
    start_state = the_problem["initial_world_state"]
    target = the_problem["target"]

    response = one_step_chain.invoke(
        {
            "target": target,
            "initial_state": start_state,
        }
    )
    return response

    result, node = behavior_tree_stewardship.sk_sim_run(
        start_state,
        response.get("behavior_tree"),
    )

    pprint(result.to_json())
    pprint(f'LLM thought flow: {response["thought"]}')
    pprint(f'the action sequence: {response["action_sequence"]}')
    render_bt(response.get("behavior_tree"))


def test_one_step():
    response = one_step()

    render_bt(response.get("behavior_tree"), dir=result_dir)

    tree_result = behavior_tree_simulation(
        response.get("behavior_tree"), the_problem["initial_world_state"]
    )

    write_result(response.get("behavior_tree"), result_dir, tree_result)


if __name__ == "__main__":
    pass
    test_one_step()
