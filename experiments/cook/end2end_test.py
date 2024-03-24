import json
import os
from pprint import pprint
from typing import List, Tuple, Annotated, TypedDict


"""
unit tree generation
"""

os.environ["LANGCHAIN_TRACING_V2"] = "true"
os.environ["LANGCHAIN_ENDPOINT"] = "https://api.smith.langchain.com"
os.environ["LANGCHAIN_PROJECT"] = "kios_e2e"

from kios_bt.bt_stewardship import BehaviorTreeStewardship
from kios_scene.scene_factory import SceneFactory
from kios_bt.bt_factory import BehaviorTreeFactory
from kios_robot.robot_interface import RobotInterface
from kios_world.world_interface import WorldInterface

from kios_agent.llm_supporter import KiosLLMSupporter
from kios_agent.data_types import KiosPromptSkeleton

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
from kios_utils.pddl_problem_parser import parse_problem_init, parse_problem_objects


def render_bt(bt_json: dict):
    test_class = BehaviorTreeFactory()
    bt = test_class.from_json_to_simple_bt(bt_json)
    # bt = test_class.from_json_to_tree_root(bt_json)
    bt_stewardship = generate_bt_stewardship(bt)
    # bt_stewardship.setup(timeout=15)
    render_dot_tree(bt_stewardship)


def get_problem(problem_id: int) -> dict:
    file_dir = os.path.dirname(os.path.abspath(__file__))
    problem_dir = os.path.join(
        file_dir, "problem_set", f"problem_{str(problem_id).zfill(3)}.json"
    )
    with open(problem_dir, "r") as file:
        return json.load(file)


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
system_file = os.path.join(prompt_dir, "end_to_end_v3/system.txt")
task_file = os.path.join(prompt_dir, "end_to_end_v3/task.txt")
domain_file = os.path.join(prompt_dir, "end_to_end_v3/domain.txt")
behaviortree_file = os.path.join(prompt_dir, "end_to_end_v3/behaviortree.txt")
example_file = os.path.join(prompt_dir, "end_to_end_v3/example.txt")
output_format_file = os.path.join(prompt_dir, "end_to_end_v3/output_format.txt")
state_file = os.path.join(prompt_dir, "end_to_end_v3/state.txt")
template_file = os.path.join(prompt_dir, "end_to_end_v3/template.txt")
with open(template_file, "r") as f:
    template_ppt = PromptTemplate.from_template(f.read())
with open(task_file, "r") as f:
    task_ppt = PromptTemplate.from_template(f.read())
with open(system_file, "r") as f:
    system_ppt = PromptTemplate.from_template(f.read())
with open(domain_file, "r") as f:
    domain_ppt = PromptTemplate.from_template(f.read())
with open(state_file, "r") as f:
    state_ppt = PromptTemplate.from_template(f.read())
with open(output_format_file, "r") as f:
    output_format_ppt = PromptTemplate.from_template(f.read())
with open(behaviortree_file, "r") as f:
    ppt_tmp = PromptTemplate.from_template("{input}")
    behaviortree_ppt = ppt_tmp.partial(input=f.read())
with open(example_file, "r") as f:
    ppt_tmp = PromptTemplate.from_template("{input}")
    example_ppt = ppt_tmp.partial(input=f.read())

full_template_ppt = ChatPromptTemplate.from_template(
    """{system}

    {task}

    {domain}

    {state}

    {behaviortree}

    {output_format}

    {example}

    {template}

    {format_instructions}
    """
)

parser = JsonOutputParser()

format_instructions = PromptTemplate.from_template("""{input}""").partial(
    input=parser.get_format_instructions()
)

e2e_ppt_ppl = PipelinePromptTemplate(
    final_prompt=full_template_ppt,
    pipeline_prompts=[
        ("template", template_ppt),
        ("task", task_ppt),
        ("system", system_ppt),
        ("domain", domain_ppt),
        ("behaviortree", behaviortree_ppt),
        ("output_format", output_format_ppt),
        ("example", example_ppt),
        ("state", state_ppt),
        ("format_instructions", format_instructions),
    ],
)

e2e_chain = (
    e2e_ppt_ppl
    # | ChatOpenAI(model="gpt-3.5-turbo-0125", temperature=0)
    # | ChatOpenAI(model="ft:gpt-3.5-turbo-0125:kifabrik-mirmi::8y1cXwVw", temperature=0)
    # | ChatOpenAI(
    #     model="ft:gpt-3.5-turbo-0125:kifabrik-mirmi:kios-ut-gen-v2:8z2KbPsr",
    #     temperature=0,
    # )
    | ChatOpenAI(model="gpt-4", temperature=0)
    | JsonOutputParser()
)


@traceable(name="e2e_test_baselines")
def e2e_test(problem_id: int):
    task = get_problem(problem_id)
    bt = e2e_chain.invoke(
        {
            "target": task["target"],
            "initial_state": task["initial_world_state"],
        }
    )

    result, node = behavior_tree_stewardship.sk_sim_run(
        task["initial_world_state"],
        bt.get("behavior_tree"),
    )

    pprint(result.to_json())
    pprint(f'LLM thought flow: {bt["thought"]}')
    pprint(f'the action sequence: {bt["action_sequence"]}')
    render_bt(bt.get("behavior_tree"))


# def e2e_test_baselines():
#     for i in range(17):
#         e2e_test(i)


if __name__ == "__main__":
    pass
    # e2e_test_baselines()
