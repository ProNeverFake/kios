import json
import os
import re
from pprint import pprint
from typing import List, Tuple, Annotated, TypedDict
import operator
import logging
import asyncio

"""
unit tree generation
"""

os.environ["LANGCHAIN_TRACING_V2"] = "true"
os.environ["LANGCHAIN_ENDPOINT"] = "https://api.smith.langchain.com"
os.environ["LANGCHAIN_PROJECT"] = "kios_agent"

from kios_bt.bt_stewardship import BehaviorTreeStewardship
from kios_scene.scene_factory import SceneFactory
from kios_bt.bt_factory import BehaviorTreeFactory
from kios_robot.robot_interface import RobotInterface
from kios_world.world_interface import WorldInterface

from kios_agent.llm_supporter import KiosLLMSupporter
from kios_agent.data_types import KiosPromptSkeleton

from kios_agent.kios_tools import (
    BehaviorTreeExecutorTool,
    BehaviorTreeSimulatorTool,
    WorldStateQueryTool,
)

from dotenv import load_dotenv

from langchain import hub

from langchain_openai import ChatOpenAI

from langchain.chains.openai_functions import (
    create_structured_output_runnable,
    create_openai_fn_runnable,
)
from langchain_core.utils.function_calling import convert_to_openai_function
from langchain_core.agents import AgentActionMessageLog, AgentFinish
from langchain_core.messages import HumanMessage
from langchain_core.prompts import ChatPromptTemplate, MessagesPlaceholder
from langchain_core.output_parsers import JsonOutputParser, StrOutputParser
from langchain_core.pydantic_v1 import BaseModel, Field

from langchain.agents.format_scratchpad import format_to_openai_function_messages
from langchain.agents import create_openai_functions_agent, AgentExecutor

from langchain.prompts.pipeline import PipelinePromptTemplate
from langchain.prompts.prompt import PromptTemplate

from langchain_community.document_loaders import TextLoader
from langchain_openai import OpenAIEmbeddings

from langchain_community.vectorstores import FAISS
from langchain.text_splitter import (
    RecursiveCharacterTextSplitter,
    CharacterTextSplitter,
)

from langgraph.graph import StateGraph, END

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
    world_state_json_object = json.load(file)
    world_interface.load_world_from_json(world_state_json_object)

from_problem = parse_problem_init(problem=problem)
objects = parse_problem_objects(problem=problem)
world_interface.update_world(from_problem)

world_state_json = world_interface.get_world_to_json()
pprint(from_problem)
pprint(world_state_json)
pprint(objects)


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

####################### * tools
# bt_executor_tool = BehaviorTreeExecutorTool(behavior_tree_stewardship)

bt_sim_tool = BehaviorTreeSimulatorTool(metadata={"bt_stw": behavior_tree_stewardship})

# world_state_query_tool = WorldStateQueryTool(behavior_tree_stewardship)

executor_tools = [bt_sim_tool]

# * kios data prompt skeleton dir
data_dir = os.environ.get("KIOS_DATA_DIR").format(username=os.getlogin())
print(data_dir)
prompt_sk_dir = os.path.join(data_dir, "prompt_skeletons")
prompt_dir = os.path.join(data_dir, "prompts")


# * the graph state
class PlanExecuteState(TypedDict):
    plan: List[str]
    behavior_tree: dict
    world_state: Annotated[
        List[dict], operator.add
    ]  # ! to add, you need to make world_state a list of dict
    past_steps: Annotated[List[Tuple], operator.add]
    user_input: str
    objects: dict[str, list[str]]
    last_behavior_tree: dict
    last_failed_node: dict
    runtime_world_state: dict
    behavior_tree_execution_summary: str
    BTExecutionHasSucceeded: bool

    action_sequence: List[str]


########## verifier

template_file = os.path.join(prompt_dir, "seq_planner/template.txt")
task_file = os.path.join(prompt_dir, "seq_planner/task.txt")
system_file = os.path.join(prompt_dir, "seq_planner/system.txt")
domain_file = os.path.join(prompt_dir, "seq_planner/domain.txt")
with open(template_file, "r") as f:
    template_ppt = PromptTemplate.from_template(f.read())
with open(task_file, "r") as f:
    task_ppt = PromptTemplate.from_template(f.read())
with open(system_file, "r") as f:
    system_ppt = PromptTemplate.from_template(f.read())
with open(domain_file, "r") as f:
    domain_ppt = PromptTemplate.from_template(f.read())

full_template_ppt = ChatPromptTemplate.from_template(
    """{system}

    {task}

    {domain}

    {template}
    """
)

seq_planner_ppt_ppl = PipelinePromptTemplate(
    final_prompt=full_template_ppt,
    pipeline_prompts=[
        ("template", template_ppt),
        ("task", task_ppt),
        ("system", system_ppt),
        ("domain", domain_ppt),
    ],
)

seq_planner_chain = (
    seq_planner_ppt_ppl
    | ChatOpenAI(model="gpt-4-turbo-preview", temperature=0)
    | StrOutputParser()
)


# * output schema of the recursive_sk_generator
class SolutionBehaviortree(BaseModel):
    """the fixed behavior tree"""

    behavior_tree: dict = Field(
        description="the behavior tree that adheres the required format."
    )


behaviortree_file = os.path.join(prompt_dir, "rec_sk_gen/behaviortree.txt")
template_file = os.path.join(prompt_dir, "rec_sk_gen/template.txt")
task_file = os.path.join(prompt_dir, "rec_sk_gen/task.txt")
system_file = os.path.join(prompt_dir, "rec_sk_gen/system.txt")
object_file = os.path.join(prompt_dir, "rec_sk_gen/object.txt")
domain_file = os.path.join(prompt_dir, "rec_sk_gen/domain.txt")
example_file = os.path.join(prompt_dir, "rec_sk_gen/example.txt")
state_file = os.path.join(prompt_dir, "rec_sk_gen/state.txt")
with open(template_file, "r") as f:
    template_ppt = PromptTemplate.from_template(f.read())
with open(task_file, "r") as f:
    task_ppt = PromptTemplate.from_template(f.read())
with open(system_file, "r") as f:
    system_ppt = PromptTemplate.from_template(f.read())
with open(domain_file, "r") as f:
    domain_ppt = PromptTemplate.from_template(f.read())
with open(object_file, "r") as f:
    object_ppt = PromptTemplate.from_template(f.read())
with open(behaviortree_file, "r") as f:
    ppt_tmp = PromptTemplate.from_template("{input}")
    behaviortree_ppt = ppt_tmp.partial(input=f.read())

with open(state_file, "r") as f:
    ppt_tmp = PromptTemplate.from_template("{input}")
    state_ppt = ppt_tmp.partial(input=f.read())

with open(example_file, "r") as f:
    ppt_tmp = PromptTemplate.from_template("{input}")
    example_ppt = ppt_tmp.partial(input=f.read())
full_template_ppt = ChatPromptTemplate.from_template(
    """{system}

    {task}

    {domain}

    {state}

    {object}

    {behaviortree}

    {example}

    {template}
    """
)

re_sk_gen_ppt_ppl = PipelinePromptTemplate(
    final_prompt=full_template_ppt,
    pipeline_prompts=[
        ("template", template_ppt),
        ("task", task_ppt),
        ("system", system_ppt),
        ("domain", domain_ppt),
        ("object", object_ppt),
        ("behaviortree", behaviortree_ppt),
        ("example", example_ppt),
        ("state", state_ppt),
    ],
)

rec_sk_gen_chain = (
    re_sk_gen_ppt_ppl
    | ChatOpenAI(model="gpt-4-turbo-preview", temperature=0)
    | JsonOutputParser()
)

# recursive_sk_generator = create_structured_output_runnable(
#     SolutionBehaviortree,
#     ChatOpenAI(model="gpt-4-turbo-preview", temperature=0),
#     re_sk_gen_ppt_ppl,
#     mode="openai-json",
# )


##################################################### * planner
# * output schema of the planner
class Plan(BaseModel):
    """Plan to follow in future"""

    steps: List[str] = Field(
        description="a list of different steps to follow, should be in sorted order"
    )


template_file = os.path.join(prompt_dir, "planner/template.txt")
task_file = os.path.join(prompt_dir, "planner/task.txt")
system_file = os.path.join(prompt_dir, "planner/system.txt")
domain_file = os.path.join(prompt_dir, "planner/domain.txt")
with open(template_file, "r") as f:
    template_ppt = PromptTemplate.from_template(f.read())
with open(task_file, "r") as f:
    task_ppt = PromptTemplate.from_template(f.read())
with open(system_file, "r") as f:
    system_ppt = PromptTemplate.from_template(f.read())
with open(domain_file, "r") as f:
    domain_ppt = PromptTemplate.from_template(f.read())

full_template_ppt = ChatPromptTemplate.from_template(
    """{system}

    {task}

    {domain}

    {template}
    """
)

planner_ppt_ppl = PipelinePromptTemplate(
    final_prompt=full_template_ppt,
    pipeline_prompts=[
        ("template", template_ppt),
        ("task", task_ppt),
        ("system", system_ppt),
        ("domain", domain_ppt),
    ],
)

planner = create_structured_output_runnable(
    Plan, ChatOpenAI(model="gpt-4-turbo-preview", temperature=0), planner_ppt_ppl
)

##################################################### * plan_updaterner

template_file = os.path.join(prompt_dir, "plan_updater/template.txt")
task_file = os.path.join(prompt_dir, "plan_updater/task.txt")
system_file = os.path.join(prompt_dir, "plan_updater/system.txt")
domain_file = os.path.join(prompt_dir, "plan_updater/domain.txt")
with open(template_file, "r") as f:
    template_ppt = PromptTemplate.from_template(f.read())
with open(task_file, "r") as f:
    task_ppt = PromptTemplate.from_template(f.read())
with open(system_file, "r") as f:
    system_ppt = PromptTemplate.from_template(f.read())
with open(domain_file, "r") as f:
    domain_ppt = PromptTemplate.from_template(f.read())

full_template_ppt = ChatPromptTemplate.from_template(
    """{system}

    {task}

    {domain}

    {template}
    """
)

plan_updater_ppt_ppl = PipelinePromptTemplate(
    final_prompt=full_template_ppt,
    pipeline_prompts=[
        ("template", template_ppt),
        ("task", task_ppt),
        ("system", system_ppt),
        ("domain", domain_ppt),
    ],
)

plan_updater = create_structured_output_runnable(
    Plan, ChatOpenAI(model="gpt-4-turbo-preview", temperature=0), plan_updater_ppt_ppl
)
# ! create openai fn runnable has bug


##################################################### * graph node functions
@traceable(name="user_input_node_step")
async def user_input_step(state: PlanExecuteState):
    """
    get the input from the user
    """
    print(f"-----user_input_step-----")

    user_input = input("Your next instruction: ")

    return {
        "user_input": user_input,
    }


@traceable(name="sequence_generate_step")
async def sequence_generate_step(state: PlanExecuteState):
    """
    generate the sequence based on the instruction
    """
    print(f"-----sequence_generate_step-----")

    instruction = state["plan"][0]
    start_world_state = state["world_state"][-1]
    objects = state["objects"]

    action_sequence = await seq_planner_chain.ainvoke(
        {
            "objects": objects,
            "start_world_state": start_world_state,
            "user_instruction": instruction,
        }
    )

    return {
        "action_sequence": action_sequence,
    }


@traceable(name="behavior_tree_generate_step")
async def behavior_tree_generate_step(state: PlanExecuteState):
    """
    generate the behavior tree based on the instruction
    """
    print(f"-----behavior_tree_generate_step-----")

    instruction = state["plan"][0]
    if state["runtime_world_state"] is None:
        runtime_world_state = state["world_state"][-1]
    else:
        runtime_world_state = state["runtime_world_state"]
    objects = state["objects"]

    bt_skeleton = await rec_sk_gen_chain.ainvoke(
        {
            "objects": objects,
            "runtime_world_state": runtime_world_state,
            "last_behavior_tree": state["last_behavior_tree"],
            "last_failed_node": state["last_failed_node"],
            "summary": state["behavior_tree_execution_summary"],
            "action_sequence": state["action_sequence"],
        }
    )

    render_bt(bt_skeleton)

    pause = input("paused here")

    return {
        "last_behavior_tree": bt_skeleton,
        "BTExecutionHasSucceeded": False,
    }


@traceable(name="behavior_tree_execute_step")
async def behavior_tree_execute_step(state: PlanExecuteState):
    """
    execute the first step of the plan, append the result to the past steps
    """
    print(f"-----behavior_tree_execute_step-----")

    this_step = state["plan"][0]
    behavior_tree_skeleton = state["last_behavior_tree"]
    latest_world_state = state["world_state"][-1]

    # * first sim run
    tree_result, skeleton_json = behavior_tree_stewardship.sk_sim_run(
        world_state=latest_world_state, skeleton_json=behavior_tree_skeleton
    )

    pprint(tree_result.to_json())
    pause = input("paused here")

    # * check result
    if tree_result.result == "success":

        # * second fake run
        tree_result, skeleton_json = behavior_tree_stewardship.sk_fake_run(
            world_state=latest_world_state, skeleton_json=behavior_tree_skeleton
        )

        pprint(tree_result.to_json())
        pause = input("paused here")

        if tree_result.result == "success":
            return {
                "BTExecutionHasSucceeded": True,
                "past_steps": (
                    this_step,
                    tree_result.result,
                ),  # * only one because new plan will be generated and old steps are all removed
                "world_state": [tree_result.world_state],
                # * clear all
                "last_behavior_tree": None,
                "last_failed_node": None,
                "runtime_world_state": None,
                "behavior_tree_execution_summary": None,
            }
        else:
            return {
                "world_state": [tree_result.world_state],  # * real world state
                "last_behavior_tree": skeleton_json,
                "last_failed_node": {
                    "summary": tree_result.final_node.get("summary"),
                    "name": tree_result.final_node.get("name"),
                    "identifier": tree_result.final_node.get("identifier"),
                },
                "runtime_world_state": tree_result.world_state,
                "behavior_tree_execution_summary": tree_result.summary,
            }

    else:
        return {
            # ! do not change world state
            "last_behavior_tree": skeleton_json,
            "last_failed_node": {
                "summary": tree_result.final_node.get("summary"),
                "name": tree_result.final_node.get("name"),
                "identifier": tree_result.final_node.get("identifier"),
            },
            "runtime_world_state": tree_result.world_state,
            "behavior_tree_execution_summary": tree_result.summary,
        }


@traceable(name="planner_step")
async def planner_step(state: PlanExecuteState):
    """
    plan the steps based on user input and world state
    """
    print(f"-----plan_step-----")

    plan = await planner.ainvoke(
        {
            "user_input": state["user_input"],
            "world_state": state["world_state"],
        }
    )
    return {"plan": plan.steps}


@traceable(name="plan_updater_step")
async def plan_updater_step(state: PlanExecuteState):
    """
    if return a response, then success, response the use, end.
    otherwise, return the updated newplan (normally the same as the old plan with the first step popped out.
    """
    print(f"-----plan_updater_step-----")

    # ! BOOOM! BUG!
    output = await plan_updater.ainvoke(
        {
            "user_input": state["user_input"],
            "plan": state["plan"],
            "world_state": state["world_state"],
            "past_steps": state["past_steps"],
        }
    )
    # if isinstance(
    #     output, UpdaterResponse
    # ):  # * determine if it is time to response and end
    #     return {
    #         "response": "Your last instruction has been finished."
    #     }  # * Don't need to update.
    # else:
    return {"plan": output.steps}  # * update the plan


##################################################### * construct the graph

workflow = StateGraph(PlanExecuteState)

workflow.add_node("planner", planner_step)
workflow.add_node("sequence_generator", sequence_generate_step)
workflow.add_node("behavior_tree_generator", behavior_tree_generate_step)
workflow.add_node("behavior_tree_executor", behavior_tree_execute_step)
workflow.add_node("plan_updater", plan_updater_step)
workflow.add_node("user_input_node", user_input_step)
workflow.set_entry_point("user_input_node")
workflow.add_edge("planner", "sequence_generator")
workflow.add_edge("sequence_generator", "behavior_tree_generator")
workflow.add_edge("behavior_tree_generator", "behavior_tree_executor")


def executor_should_end(state: PlanExecuteState):
    """
    end router
    """
    print(f"-----executor_should_end-----")

    if state["BTExecutionHasSucceeded"] == True:
        state["BTExecutionHasSucceeded"] = False
        return True
    else:
        return False


workflow.add_conditional_edges(
    "behavior_tree_executor",
    executor_should_end,
    {
        True: "plan_updater",
        False: "behavior_tree_generator",
    },
)


def plan_updater_should_end(state: PlanExecuteState):
    """
    end router
    """
    print(f"-----plan_updater_should_end-----")

    if state["plan"] == [] or len(state["plan"]) == 0:
        return True
    else:
        return False


workflow.add_conditional_edges(
    "plan_updater",
    plan_updater_should_end,
    {
        True: "user_input_node",
        False: "behavior_tree_generator",
    },
)


def user_input_should_end(state: PlanExecuteState):
    """
    if the user input is empty, then end
    """
    print(f"-----user_input_should_end-----")

    if not state["user_input"] or state["user_input"] == "":
        return True
    else:
        return False


workflow.add_conditional_edges(
    "user_input_node",
    user_input_should_end,
    {
        True: END,
        False: "planner",
    },
)

app = workflow.compile()

config = {"recursion_limit": 500}

inputs = {
    "world_state": [world_state_json],
    "objects": objects,
}

# * unit tree generator ppl
system_file = os.path.join(prompt_dir, "new/ut_gen/system.txt")
task_file = os.path.join(prompt_dir, "new/ut_gen/task.txt")
domain_file = os.path.join(prompt_dir, "new/ut_gen/new_domain_nl.txt")
behaviortree_file = os.path.join(prompt_dir, "new/ut_gen/new_behaviortree.txt")
template_file = os.path.join(prompt_dir, "new/ut_gen/template.txt")
with open(template_file, "r") as f:
    template_ppt = PromptTemplate.from_template(f.read())
with open(task_file, "r") as f:
    task_ppt = PromptTemplate.from_template(f.read())
with open(system_file, "r") as f:
    system_ppt = PromptTemplate.from_template(f.read())
with open(domain_file, "r") as f:
    domain_ppt = PromptTemplate.from_template(f.read())
with open(behaviortree_file, "r") as f:
    ppt_tmp = PromptTemplate.from_template("{input}")
    behaviortree_ppt = ppt_tmp.partial(input=f.read())

full_template_ppt = ChatPromptTemplate.from_template(
    """{system}

    {task}

    {domain}

    {behaviortree}

    {template}

    {format_instructions}
    """
)

parser = JsonOutputParser()

format_instructions = PromptTemplate.from_template("""{input}""").partial(
    input=parser.get_format_instructions()
)

ut_gen_ppt_ppl = PipelinePromptTemplate(
    final_prompt=full_template_ppt,
    pipeline_prompts=[
        ("template", template_ppt),
        ("task", task_ppt),
        ("system", system_ppt),
        ("domain", domain_ppt),
        ("behaviortree", behaviortree_ppt),
        ("format_instructions", format_instructions),
    ],
)

ut_gen_chain = (
    ut_gen_ppt_ppl
    # | ChatOpenAI(model="gpt-3.5-turbo-0125", temperature=0)
    # | ChatOpenAI(model="ft:gpt-3.5-turbo-0125:kifabrik-mirmi::8y1cXwVw", temperature=0)
    | ChatOpenAI(
        model="ft:gpt-3.5-turbo-0125:kifabrik-mirmi:kios-ut-gen-v2:8z2KbPsr",
        temperature=0,
    )
    # | ChatOpenAI(model="gpt-4", temperature=0)
    | JsonOutputParser()
)

# ! sequence planner estimation ppl
system_file = os.path.join(prompt_dir, "seq_planner_est/system.txt")
task_file = os.path.join(prompt_dir, "seq_planner_est/task.txt")
domain_file = os.path.join(prompt_dir, "seq_planner_est/domain.txt")
state_file = os.path.join(prompt_dir, "seq_planner_est/state.txt")
output_format_file = os.path.join(prompt_dir, "seq_planner_est/output_format.txt")
template_file = os.path.join(prompt_dir, "seq_planner_est/template.txt")
example_file = os.path.join(prompt_dir, "seq_planner_est/example.txt")
with open(template_file, "r") as f:
    template_ppt = PromptTemplate.from_template(f.read())
with open(task_file, "r") as f:
    task_ppt = PromptTemplate.from_template(f.read())
with open(system_file, "r") as f:
    system_ppt = PromptTemplate.from_template(f.read())
with open(domain_file, "r") as f:
    domain_ppt = PromptTemplate.from_template(f.read())
with open(output_format_file, "r") as f:
    output_format_ppt = PromptTemplate.from_template(f.read())
with open(example_file, "r") as f:
    ppt_tmp = PromptTemplate.from_template("{input}")
    example_ppt = ppt_tmp.partial(input=f.read())
with open(state_file, "r") as f:
    ppt_tmp = PromptTemplate.from_template("{input}")
    state_ppt = ppt_tmp.partial(input=f.read())

full_template_ppt = ChatPromptTemplate.from_template(
    """{system}

    {task}

    {domain}

    {state}

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

seq_planner_est_ppt_ppl = PipelinePromptTemplate(
    final_prompt=full_template_ppt,
    pipeline_prompts=[
        ("template", template_ppt),
        ("task", task_ppt),
        ("system", system_ppt),
        ("domain", domain_ppt),
        ("state", state_ppt),
        ("output_format", output_format_ppt),
        ("format_instructions", format_instructions),
        ("example", example_ppt),
    ],
)

seq_planner_est_chain = (
    seq_planner_est_ppt_ppl
    # | ChatOpenAI(model="gpt-4", temperature=0)
    | ChatOpenAI(model="gpt-3.5-turbo-0125", temperature=0)
    | JsonOutputParser()
)


async def core_run():
    async for event in app.astream(
        inputs,
        config=config,
    ):
        for k, v in event.items():
            if k != "__end__":
                print(v)


def seq_planner_est_test():
    return seq_planner_est_chain.invoke(
        {
            "start_world_state": world_state_json,
            # "target": "is_inserted_to(shaft1, gearbase_hole1)",
            "target": "hold(left_hand, outward_claw)",
        }
    )


# * sequence action planner
system_file = os.path.join(prompt_dir, "new/seq_plan/system.txt")
task_file = os.path.join(prompt_dir, "new/seq_plan/task.txt")
domain_file = os.path.join(prompt_dir, "new/seq_plan/new_domain_nl.txt")
state_file = os.path.join(prompt_dir, "new/seq_plan/state.txt")
output_format_file = os.path.join(prompt_dir, "new/seq_plan/output_format.txt")
template_file = os.path.join(prompt_dir, "new/seq_plan/template.txt")
example_file = os.path.join(prompt_dir, "new/seq_plan/new_example.txt")
chain_file = os.path.join(prompt_dir, "new/seq_plan/chain.txt")
with open(template_file, "r") as f:
    template_ppt = PromptTemplate.from_template(f.read())
with open(task_file, "r") as f:
    task_ppt = PromptTemplate.from_template(f.read())
with open(system_file, "r") as f:
    system_ppt = PromptTemplate.from_template(f.read())
with open(domain_file, "r") as f:
    domain_ppt = PromptTemplate.from_template(f.read())
with open(output_format_file, "r") as f:
    output_format_ppt = PromptTemplate.from_template(f.read())
with open(example_file, "r") as f:
    ppt_tmp = PromptTemplate.from_template("{input}")
    example_ppt = ppt_tmp.partial(input=f.read())
with open(state_file, "r") as f:
    ppt_tmp = PromptTemplate.from_template("{input}")
    state_ppt = ppt_tmp.partial(input=f.read())
# # * this is for gpt 3.5
# with open(chain_file, "r") as f:
#     chain_ppt = PromptTemplate.from_template(f.read())

full_template_ppt = ChatPromptTemplate.from_template(
    """{system}

    {task}

    {domain}

    {state}

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

seq_ac_pl_ppt_ppl = PipelinePromptTemplate(
    final_prompt=full_template_ppt,
    pipeline_prompts=[
        ("template", template_ppt),
        ("task", task_ppt),
        ("system", system_ppt),
        ("domain", domain_ppt),
        ("state", state_ppt),
        ("output_format", output_format_ppt),
        ("format_instructions", format_instructions),
        ("example", example_ppt),
        # ("chain", chain_ppt),
    ],
)

seq_ac_pl_chain = (
    seq_ac_pl_ppt_ppl
    | ChatOpenAI(model="gpt-4", temperature=0)
    # | ChatOpenAI(model="gpt-3.5-turbo-0125", temperature=0)
    | JsonOutputParser()
)

# * state estimater
system_file = os.path.join(prompt_dir, "new/state_est/system.txt")
task_file = os.path.join(prompt_dir, "new/state_est/task.txt")
domain_file = os.path.join(prompt_dir, "new/state_est/new_domain_nl.txt")
state_file = os.path.join(prompt_dir, "new/state_est/state.txt")
output_format_file = os.path.join(prompt_dir, "new/state_est/output_format.txt")
template_file = os.path.join(prompt_dir, "new/state_est/template.txt")
example_file = os.path.join(prompt_dir, "new/state_est/new_example.txt")
with open(template_file, "r") as f:
    template_ppt = PromptTemplate.from_template(f.read())
with open(task_file, "r") as f:
    task_ppt = PromptTemplate.from_template(f.read())
with open(system_file, "r") as f:
    system_ppt = PromptTemplate.from_template(f.read())
with open(domain_file, "r") as f:
    domain_ppt = PromptTemplate.from_template(f.read())
with open(output_format_file, "r") as f:
    output_format_ppt = PromptTemplate.from_template(f.read())
with open(example_file, "r") as f:
    ppt_tmp = PromptTemplate.from_template("{input}")
    example_ppt = ppt_tmp.partial(input=f.read())
with open(state_file, "r") as f:
    ppt_tmp = PromptTemplate.from_template("{input}")
    state_ppt = ppt_tmp.partial(input=f.read())

full_template_ppt = ChatPromptTemplate.from_template(
    """{system}

    {task}

    {domain}

    {state}

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

state_est_ppt_ppl = PipelinePromptTemplate(
    final_prompt=full_template_ppt,
    pipeline_prompts=[
        ("template", template_ppt),
        ("task", task_ppt),
        ("system", system_ppt),
        ("domain", domain_ppt),
        ("state", state_ppt),
        ("output_format", output_format_ppt),
        ("format_instructions", format_instructions),
        ("example", example_ppt),
    ],
)


state_est_ppt_ppl_chain = (
    state_est_ppt_ppl
    # | ChatOpenAI(model="gpt-3.5-turbo", temperature=0)
    | ChatOpenAI(model="gpt-4", temperature=0)
    | JsonOutputParser()
)


async def core_run():
    async for event in app.astream(
        inputs,
        config=config,
    ):
        for k, v in event.items():
            if k != "__end__":
                print(v)


def ut_gen_test():
    ut = ut_gen_chain.invoke(
        {
            # "action": "insert(left_hand, defaultgripper, gear1, shaft1)",
            # "action": "insert(left_hand, parallel_box1, shaft1, gearbase_hole1)",
            # "action": "change_tool(left_hand, parallel_box1, defaultgripper)",
            "action": "insert(left_hand, clampgripper, gear2, shaft2)",
        }
    )

    render_bt(ut)


def seq_planner_est_test():
    return seq_planner_est_chain.invoke(
        {
            "start_world_state": world_state_json,
            "target": "is_inserted_to(shaft1, gearbase_hole1)",
            # "target": "hold(left_hand, outward_claw)",
        }
    )


def seq_action_plan_test():
    return seq_ac_pl_chain.invoke(
        {
            "start_world_state": world_state_json,
            "target": "is_inserted_to(shaft1, gearbase_hole1)",
            # "target": "hold(left_hand, outward_claw)",
        }
    )


def state_est_test():
    return state_est_ppt_ppl_chain.invoke(
        {
            "start_world_state": world_state_json,
            "action_plan": [
                "unload_tool(left_hand, outward_claw)",
                "load_tool(left_hand, parallel_box1)",
                "pick_up(left_hand, parallel_box1, shaft1)",
                "insert(left_hand, parallel_box1, shaft1, gearbase_hole1)",
            ],
        }
    )


##################################### * speed imp
@traceable(name="make_plan")
def make_plan(state: dict, goal: str) -> list[str]:
    print(f"----------start to make plan for the goal {goal}")
    response = seq_ac_pl_chain.invoke(
        {
            "start_world_state": state,
            "target": goal,
        }
    )
    print(f"finished making plan for the goal {goal}.")
    pprint(f'LLM thought flow: {response["explanation"]}')
    return response["task_plan"]


@traceable(name="estimate_state")
def estimate_state(start_world_state: dict, action_plan: list[str]) -> dict:
    print("----------start to estimate the state after the action plan:")
    pprint(action_plan)
    response = state_est_ppt_ppl_chain.invoke(
        {
            "start_world_state": start_world_state,
            "action_plan": action_plan,
        }
    )
    print(f"finished estimating the state after the action plan {action_plan}.")
    return response["estimated_world_state"]


@traceable(name="generate_unit_subtree")
def generate_unit_subtree(action: str) -> dict:
    print("----------start to generate the unit subtree for the action")
    pprint(action)
    response = ut_gen_chain.invoke(
        {
            "action": action,
        }
    )
    print(f"finished generating the unit subtree for the action {action}.")
    return response


def get_node_list_from_tree(unit_subtree: dict) -> list[dict]:
    children = unit_subtree["children"][1][
        "children"
    ]  # * the second child is a sequence
    return children


def extract_goal(node: dict) -> str:
    name = node["name"]


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


def embed_ut_nl(unit_subtree: dict) -> str:
    """
    embed the unit subtree into the overall tree
    """
    selector_children = unit_subtree["children"]
    target = ""
    # * target
    for node in selector_children:
        if match_type(node)[0] == "target":
            target += node["summary"]

    sequence_children = selector_children[1]["children"]
    preconditions = []
    action = ""
    for node in sequence_children:
        if match_type(node)[0] == "precondition":
            preconditions.append(node["summary"])
        if match_type(node)[0] == "action":
            action = node["summary"]

    embedding = f"if {' and '.join(preconditions)} then {action}, {target}"

    return embedding


def test_embedding_nl():
    unit_tree = ut_gen_test()
    pprint(unit_tree)
    nl = embed_ut_nl(unit_tree)
    print(nl)


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


if __name__ == "__main__":
    pass

    # asyncio.run(core_run())
    # pprint(ut_gen_test())
    # pprint(seq_action_plan_test())
    # pprint(state_est_test())

    test_expand_nodes()

    # test_embedding_nl()
