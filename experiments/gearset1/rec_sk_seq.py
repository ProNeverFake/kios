import json
import os
from pprint import pprint
from typing import List, Tuple, Annotated, TypedDict
import operator

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


def render_bt(bt_json: json):
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


async def core_run():
    async for event in app.astream(
        inputs,
        config=config,
    ):
        for k, v in event.items():
            if k != "__end__":
                print(v)


if __name__ == "__main__":
    import asyncio

    asyncio.run(core_run())
