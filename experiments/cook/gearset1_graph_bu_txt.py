import json
import os

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

from typing import List, Tuple, Annotated, TypedDict
import operator

from dotenv import load_dotenv

from langchain_openai import ChatOpenAI

from langchain.chains.openai_functions import (
    create_structured_output_runnable,
    create_openai_fn_runnable,
)

from langchain_core.prompts import ChatPromptTemplate, MessagesPlaceholder
from langchain_core.output_parsers import JsonOutputParser, StrOutputParser
from langchain_core.pydantic_v1 import BaseModel, Field

from langchain.agents.format_scratchpad import format_to_openai_function_messages
from langchain.agents import create_openai_functions_agent, AgentExecutor

from langchain.prompts.pipeline import PipelinePromptTemplate
from langchain.prompts.prompt import PromptTemplate

from langgraph.graph import StateGraph, END

from langsmith import traceable

load_dotenv()


####################### dirs
current_dir = os.path.dirname(os.path.abspath(__file__))
scene_path = os.path.join(current_dir, "scene.json")
# bt_json_file_path = os.path.join(current_dir, "behavior_tree.json")
world_state_path = os.path.join(current_dir, "world_state.json")
problem_path = os.path.join(current_dir, "gearset.problem")


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

# * kios data prompt skeleton dir
data_dir = os.environ.get("KIOS_DATA_DIR").format(username=os.getlogin())
print(data_dir)
prompt_sk_dir = os.path.join(data_dir, "prompt_skeletons")
prompt_dir = os.path.join(data_dir, "prompts")

# * refiner chain
llm_spt_re_sk = KiosLLMSupporter()
with open(os.path.join(prompt_sk_dir, "skeleton_refiner.json"), "r") as f:
    re_sk_pptsk_json = json.load(f)
    re_sk_pptsk = KiosPromptSkeleton.from_json(re_sk_pptsk_json)
    llm_spt_re_sk.initialize_from_prompt_skeleton(re_sk_pptsk)
re_sk_ppt = llm_spt_re_sk.create_prompt()
re_sk_llm = ChatOpenAI(model_name=llm_spt_re_sk.model_name, temperature=0)

re_sk_chain = re_sk_ppt | re_sk_llm | JsonOutputParser()


# * skeleton generator chain
llm_spt_skeleton_generator = KiosLLMSupporter()
with open(os.path.join(prompt_sk_dir, "skeleton_generator.json"), "r") as f:
    skeleton_generator_pptsk_json = json.load(f)
    skeleton_generator_pptsk = KiosPromptSkeleton.from_json(
        skeleton_generator_pptsk_json
    )
    llm_spt_skeleton_generator.initialize_from_prompt_skeleton(skeleton_generator_pptsk)
skeleton_generator_ppt = llm_spt_skeleton_generator.create_prompt()
skeleton_generator_llm = ChatOpenAI(
    model_name=llm_spt_skeleton_generator.model_name, temperature=0
)

skeleton_generator_chain = (
    skeleton_generator_ppt
    | skeleton_generator_llm
    # | StrOutputParser()
    | JsonOutputParser()
)


# * the graph state
class PlanExecuteState(TypedDict):
    plan: List[str]
    behavior_tree_skeleton: dict  # ! not sure if use this or not.
    behavior_tree: dict
    world_state: Annotated[
        List[dict], operator.add
    ]  # ! to add, you need to make world_state a list of dict
    # = Field(default=None)
    past_steps: Annotated[List[Tuple], operator.add]
    response: str  # ! not sure if use this or not.
    # to_user: bool
    user_input: str
    problem: str


##################################################### * planner
# * output schema of the planner
class Plan(BaseModel):
    """Plan to follow in future"""

    steps: List[str] = Field(
        description="a list of different steps to follow, should be in sorted order"
    )


planner_prompt_file = os.path.join(prompt_dir, "planner/template.txt")

with open(planner_prompt_file, "w") as f:
    planner_prompt = f.read()

planner = create_structured_output_runnable(
    Plan, ChatOpenAI(model="gpt-4-turbo-preview", temperature=0), planner_prompt
)


##################################################### * plan_updaterner


class UpdaterResponse(BaseModel):
    """used to response to the user for asking for more inputs."""

    response: str


plan_updater_prompt = ChatPromptTemplate.from_template(
    """For the given objective, come up with a simple step by step plan. \
This plan should involve individual tasks, that if executed correctly will yield the correct answer. Do not add any superfluous steps. \
The result of the final step should be the final answer. Make sure that each step has all the information needed - do not skip steps.

Your objective was this:
{user_input}

Your original plan was this:
{plan}

You have currently done the follow steps:
{past_steps}

The world state is:
{world_state}

Read the world state. Check the original plan from the first step and update your plan with the steps need to be conducted next according to the world state.\
If no more steps are needed to achieve the objective defined by user input, respond to the user for more input. Otherwise, fill out the plan.\
Only add steps to the plan that still NEED to be done. Do not return previously done steps as part of the plan."""
)

plan_updater = create_openai_fn_runnable(
    [Plan, UpdaterResponse],  # * here two schemas are used
    ChatOpenAI(model="gpt-4-turbo-preview", temperature=0),
    plan_updater_prompt,
)


##################################################### * graph node functions
@traceable(name="user_input_node_step")
async def user_input_step(state: PlanExecuteState):
    """
    get the input from the user
    """
    print(f"-----user_input_step-----")
    if state["response"] is not None:
        user_input = input(state["response"])
        state["response"] = None
    else:
        user_input = input("Your next instruction: ")

    return {
        "user_input": user_input,
    }


@traceable(name="behavior_tree_generate_step")
async def behavior_tree_generate_step(state: PlanExecuteState):
    """
    generate the behavior tree based on the instruction
    """
    print(f"-----behavior_tree_generate_step-----")

    instruction = state["plan"][0]
    latest_world_state = state["world_state"][-1]

    skeleton = await skeleton_generator_chain.ainvoke(
        {
            "problem": state["problem"],
            "world_state": latest_world_state,
            "instructions": instruction,
        }
    )

    # ! OBJECT CONCENTRATION

    response = await re_sk_chain.ainvoke(
        {
            "input": skeleton,
        }
    )

    behavior_tree = response.get("task_plan").get("behavior_tree")

    init_world_state = [response.get("initial_state")]

    return {
        "behavior_tree": behavior_tree,
        "world_state": init_world_state,  # ! this is necessary because the constraints in problem
    }


# ! BBWARN INITIALIZE THE CONSTRAINT IN THE PROMPT OR IN THE INITIALIZATION NODE


@traceable(name="behavior_tree_execute_step")
async def behavior_tree_execute_step(state: PlanExecuteState):
    """
    execute the first step of the plan, append the result to the past steps
    """
    print(f"-----behavior_tree_execute_step-----")

    behavior_tree = state["behavior_tree"]
    latest_world_state = state["world_state"][-1]

    response = behavior_tree_stewardship.fake_run(
        world_state=latest_world_state, bt_json=behavior_tree
    )

    hasSucceeded = response.result
    new_world_state = [response.world_state]
    the_step = state["plan"][0]

    return {
        "past_steps": (the_step, hasSucceeded),
        "world_state": new_world_state,
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

    output = await plan_updater.ainvoke(state)
    if isinstance(
        output, UpdaterResponse
    ):  # * determine if it is time to response and end
        return {
            "response": "Your last instruction has been finished."
        }  # * Don't need to update.
    else:
        return {"plan": output.steps}  # * update the plan


##################################################### * construct the graph

workflow = StateGraph(PlanExecuteState)

workflow.add_node("planner", planner_step)

workflow.add_node("behavior_tree_generator", behavior_tree_generate_step)

workflow.add_node("behavior_tree_executor", behavior_tree_execute_step)

workflow.add_node("plan_updater", plan_updater_step)

workflow.add_node("user_input_node", user_input_step)

workflow.set_entry_point("user_input_node")

# workflow.add_edge("user_input_node", "planner")

workflow.add_edge(
    "planner", "behavior_tree_generator"
)  # ! may need to add a condition here

workflow.add_edge("behavior_tree_generator", "behavior_tree_executor")

workflow.add_edge("behavior_tree_executor", "plan_updater")


def plan_updater_should_end(state: PlanExecuteState):
    """
    end router
    """
    print(f"-----plan_updater_should_end-----")

    if state["response"]:
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

    # print(f'user_input: {state["user_input"]}')
    # print(f'world_state: {state["world_state"]}')

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


# config = {"recursion_limit": 50}

inputs = {
    "world_state": [world_state_json_object],
    "problem": problem,
}


async def core_run():
    async for event in app.astream(
        inputs,
        # config=config,
    ):
        for k, v in event.items():
            if k != "__end__":
                print(v)


if __name__ == "__main__":
    import asyncio

    asyncio.run(core_run())
