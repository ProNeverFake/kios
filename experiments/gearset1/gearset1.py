import json
import os

from kios_bt_planning.kios_bt.bt_stewardship import BehaviorTreeStewardship
from kios_bt_planning.kios_scene.scene_factory import SceneFactory
from kios_bt_planning.kios_bt.bt_factory import BehaviorTreeFactory
from kios_bt_planning.kios_robot.robot_interface import RobotInterface
from kios_bt_planning.kios_world.world_interface import WorldInterface
from kios_bt_planning.kios_utils.pybt_test import (
    generate_bt_stewardship,
    render_dot_tree,
    tick_loop_test,
    tick_1000HZ_test,
    tick_frequency_test,
)
from kios_bt_planning.kios_agent.llm_supporter import KiosLLMSupporter
from kios_bt_planning.kios_agent.data_types import KiosPromptSkeleton

from kios_bt_planning.kios_agent.kios_tools import (
    BehaviorTreeExecutorTool,
    BehaviorTreeSimulatorTool,
    WorldStateQueryTool,
)

from langchain_core.pydantic_v1 import BaseModel, Field
from typing import List, Tuple, Annotated, TypedDict
import operator

from dotenv import load_dotenv

from langchain import hub
from langchain.agents import create_openai_functions_agent
from langchain_openai import ChatOpenAI
from langchain_core.prompts import ChatPromptTemplate
from langchain_core.output_parsers import JsonOutputParser

load_dotenv()

####################### dirs
current_dir = os.path.dirname(os.path.abspath(__file__))
scene_path = os.path.join(current_dir, "scene.json")
# bt_json_file_path = os.path.join(current_dir, "behavior_tree.json")
world_state_path = os.path.join(current_dir, "world_state.json")
problem_path = os.path.join(current_dir, "gearset.problem")

####################### scene
with open(scene_path, "r") as file:
    json_object = json.load(file)

scene = SceneFactory().create_scene_from_json(json_object)

####################### world
world_interface = WorldInterface()
with open(world_state_path, "r") as file:
    json_object = json.load(file)
    world_interface.load_world_from_json(json_object)

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

# with open(bt_json_file_path, "r") as file:
#     json_object = json.load(file)
#     behavior_tree_stewardship.load_bt_json(json_object)

# behavior_tree_stewardship.generate_behavior_tree()
# behavior_tree_stewardship.setup_behavior_tree()
# behavior_tree_stewardship.render_dot_tree()

####################### tools
# bt_executor_tool = BehaviorTreeExecutorTool(behavior_tree_stewardship)

bt_simulator_tool = BehaviorTreeSimulatorTool(behavior_tree_stewardship)

# world_state_query_tool = WorldStateQueryTool(behavior_tree_stewardship)

executor_tools = [bt_simulator_tool]

# * kios data prompt skeleton dir
data_dir = os.environ.get("KIOS_DATA_DIR").format(username=os.getlogin())
print(data_dir)
prompt_sk_dir = os.path.join(data_dir, "prompt_skeletons")

# * refiner chain
llm_spt_re_sk = KiosLLMSupporter()
with open(os.path.join(prompt_sk_dir, "skeleton_refiner.json"), "r") as f:
    re_sk_pptsk_json = json.load(f)
    re_sk_pptsk = KiosPromptSkeleton.from_json(re_sk_pptsk_json)
    llm_spt_re_sk.initialize_from_prompt_skeleton(re_sk_pptsk)
re_sk_ppt = llm_spt_re_sk.create_prompt()
re_sk_llm = ChatOpenAI(
    model_name=llm_spt_re_sk.prompt_skeleton.model_name, temperature=0
)

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
    model_name=llm_spt_skeleton_generator.prompt_skeleton.model_name, temperature=0
)

skeleton_generator_chain = (
    skeleton_generator_ppt | skeleton_generator_llm | JsonOutputParser()
)

# * behavior tree generator chain
behavior_tree_generator_chain = skeleton_generator_chain | re_sk_chain

############################################### * behavior tree executor agent
executor_ppt = hub.pull("hwchase17/openai-functions-agent")

executor_llm = ChatOpenAI(model="gpt-3.5-turbo", temperature=0)

executor_agent_runnable = create_openai_functions_agent(
    executor_llm, executor_tools, executor_ppt
)
from langgraph.prebuilt import create_agent_executor

executor_agent_executor = create_agent_executor(executor_agent_runnable, executor_tools)


# * the graph state
class PlanExecuteState(TypedDict):
    input: str
    plan: List[str]
    world_state: List[dict] = Field(default=None)
    past_steps: Annotated[List[Tuple], operator.add]
    response: str
    feedback: str


##################################################### * planner
# * output schema of the planner
class Plan(BaseModel):
    """Plan to follow in future"""

    steps: List[str] = Field(
        description="different steps to follow, should be in sorted order"
    )


from langchain.chains.openai_functions import create_structured_output_runnable
from langchain_core.prompts import ChatPromptTemplate

planner_prompt = ChatPromptTemplate.from_template(
    """For the given nature language instruction, come up with a simple step by step plan. \
This plan should involve individual tasks, that if executed correctly will yield the correct answer. Do not add any superfluous steps. \
The result of the final step should be the final answer. Make sure that each step has all the information needed - do not skip steps.

problem: {instruction}\
world_state: {world_state}\
instructions: {instructions}\
"""
)
planner = create_structured_output_runnable(
    Plan, ChatOpenAI(model="gpt-4-turbo-preview", temperature=0), planner_prompt
)


##################################################### * plan_updaterner
from langchain.chains.openai_functions import create_openai_fn_runnable


class Response(BaseModel):
    """Response to user."""

    response: str


plan_updater_prompt = ChatPromptTemplate.from_template(
    """For the given objective, come up with a simple step by step plan. \
This plan should involve individual tasks, that if executed correctly will yield the correct answer. Do not add any superfluous steps. \
The result of the final step should be the final answer. Make sure that each step has all the information needed - do not skip steps.

Your objective was this:
{input}

Your original plan was this:
{plan}

You have currently done the follow steps:
{past_steps}

Update your plan accordingly. If no more steps are needed and you can return to the user, then respond with that. Otherwise, fill out the plan. Only add steps to the plan that still NEED to be done. Do not return previously done steps as part of the plan."""
)

plan_updater = create_openai_fn_runnable(
    [Plan, Response],  # * here two schemas are used
    ChatOpenAI(model="gpt-4-turbo-preview", temperature=0),
    plan_updater_prompt,
)


##################################################### * graph node functions
async def feedback_step(state: PlanExecuteState):
    """
    get the feedback from the user
    """

    feedback = input("Your next instruction: ")

    return {"feedback": feedback}


def feedback_should_end(state: PlanExecuteState):
    """
    end router
    """
    if state["feedback"]:
        return True
    else:
        return False


async def behavior_tree_generate_step(state: PlanExecuteState):
    """
    generate the behavior tree based on the instruction
    """
    response = await behavior_tree_generator_chain.ainvoke(
        {"input": state["input"], "chat_history": []}
    )

    behavior_tree = await behavior_tree_generator_chain.ainvoke(
        {"input": state["input"], "chat_history": []}
    )
    return {"behavior_tree": behavior_tree}


async def execute_step(state: PlanExecuteState):
    """
    execute the first step of the plan, append the result to the past steps
    """
    behavior_tree = state["behavior_tree"]

    agent_response = await executor_agent_executor.ainvoke(
        {"input": aaa, "chat_history": []}
    )
    return {
        "past_steps": (task, agent_response["agent_outcome"].return_values["output"])
    }


async def plan_step(state: PlanExecuteState):
    """
    plan the steps based on user input
    """
    plan = await planner.ainvoke(
        {
            "objective": state["instructions"],
            "world_state": state["world_state"],
        }
    )
    return {"plan": plan.steps}


async def plan_updater_step(state: PlanExecuteState):
    """
    if return a response, then success, response the use, end.
    otherwise, return the updated newplan (normally the same as the old plan with the first step popped out.
    """
    output = await plan_updater.ainvoke(state)
    if isinstance(output, Response):  # * determine if it is time to response and end
        return {"response": output.response}
    else:
        return {"plan": output.steps}


def should_end(state: PlanExecuteState):
    """
    end router
    """
    if state["response"]:
        return True
    else:
        return False


##################################################### * construct the graph
from langgraph.graph import StateGraph, END

workflow = StateGraph(PlanExecuteState)

# Add the plan node
workflow.add_node("planner", plan_step)

workflow.add_node("behavior_tree_generator", behavior_tree_generate_step)
# Add the execution step
workflow.add_node("behavior_tree_executor", execute_step)

# Add a plan_updater node
workflow.add_node("plan_updater", plan_updater_step)

workflow.add_node("feedback", feedback_step)

workflow.set_entry_point("planner")

# From plan we go to agent
workflow.add_edge("planner", "behavior_tree_generator")

workflow.add_edge("behavior_tree_generator", "behavior_tree_executor")

# From agent, we plan_updater
workflow.add_edge("behavior_tree_executor", "plan_updater")

workflow.add_conditional_edges(
    "plan_updater",
    # Next, we pass in the function that will determine which node is called next.
    should_end,
    {
        # If `tools`, then we call the tool node.
        True: END,
        False: "agent",
    },
)

workflow.add_conditional_edges(
    "feedback",
    feedback_should_end,
    {
        True: END,
        False: "planner",
    },
)

# compiles it into a LangChain Runnable,
app = workflow.compile()
