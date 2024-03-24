import json
import os
from pprint import pprint

"""
test skeleton behavior tree generation.
the btw data structure should be updated.
"""

os.environ["LANGCHAIN_TRACING_V2"] = "true"
os.environ["LANGCHAIN_ENDPOINT"] = "https://api.smith.langchain.com"
os.environ["LANGCHAIN_PROJECT"] = "default"

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

from typing import List, Tuple, Annotated, TypedDict
import operator

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

from langchain.prompts.pipeline import PipelinePromptTemplate
from langchain.prompts.prompt import PromptTemplate

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
    behavior_tree: dict
    world_state: Annotated[
        List[dict], operator.add
    ]  # ! to add, you need to make world_state a list of dict
    # = Field(default=None)
    past_steps: Annotated[List[Tuple], operator.add]
    response: str  # ! not sure if use this or not.
    user_input: str
    problem: str
    objects: dict[str, list[str]]


# * output schema of the planner
class SolutionSubtree(BaseModel):
    """solution subtree that will be used to replace the failed condition node in the last behavior tree"""

    subtree: dict = Field(
        description="a subtree that adheres the required format, which will be used to replace the failed condition node in the last behavior tree"
    )


behaviortree_file = os.path.join(prompt_dir, "rec_sk_gen/behaviortree.txt")
template_file = os.path.join(prompt_dir, "rec_sk_gen/template.txt")
task_file = os.path.join(prompt_dir, "rec_sk_gen/task.txt")
system_file = os.path.join(prompt_dir, "rec_sk_gen/system.txt")
object_file = os.path.join(prompt_dir, "rec_sk_gen/object.txt")
domain_file = os.path.join(prompt_dir, "rec_sk_gen/domain.txt")
example_file = os.path.join(prompt_dir, "rec_sk_gen/example.txt")
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

    {behaviortree}

    {domain}

    {example}

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
    PlanExecuteState,
    ChatOpenAI(model="gpt-4-turbo-preview", temperature=0),
    planner_ppt_ppl,
)


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
    # if state["response"] is not None:
    #     user_input = input(state["response"])
    #     state["response"] = None
    # else:
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
    objects = state["objects"]

    skeleton = await skeleton_generator_chain.ainvoke(
        {
            # "problem": state["problem"],
            "objects": objects,
            "world_state": latest_world_state,
            "instructions": instruction,
        }
    )

    # sk_json = JsonOutputParser().invoke(skeleton)
    sk_json = skeleton

    behavior_tree_sk = sk_json["task_plan"]["behavior_tree"]

    render_bt(behavior_tree_sk)

    pause = input("paused here")

    # ! OBJECT CONCENTRATION

    # response = await re_sk_chain.ainvoke(
    #     {
    #         "input": skeleton,
    #     }
    # )

    behavior_tree = skeleton.get("task_plan").get("behavior_tree")

    # init_world_state = [skeleton.get("initial_state")]

    return {
        "behavior_tree": behavior_tree,
        # "world_state": init_world_state,  # ! this is necessary because the constraints in problem
    }


@traceable(name="behavior_tree_sim_step")
async def behavior_tree_sim_step(state: PlanExecuteState):
    """
    simulate the behavior tree based on the world state
    """
    print(f"-----behavior_tree_sim_step-----")

    behavior_tree = state["behavior_tree"]
    latest_world_state = state["world_state"][-1]

    # exe_input = {
    #     "behavior_tree": behavior_tree,
    #     "world_state": latest_world_state,
    # }

    # agent_response = await bt_sim_tool.ainvoke(
    #     {
    #         "input": exe_input,
    #     }
    # )

    response = behavior_tree_stewardship.sk_fake_run(
        world_state=latest_world_state, bt_json=behavior_tree
    )

    # hasSucceeded = response["hasSucceeded"]
    # new_world_state = [response["world_state"]]
    hasSucceeded = response.result
    new_world_state = [response.world_state]
    the_step = state["plan"][0]

    return {
        "past_steps": (
            the_step,
            hasSucceeded,
        ),  # * only one because new plan will be generated and old steps are all removed
        "world_state": new_world_state,
    }


@traceable(name="behavior_tree_execute_step")
async def behavior_tree_execute_step(state: PlanExecuteState):
    """
    execute the first step of the plan, append the result to the past steps
    """
    print(f"-----behavior_tree_execute_step-----")

    behavior_tree = state["behavior_tree"]
    latest_world_state = state["world_state"][-1]

    # exe_input = {
    #     "behavior_tree": behavior_tree,
    #     "world_state": latest_world_state,
    # }

    # agent_response = await bt_exe_agent_executor.ainvoke(
    #     {
    #         "input": exe_input,
    #     }
    # )

    response = behavior_tree_stewardship.sk_fake_run(
        world_state=latest_world_state, bt_json=behavior_tree
    )

    # hasSucceeded = response["hasSucceeded"]
    # new_world_state = [response["world_state"]]
    hasSucceeded = response.result
    new_world_state = [response.world_state]
    the_step = state["plan"][0]

    return {
        "past_steps": (
            the_step,
            hasSucceeded,
        ),  # * only one because new plan will be generated and old steps are all removed
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

workflow.add_node("behavior_tree_generator", behavior_tree_generate_step)

workflow.add_node("behavior_tree_executor", behavior_tree_execute_step)

workflow.add_node("plan_updater", plan_updater_step)

workflow.add_node("user_input_node", user_input_step)

workflow.set_entry_point("user_input_node")

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

    # if state["response"]:
    #     return True
    # else:
    #     return False
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
