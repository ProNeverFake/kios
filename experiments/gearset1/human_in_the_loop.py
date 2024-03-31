import json
import os
from pprint import pprint
from typing import List, Tuple, Annotated, TypedDict
import operator

os.environ["LANGCHAIN_TRACING_V2"] = "true"
os.environ["LANGCHAIN_ENDPOINT"] = "https://api.smith.langchain.com"
os.environ["LANGCHAIN_PROJECT"] = "trash"

from kios_bt.bt_stewardship import BehaviorTreeStewardship
from kios_scene.scene_factory import SceneFactory
from kios_bt.bt_factory import BehaviorTreeFactory
from kios_robot.robot_interface import RobotInterface
from kios_world.world_interface import WorldInterface

from kios_agent.llm_supporter import KiosLLMSupporter
from kios_agent.data_types import KiosPromptSkeleton

from kios_agent.kios_graph import (
    plan_updater,
    planner,
    seq_planner_chain,
    human_instruction_chain,
)

from kios_agent.kios_routers import KiosRouterFactory

from dotenv import load_dotenv

from langchain_openai import ChatOpenAI

from langchain.chains.openai_functions import (
    create_structured_output_runnable,
    create_openai_fn_runnable,
)

from langchain_core.prompts import ChatPromptTemplate, MessagesPlaceholder
from langchain_core.output_parsers import JsonOutputParser, StrOutputParser
from langchain_core.pydantic_v1 import BaseModel, Field

from langchain.prompts.pipeline import PipelinePromptTemplate
from langchain.prompts.prompt import PromptTemplate

from langgraph.graph import StateGraph, END

from langsmith import traceable

load_dotenv()

from kios_utils.pybt_test import generate_bt_stewardship, render_dot_tree


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
# domain_knowledge_path = os.path.join(current_dir, "domain_knowledge.txt")

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
# print(data_dir)
# prompt_sk_dir = os.path.join(data_dir, "prompt_skeletons")
prompt_dir = os.path.join(data_dir, "prompts")


# * the graph state
class PlanExecuteState(TypedDict):
    user_input: str  # this is for assembly planning
    plan: List[str]  # this is the assembly plan

    action_sequence: List[
        str
    ]  # this is the action sequence for only ONE step from the assembly plan
    behavior_tree: dict  # this is the behavior tree for the action sequence
    world_state: Annotated[
        List[dict], operator.add
    ]  # ! to add, you need to make world_state a list of dict
    past_steps: Annotated[
        List[Tuple], operator.add
    ]  # this is for update the assembly plan

    user_instruction: str  # this is for improve the behavior tree
    last_behavior_tree: dict  # record the last behavior tree for improvement
    last_failed_node: dict  # record the last failed node for improvement
    runtime_world_state: (
        dict  # record the world state after the last execution for diagnosis
    )

    BTExecutionHasSucceeded: bool  # this indicates if the last execution has succeeded


# ########## *sequential planner for BT
# region

# template_file = os.path.join(prompt_dir, "seq_planner/template.txt")
# task_file = os.path.join(prompt_dir, "seq_planner/task.txt")
# system_file = os.path.join(prompt_dir, "seq_planner/system.txt")
# domain_file = os.path.join(prompt_dir, "seq_planner/domain.txt")
# with open(template_file, "r") as f:
#     template_ppt = PromptTemplate.from_template(f.read())
# with open(task_file, "r") as f:
#     task_ppt = PromptTemplate.from_template(f.read())
# with open(system_file, "r") as f:
#     system_ppt = PromptTemplate.from_template(f.read())
# with open(domain_file, "r") as f:
#     domain_ppt = PromptTemplate.from_template(f.read())

# full_template_ppt = ChatPromptTemplate.from_template(
#     """{system}

#     {task}

#     {domain}

#     {template}
#     """
# )

# seq_planner_ppt_ppl = PipelinePromptTemplate(
#     final_prompt=full_template_ppt,
#     pipeline_prompts=[
#         ("template", template_ppt),
#         ("task", task_ppt),
#         ("system", system_ppt),
#         ("domain", domain_ppt),
#     ],
# )

# seq_planner_chain = (
#     seq_planner_ppt_ppl
#     | ChatOpenAI(model="gpt-4-turbo-preview", temperature=0)
#     | StrOutputParser()
# )
# endregion


# # * human instruction helped behavior tree generation
# region

# behaviortree_file = os.path.join(prompt_dir, "human_instruction/behaviortree.txt")
# template_file = os.path.join(prompt_dir, "human_instruction/template.txt")
# task_file = os.path.join(prompt_dir, "human_instruction/task.txt")
# system_file = os.path.join(prompt_dir, "human_instruction/system.txt")
# domain_file = os.path.join(prompt_dir, "human_instruction/domain.txt")
# with open(template_file, "r") as f:
#     template_ppt = PromptTemplate.from_template(f.read())
# with open(task_file, "r") as f:
#     task_ppt = PromptTemplate.from_template(f.read())
# with open(system_file, "r") as f:
#     system_ppt = PromptTemplate.from_template(f.read())
# with open(domain_file, "r") as f:
#     domain_ppt = PromptTemplate.from_template(f.read())
# with open(behaviortree_file, "r") as f:
#     ppt_tmp = PromptTemplate.from_template("{input}")
#     behaviortree_ppt = ppt_tmp.partial(input=f.read())

# # with open(state_file, "r") as f:
# #     ppt_tmp = PromptTemplate.from_template("{input}")
# #     state_ppt = ppt_tmp.partial(input=f.read())

# # with open(example_file, "r") as f:
# #     ppt_tmp = PromptTemplate.from_template("{input}")
# #     example_ppt = ppt_tmp.partial(input=f.read())
# full_template_ppt = ChatPromptTemplate.from_template(
#     """{system}

#     {task}

#     {domain}

#     {behaviortree}

#     {template}
#     """
# )

# human_instruction_ppt_ppl = PipelinePromptTemplate(
#     final_prompt=full_template_ppt,
#     pipeline_prompts=[
#         ("template", template_ppt),
#         ("task", task_ppt),
#         ("system", system_ppt),
#         ("domain", domain_ppt),
#         ("behaviortree", behaviortree_ppt),
#     ],
# )

# human_instruction_chain = (
#     human_instruction_ppt_ppl
#     | ChatOpenAI(model="gpt-4-turbo-preview", temperature=0)
#     | JsonOutputParser()
# )

# endregion


# ##################################################### * planner
# region
# # * output schema of the planner
# class Plan(BaseModel):
#     """Plan to follow in future"""

#     steps: List[str] = Field(
#         description="a list of different steps to follow, should be in sorted order"
#     )


# template_file = os.path.join(prompt_dir, "planner/template.txt")
# task_file = os.path.join(prompt_dir, "planner/task.txt")
# system_file = os.path.join(prompt_dir, "planner/system.txt")
# domain_file = os.path.join(prompt_dir, "planner/domain.txt")
# with open(template_file, "r") as f:
#     template_ppt = PromptTemplate.from_template(f.read())
# with open(task_file, "r") as f:
#     task_ppt = PromptTemplate.from_template(f.read())
# with open(system_file, "r") as f:
#     system_ppt = PromptTemplate.from_template(f.read())
# with open(domain_file, "r") as f:
#     domain_ppt = PromptTemplate.from_template(f.read())

# full_template_ppt = ChatPromptTemplate.from_template(
#     """{system}

#     {task}

#     {domain}

#     {template}
#     """
# )

# planner_ppt_ppl = PipelinePromptTemplate(
#     final_prompt=full_template_ppt,
#     pipeline_prompts=[
#         ("template", template_ppt),
#         ("task", task_ppt),
#         ("system", system_ppt),
#         ("domain", domain_ppt),
#     ],
# )

# planner = create_structured_output_runnable(
#     Plan, ChatOpenAI(model="gpt-4-turbo-preview", temperature=0), planner_ppt_ppl
# )

# ##################################################### * plan_updaterner

# template_file = os.path.join(prompt_dir, "plan_updater/template.txt")
# task_file = os.path.join(prompt_dir, "plan_updater/task.txt")
# system_file = os.path.join(prompt_dir, "plan_updater/system.txt")
# domain_file = os.path.join(prompt_dir, "plan_updater/domain.txt")
# with open(template_file, "r") as f:
#     template_ppt = PromptTemplate.from_template(f.read())
# with open(task_file, "r") as f:
#     task_ppt = PromptTemplate.from_template(f.read())
# with open(system_file, "r") as f:
#     system_ppt = PromptTemplate.from_template(f.read())
# with open(domain_file, "r") as f:
#     domain_ppt = PromptTemplate.from_template(f.read())

# full_template_ppt = ChatPromptTemplate.from_template(
#     """{system}

#     {task}

#     {domain}

#     {template}
#     """
# )

# plan_updater_ppt_ppl = PipelinePromptTemplate(
#     final_prompt=full_template_ppt,
#     pipeline_prompts=[
#         ("template", template_ppt),
#         ("task", task_ppt),
#         ("system", system_ppt),
#         ("domain", domain_ppt),
#     ],
# )

# plan_updater = create_structured_output_runnable(
#     Plan, ChatOpenAI(model="gpt-4-turbo-preview", temperature=0), plan_updater_ppt_ppl
# )
# ! create openai fn runnable has bug
# endregion


##################################################### * graph node functions
@traceable(name="user_input_node_step")
async def user_input_step(state: PlanExecuteState):
    """
    get the input from the user
    """
    print(f"-----user_input_step-----")

    user_input = input("Please provide your instructions:\n")

    return {
        "user_input": user_input,
    }


@traceable(name="sequence_generate_step")
async def sequence_generate_step(state: PlanExecuteState):
    """
    generate the sequence based on the instruction
    """
    print(f"-----sequence_generate_step-----")

    plan_goal = state["plan"][0]
    start_world_state = state["world_state"][-1]

    action_sequence = await seq_planner_chain.ainvoke(
        {
            "start_world_state": start_world_state,
            "user_instruction": plan_goal,
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
    user_instruction = state["user_instruction"]

    bt_skeleton = await human_instruction_chain.ainvoke(
        {
            "user_instruction": state["user_instruction"],
            "last_behavior_tree": state["last_behavior_tree"],
            "action_sequence": state["action_sequence"],
        }
    )

    # * the test is currently without sim
    # * first sim run
    # tree_result, skeleton_json = behavior_tree_stewardship.sk_sim_run(
    #     world_state=state["world_state"], skeleton_json=bt_skeleton
    # )

    # pprint(tree_result.to_json())

    render_bt(bt_skeleton)

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

    behavior_tree_stewardship.set_world_state(latest_world_state)

    behavior_tree_stewardship.generate_behavior_tree_from_skeleton(
        behavior_tree_skeleton
    )

    tree_result = behavior_tree_stewardship.tick_tree()

    pprint(tree_result.to_json())
    pause = input("DEBUG: please check the tree result. Press enter to continue.")

    # * check result
    if tree_result.result == "success":
        return {
            "BTExecutionHasSucceeded": True,
            "past_steps": (
                this_step,
                tree_result.result,
            ),  # * only one because new plan will be generated and old steps are all removed
            "world_state": [tree_result.world_state],
            # "last_behavior_tree": None, # ! do not change the last behavior tree
            "runtime_world_state": tree_result.world_state,  # * this is world_state for successful execution
        }
    else:
        return {
            "BTExecutionHasSucceeded": False,
            # "last_behavior_tree": skeleton_json, # * do not change the last behavior tree
            "world_state": [tree_result.world_state],
            "runtime_world_state": tree_result.world_state,
        }


# region * sim
# @traceable(name="behavior_tree_execute_step")
# async def behavior_tree_execute_step(state: PlanExecuteState):
#     """
#     execute the first step of the plan, append the result to the past steps
#     """
#     print(f"-----behavior_tree_execute_step-----")

#     this_step = state["plan"][0]
#     behavior_tree_skeleton = state["last_behavior_tree"]
#     latest_world_state = state["world_state"][-1]

#     # * first sim run
#     tree_result, skeleton_json = behavior_tree_stewardship.sk_sim_run(
#         world_state=latest_world_state, skeleton_json=behavior_tree_skeleton
#     )

#     pprint(tree_result.to_json())
#     pause = input("paused here")

#     # * check result
#     if tree_result.result == "success":
#         return {
#             "BTExecutionHasSucceeded": True,
#             "past_steps": (
#                 this_step,
#                 tree_result.result,
#             ),  # * only one because new plan will be generated and old steps are all removed
#             "world_state": [tree_result.world_state],
#             # "last_behavior_tree": None, # ! do not change the last behavior tree
#             "runtime_world_state": tree_result.world_state,  # * this is world_state for successful execution
#         }
#     else:
#         return {
#             "BTExecutionHasSucceeded": False,
#             # ! do not change world state
#             # "last_behavior_tree": skeleton_json, # * do not change the last behavior tree
#             "world_state": [tree_result.world_state],
#             "runtime_world_state": tree_result.world_state,
#         }
# endregion


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

router_factory = KiosRouterFactory()

user_instruction_router = router_factory.create_router_layer(
    route_names=["rectify", "approve"]
)


def user_instruction_should_end(state: PlanExecuteState):
    """
    router for user hint.
    """
    print(f"-----user_instruction_should_end-----")
    user_instruction = input(
        "What should I do to improve the behavior tree?\nPlease give me your hint: "
    )
    # * you only update the user_instruction here once
    state["user_instruction"] = user_instruction

    if user_instruction == "" or not user_instruction:
        state["user_instruction"] = None
        return True

    while True:
        route = user_instruction_router(user_instruction)
        if route.name == None:
            user_instruction = input(
                "I don't understand your intention. Can you explain do you want me to execute the plan, or improve the behavior tree?"
            )
        else:
            break

    if route.name == "approve":
        state["user_instruction"] = None  # * clear the user instruction
        return True  # go to exectuor
    elif route.name == "rectify":
        return False  # go back to generator
    else:
        raise ValueError(f"Route {route.name} not supported!")


workflow.add_conditional_edges(
    "behavior_tree_generator",
    user_instruction_should_end,
    {
        True: "behavior_tree_executor",
        False: "behavior_tree_generator",
    },
)

executor_router = router_factory.create_router_layer(
    route_names=[
        "finish",
        "rectify",
        "approve",
        "disapprove",
    ]
)


def executor_should_end(state: PlanExecuteState):
    """
    end router
    """
    print(f"-----executor_should_end-----")

    if state["BTExecutionHasSucceeded"] == True:
        # ask for user confirmation and end, or go back to the behavior tree generator if the user wants to improve
        user_instruction = input(
            "the behavior tree has succeeded.\n Is the target of this step satisfied now? Is there anything wrong?"
        )
        state["user_instruction"] = user_instruction

        if user_instruction == "" or not user_instruction:
            state["user_instruction"] = None
            state["last_behavior_tree"] = None
            state["BTExecutionHasSucceeded"] = False
            return True

        while True:
            route = executor_router(user_instruction)
            if route.name == None:
                user_instruction = input(
                    "I don't understand your intention. Can you explain is the target satisfied, or is there something wrong?"
                )
            else:
                break

        if route.name in ["finish", "approve"]:
            # * clear the states for this step
            state["user_instruction"] = None
            state["last_behavior_tree"] = None
            state["BTExecutionHasSucceeded"] = False
            return True
        elif route.name in ["rectify", "disapprove"]:
            return False
        else:
            raise ValueError(f"Route {route.name} not supported!")
    else:
        # ask for user hint and go back to the behavior tree generator
        # * router is unnecessary here
        user_instruction = input(
            "The behavior tree has failed in its execution.\nPlease give me a hint to improve it:"
        )
        state["user_instruction"] = user_instruction
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
        print("The assembly plan has been finished.")
        return True
    else:
        print("The assembly plan has not been finished.")
        print(f'Unfinished steps: {state["plan"]}')
        return False


workflow.add_conditional_edges(
    "plan_updater",
    plan_updater_should_end,
    {
        True: "user_input_node",
        False: "sequence_generator",  # ! BUG generate the sequence first
    },
)

user_input_router = router_factory.create_router_layer(
    route_names=["finish", "instruction"]
)


def user_input_should_end(state: PlanExecuteState):
    """
    if the user input is empty, then end
    """
    print(f"-----user_input_should_end-----")

    if not state["user_input"] or state["user_input"] == "":
        return True

    while True:
        route = user_input_router(state["user_input"])
        if route.name == None:
            user_input = input(
                "I don't understand your instruction. Can you provide me with a new instruction?"
            )
        else:
            break

    if route.name == "finish":
        return True
    elif route.name == "instruction":
        return False
    else:
        raise ValueError(f"Route {route.name} not supported!")


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
