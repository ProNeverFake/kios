import json
import os
from pprint import pprint
from typing import List, Tuple, Annotated, TypedDict
import operator
from dotenv import load_dotenv
import datetime

os.environ["LANGCHAIN_TRACING_V2"] = "true"
os.environ["LANGCHAIN_ENDPOINT"] = "https://api.smith.langchain.com"
os.environ["LANGCHAIN_PROJECT"] = "human_in_the_loop_generation"

from kios_bt.bt_stewardship import BehaviorTreeStewardship
# from kios_scene.scene_factory import SceneFactory # ! scene ceased
from kios_bt.bt_factory import BehaviorTreeFactory
from kios_robot.robot_interface import RobotInterface
from kios_world.world_interface import WorldInterface

from kios_agent.kios_graph import (
    plan_updater,
    planner,
    seq_planner_chain,
    human_instruction_chain,
    human_instruction_chain_v2,
)
from kios_agent.kios_routers import KiosRouterFactory, load_router_from_json

from langgraph.graph import StateGraph, END
from langsmith import traceable

load_dotenv()

from kios_utils.pybt_test import generate_bt_stewardship, render_dot_tree
from kios_utils.bblab_utils import setup_logger

import colorlog
import logging

kios_handler = colorlog.StreamHandler()
kios_handler.setFormatter(
    colorlog.ColoredFormatter(
        "%(log_color)s(AI)---[%(name)s]:%(message)s",
        log_colors={
            "DEBUG": "cyan",
            "INFO": "light_blue",
            "WARNING": "yellow",
            "ERROR": "red",
            "CRITICAL": "red,bg_white",
        },
    )
)

lg_logger = setup_logger("Langgraph", level=logging.INFO)
kios_logger = setup_logger(
    name="KIOS", level=logging.INFO, special_handler=kios_handler
)
time_stamp = datetime.datetime.now().strftime("%Y%m%d%H%M%S")


def render_bt(bt_json: json):
    test_class = BehaviorTreeFactory()
    bt = test_class.from_json_to_simple_bt(bt_json)
    # bt = test_class.from_json_to_tree_root(bt_json)
    bt_stewardship = generate_bt_stewardship(bt)
    # bt_stewardship.setup(timeout=15)
    render_dot_tree(bt_stewardship)


####################### dirs
current_dir = os.path.dirname(os.path.abspath(__file__))
# scene_path = os.path.join(current_dir, "scene.json") # ! scene ceased
# bt_json_file_path = os.path.join(current_dir, "behavior_tree.json")
world_state_path = os.path.join(current_dir, "world_state.json")

####################### scene # ! scene ceased
# with open(scene_path, "r") as file:
    # scene_json_object = json.load(file)

# scene = SceneFactory().create_scene_from_json(scene_json_object) # ! scene ceased

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
# robot_interface.setup_scene(scene) # ! scene ceased

####################### bt_factory
# bt_factory = BehaviorTreeFactory(
#     world_interface=world_interface,
#     robot_interface=robot_interface,
# )

####################### behavior_tree_stewardship
behavior_tree_stewardship = BehaviorTreeStewardship(
    # behaviortree_factory=bt_factory,
    world_interface=world_interface,
    robot_interface=robot_interface,
)

# * kios data prompt skeleton dir
data_dir = os.environ.get("KIOS_DATA_DIR").format(username=os.getlogin()) # TODO change this to your own username
prompt_dir = os.path.join(data_dir, "prompts")


# * the graph state
class PlanExecuteState(TypedDict):
    user_input: str  # this is for assembly planning
    plan: List[str]  # this is the assembly plan

    action_sequence: List[
        str
    ]  # this is the action sequence for only ONE step from the assembly plan
    world_state: Annotated[
        List[dict], operator.add
    ]  # * to add, you need to make world_state a list of dict
    past_steps: Annotated[
        List[Tuple], operator.add
    ]  # this is for update the assembly plan

    last_behavior_tree: dict  # record the last behavior tree for improvement
    last_failed_node: dict  # record the last failed node for improvement
    runtime_world_state: (
        dict  # record the world state after the last execution for diagnosis
    )

    BTExecutionHasSucceeded: bool  # this indicates if the last execution has succeeded


# * the user feedback:
user_feedback: str = None


##################################################### * graph node functions
@traceable(name="user_input_node_step")
def user_input_step(state: PlanExecuteState):
    """
    get the input from the user
    """
    lg_logger.info(f"-----user_input_step-----")

    kios_logger.info(f"Please provide your instructions:")
    user_input = input()

    return {
        "user_input": user_input,
    }


@traceable(name="sequence_generate_step")
def sequence_generate_step(state: PlanExecuteState):
    """
    generate the sequence based on the instruction
    """
    lg_logger.info(f"-----sequence_generate_step-----")

    plan_goal = state["plan"][0]
    start_world_state = state["world_state"][-1]
    global user_feedback

    action_sequence = seq_planner_chain.invoke(
        {
            "start_world_state": start_world_state,
            "task_instruction": plan_goal,
            "user_feedback": user_feedback,
        }
    )

    kios_logger.info(f"{action_sequence}")

    return {
        "action_sequence": action_sequence,
    }


@traceable(name="behavior_tree_generate_step")
def behavior_tree_generate_step(state: PlanExecuteState):
    """
    generate the behavior tree based on the instruction
    """
    lg_logger.info(f"-----behavior_tree_generate_step-----")

    global user_feedback

    bt_skeleton = human_instruction_chain_v2.invoke(
        {
            # "world_state": state["world_state"][-1],
            "user_feedback": user_feedback,
            "last_behavior_tree": state["last_behavior_tree"],
            "action_sequence": state["action_sequence"],
        }
    )

    render_bt(bt_skeleton)

    kios_logger.info(
        f"What should I do to improve the behavior tree? Please give me your hint:"
    )

    user_feedback = input()

    return {
        "last_behavior_tree": bt_skeleton,
        "BTExecutionHasSucceeded": False,
    }


@traceable(name="behavior_tree_execute_step")
def behavior_tree_execute_step(state: PlanExecuteState):
    """
    execute the first step of the plan, append the result to the past steps
    """
    lg_logger.info(f"-----behavior_tree_execute_step-----")
    # * simulation shortcut. Uncomment the following line to use simulation instead of execution
    return behavior_tree_simulation_step(state)
    this_step = state["plan"][0]
    behavior_tree_skeleton = state["last_behavior_tree"]
    latest_world_state = state["world_state"][-1]

    global behavior_tree_stewardship

    tree_result = behavior_tree_stewardship.execute_behavior_tree_skeleton(
        world_state=latest_world_state,
        bt_skeleton=behavior_tree_skeleton,
        scene_json_object=scene_json_object,
        is_simulation=False,
    )

    # behavior_tree_stewardship.set_world_state(latest_world_state)

    # behavior_tree_stewardship.generate_behavior_tree_from_skeleton(
    #     behavior_tree_skeleton
    # )

    # behavior_tree_stewardship.setup_behavior_tree()

    # behavior_tree_stewardship.tick_tree()

    # tree_result = behavior_tree_stewardship.tree_result

    # # ! BB DIRTY FIX
    # behavior_tree_stewardship.refresh_scene_objects(scene_json_object)

    # result_json = tree_result.to_json()
    # lg_logger.info(f"KIOS: Tree execution summary: {tree_result.summary}")
    # pause = input("DEBUG: please check the tree result. Press enter to continue.")

    # * check result
    if tree_result.result == "success":
        kios_logger.info(f"Tree execution succeeded.")

        return {
            "BTExecutionHasSucceeded": True,
            "past_steps": (
                this_step,
                tree_result.result,
            ),  # * only one because new plan will be generated and old steps are all removed
            "world_state": [tree_result.world_state],
            "runtime_world_state": tree_result.world_state,  # * this is world_state for successful execution
        }
    else:
        kios_logger.info(f"Tree execution failed!")
        return {
            "BTExecutionHasSucceeded": False,
            "world_state": [tree_result.world_state],
            "runtime_world_state": tree_result.world_state,
        }


@traceable(name="behavior_tree_simulation_step")
def behavior_tree_simulation_step(state: PlanExecuteState):
    """
    execute the first step of the plan, append the result to the past steps
    """
    lg_logger.info(f"-----behavior_tree_simulation_step-----")

    this_step = state["plan"][0]
    behavior_tree_skeleton = state["last_behavior_tree"]
    latest_world_state = state["world_state"][-1]

    global behavior_tree_stewardship

    behavior_tree_stewardship.set_world_state(latest_world_state)

    behavior_tree_stewardship.generate_behavior_tree_from_skeleton(
        behavior_tree_skeleton
    )

    behavior_tree_stewardship.setup_simulation()

    behavior_tree_stewardship.setup_behavior_tree()

    behavior_tree_stewardship.tick_tree()

    tree_result = behavior_tree_stewardship.tree_result

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
            "runtime_world_state": tree_result.world_state,  # * this is world_state for successful execution
        }
    else:
        return {
            "BTExecutionHasSucceeded": False,
            "world_state": [tree_result.world_state],
            "runtime_world_state": tree_result.world_state,
        }


@traceable(name="planner_step")
def planner_step(state: PlanExecuteState):
    """
    plan the steps based on user input and world state
    """
    lg_logger.info(f"-----plan_step-----")

    plan = planner.invoke(
        {
            "user_input": state["user_input"],
            "world_state": state["world_state"],
        }
    )
    kios_logger.info(f"The plan for your instruction: {plan.steps}")

    return {"plan": plan.steps}


@traceable(name="plan_updater_step")
def plan_updater_step(state: PlanExecuteState):
    """
    if return a response, then success, response the use, end.
    otherwise, return the updated newplan (normally the same as the old plan with the first step popped out.
    """
    lg_logger.info(f"-----plan_updater_step-----")

    output = plan_updater.invoke(
        {
            "user_input": state["user_input"],
            "plan": state["plan"],
            "world_state": state["world_state"],
            "past_steps": state["past_steps"],
        }
    )
    return {
        "plan": output.steps,
        "last_behavior_tree": None,
    }


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

# * factory generate
# user_feedback_router = router_factory.create_router_layer(
#     route_names=["rectify", "approve"]
# )

# * load from json (offline)
user_feedback_router = load_router_from_json("user_feedback_router")


def user_feedback_should_end(state: PlanExecuteState):
    """
    router for user hint.
    """
    lg_logger.debug(f"-----user_feedback_should_end-----")
    global user_feedback

    if user_feedback == "" or not user_feedback:
        return True

    while True:
        route = user_feedback_router(user_feedback)
        if route.name == None:
            user_feedback = input(
                "I don't understand your intention. Can you explain do you want me to execute the plan, or improve the behavior tree?"
            )
        else:
            break
    if route.name == "approve":
        user_feedback = None  # clear the user feedback
        return True  # go to exectuor
    elif route.name == "rectify":
        # keep the feedback
        return False  # go back to generator
    else:
        raise ValueError(f"Route {route.name} not supported!")


workflow.add_conditional_edges(
    "behavior_tree_generator",
    user_feedback_should_end,
    {
        True: "behavior_tree_executor",
        False: "sequence_generator",  # ! BUG 01042024
    },
)


# * load from json (offline)
executor_success_router = load_router_from_json("executor_success_router")

# * load from json (offline)
executor_failure_router = load_router_from_json("executor_failure_router")


def executor_should_end(state: PlanExecuteState):
    """
    end router
    """
    lg_logger.debug(f"-----executor_should_end-----")
    global user_feedback

    if state["BTExecutionHasSucceeded"] == True:
        # ask for user confirmation and end, or go back to the behavior tree generator if the user wants to improve
        kios_logger.info(f"The behavior tree has succeeded.")
        kios_logger.info(
            f"Is the target of this step satisfied now? Is there anything wrong?"
        )
        user_feedback = input()

        if user_feedback == "" or not user_feedback:
            user_feedback = None  # clear the user feedback
            return True

        while True:
            route = executor_success_router(user_feedback)
            if route.name == None:
                kios_logger.info(
                    "I don't understand your intention. Can you explain is the target satisfied, or is there something wrong?"
                )
                user_feedback = input()
            else:
                break

        if route.name in ["finish", "approve"]:
            # * clear the states for this step
            user_feedback = None
            return True
        elif route.name in ["rectify", "disapprove"]:
            return False
        else:
            raise ValueError(f"Route {route.name} not supported!")
    else:
        # ask for user hint and go back to the behavior tree generator
        # * router is unnecessary here
        kios_logger.info(f"The behavior tree has failed.")
        kios_logger.info(
            f"KIOS: Please give me a hint to improve the behavior tree, or tell me what should I do next."
        )
        user_feedback = input()
        if user_feedback == "" or not user_feedback:
            kios_logger.info(
                "I don't understand your instruction if you leave the input empty."
            )
            kios_logger.info("At least you shoud tell me what should I do next.")
            user_feedback = input()

        while True:
            route = executor_failure_router(user_feedback)
            if route.name == None:
                kios_logger.info(
                    "I don't understand your intention. Can you explain is the target satisfied, or is there something wrong?"
                )
                user_feedback = input()
            else:
                break

        if route.name in ["approve"]:
            # * clear the states for this step
            user_feedback = None
            return True
        elif route.name in ["rectify"]:
            return False
        elif route.name in ["retry"]:
            return None
        else:
            raise ValueError(f"Route {route.name} not supported!")


workflow.add_conditional_edges(
    "behavior_tree_executor",
    executor_should_end,
    {
        True: "plan_updater",
        False: "sequence_generator",
        None: "behavior_tree_executor",
    },
)


def plan_updater_should_end(state: PlanExecuteState):
    """
    end router
    """
    lg_logger.debug(f"-----plan_updater_should_end-----")

    if state["plan"] == [] or len(state["plan"]) == 0:
        kios_logger.info("The assembly plan has been finished.")
        return True
    else:
        kios_logger.info("The assembly plan has not been finished.")
        kios_logger.info(f'Unfinished steps: {state["plan"]}')
        return False


workflow.add_conditional_edges(
    "plan_updater",
    plan_updater_should_end,
    {
        True: "user_input_node",
        False: "sequence_generator",
    },
)

# * load from json (offline)
user_input_router = load_router_from_json("user_input_router")


def user_input_should_end(state: PlanExecuteState):
    """
    if the user input is empty, then end
    """
    lg_logger.debug(f"-----user_input_should_end-----")

    if not state["user_input"] or state["user_input"] == "":
        return True

    route = user_input_router(state["user_input"])
    if route.name == None:
        kios_logger.info(
            "I don't understand your instruction. Can you provide me with a new instruction?"
        )
        return None

    elif route.name == "finish":
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
        None: "user_input_node",
    },
)

app = workflow.compile()

config = {"recursion_limit": 500}

inputs = {
    "world_state": [world_state_json],
}


def core_run():
    for event in app.stream(
        inputs,
        config=config,
    ):
        for k, v in event.items():
            if k != "__end__":
                # pprint(v)
                pass


if __name__ == "__main__":
    core_run()
