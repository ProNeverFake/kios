"""
node expanding method script. 
run the algo to generate the tree for current goal, forecast the future world state, and find the new goal to expand.
currently not a langgraph.
"""

import json
import os
from pprint import pprint
from typing import List, Tuple, Annotated, TypedDict
import operator
import logging
import re

os.environ["LANGCHAIN_TRACING_V2"] = "true"
os.environ["LANGCHAIN_ENDPOINT"] = "https://api.smith.langchain.com"
os.environ["LANGCHAIN_PROJECT"] = "recursive_generation_gpt3.5"

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
    rec_ut_generator_chain_gpt3,
    rec_state_predictor_chain_gpt3,
    rec_sequential_action_planner_chain_gpt3,
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
from kios_utils.pddl_problem_parser import parse_problem_init, parse_problem_objects
import datetime

timestamp = datetime.datetime.now().strftime("%Y%m%d%H%M%S")

current_dir = os.path.dirname(os.path.abspath(__file__))

problem_set = os.path.join(current_dir, "baseline_result.jsonl")

with open(problem_set, "r") as f:
    problem_set = f.readlines()
# NEW SHOULD BE 8
problem_number = 7

result_dir = os.path.join(
    current_dir, "recursive_generation_record_gpt3.5", str(problem_number)
)

if not os.path.exists(result_dir):
    os.makedirs(result_dir)

the_problem = json.loads(problem_set[problem_number])

metadata = {
    "method_name": "recursive_generation",
    "try_count": problem_number,
    "timestamp": timestamp,
    "llm": "gpt-3.5-turbo-0125",
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
world_state_path = os.path.join(current_dir, "world_state.json")

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
    world_state: Annotated[
        List[dict], operator.add
    ]  # ! to add, you need to make world_state a list of dict
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
    print(f"-----user_input_step-----")

    user_input = input("Please provide your instructions:\n")

    return {
        "user_input": user_input,
    }


@traceable(name="sequence_generate_step")
def sequence_generate_step(state: PlanExecuteState):
    """
    generate the sequence based on the instruction
    """
    print(f"-----sequence_generate_step-----")

    plan_goal = state["plan"][0]
    start_world_state = state["world_state"][-1]

    action_sequence = seq_planner_chain.invoke(
        {
            "start_world_state": start_world_state,
            "user_instruction": plan_goal,  # TODO the naming method "user_instruction" is confusing. try to change it later.
        }
    )

    return {
        "action_sequence": action_sequence,
    }


@traceable(name="behavior_tree_generate_step")
def behavior_tree_generate_step(state: PlanExecuteState):
    """
    generate the behavior tree based on the instruction
    """
    print(f"-----behavior_tree_generate_step-----")

    global user_feedback

    bt_skeleton = human_instruction_chain.invoke(
        {
            "user_instruction": user_feedback,
            "last_behavior_tree": state["last_behavior_tree"],
            "action_sequence": state["action_sequence"],
        }
    )

    render_bt(bt_skeleton)
    # * here the reason not to use the btw is that the generated bt can be illegal while using btw assumes the bt to be legal and will do assertion.
    # behavior_tree_stewardship.generate_behavior_tree_from_skeleton(bt_skeleton)
    # behavior_tree_stewardship.render_dot_tree()

    user_feedback = input(
        "What should I do to improve the behavior tree?\nPlease give me your hint: "
    )

    return {
        "last_behavior_tree": bt_skeleton,
        "BTExecutionHasSucceeded": False,
    }


@traceable(name="behavior_tree_execute_step")
def behavior_tree_execute_step(state: PlanExecuteState):
    """
    execute the first step of the plan, append the result to the past steps
    """
    print(f"-----behavior_tree_execute_step-----")
    # * simulation shortcut. Uncomment the following line to use simulation instead of execution
    return behavior_tree_simulation_step(state)
    this_step = state["plan"][0]
    behavior_tree_skeleton = state["last_behavior_tree"]
    latest_world_state = state["world_state"][-1]

    behavior_tree_stewardship.set_world_state(latest_world_state)

    behavior_tree_stewardship.generate_behavior_tree_from_skeleton(
        behavior_tree_skeleton
    )

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


@traceable(name="behavior_tree_simulation_step")
def behavior_tree_simulation_step(state: PlanExecuteState):
    """
    execute the first step of the plan, append the result to the past steps
    """
    print(f"-----behavior_tree_simulation_step-----")

    this_step = state["plan"][0]
    behavior_tree_skeleton = state["last_behavior_tree"]
    latest_world_state = state["world_state"][-1]

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
    print(f"-----plan_step-----")

    plan = planner.invoke(
        {
            "user_input": state["user_input"],
            "world_state": state["world_state"],
        }
    )
    return {"plan": plan.steps}


@traceable(name="plan_updater_step")
def plan_updater_step(state: PlanExecuteState):
    """
    if return a response, then success, response the use, end.
    otherwise, return the updated newplan (normally the same as the old plan with the first step popped out.
    """
    print(f"-----plan_updater_step-----")

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

user_feedback_router = router_factory.create_router_layer(
    route_names=["rectify", "approve"]
)


def user_feedback_should_end(state: PlanExecuteState):
    """
    router for user hint.
    """
    print(f"-----user_feedback_should_end-----")
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
    global user_feedback

    if state["BTExecutionHasSucceeded"] == True:
        # ask for user confirmation and end, or go back to the behavior tree generator if the user wants to improve
        user_feedback = input(
            "the behavior tree has succeeded.\n Is the target of this step satisfied now? Is there anything wrong?\n"
        )

        if user_feedback == "" or not user_feedback:
            user_feedback = None  # clear the user feedback
            return True

        while True:
            route = executor_router(user_feedback)
            if route.name == None:
                user_feedback = input(
                    "I don't understand your intention. Can you explain is the target satisfied, or is there something wrong?\n"
                )
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
        user_feedback = input(
            "The behavior tree has failed in its execution.\nPlease give me a hint to improve it:\n"
        )
        return False


workflow.add_conditional_edges(
    "behavior_tree_executor",
    executor_should_end,
    {
        True: "plan_updater",
        False: "sequence_generator",
    },
)


def plan_updater_should_end(state: PlanExecuteState):
    """
    end router
    """
    print(f"-----plan_updater_should_end-----")

    if state["plan"] == [] or len(state["plan"]) == 0:
        print("The assembly plan has been finished.\n")
        return True
    else:
        print("The assembly plan has not been finished.\n")
        print(f'Unfinished steps: {state["plan"]}')
        return False


workflow.add_conditional_edges(
    "plan_updater",
    plan_updater_should_end,
    {
        True: "user_input_node",
        False: "sequence_generator",
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

    route = user_input_router(state["user_input"])
    if route.name == None:
        print(
            "I don't understand your instruction. Can you provide me with a new instruction?\n"
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
                print(v)


################################################################ * from here

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


def seq_planner_est_test():
    return seq_planner_est_chain.invoke(
        {
            "start_world_state": world_state_json,
            # "target": "is_inserted_to(shaft1, gearbase_hole1)",
            "target": "hold(left_hand, outward_claw)",
        }
    )


def ut_gen_test():
    ut = rec_ut_generator_chain_gpt3.invoke(
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
    return rec_sequential_action_planner_chain_gpt3.invoke(
        {
            "start_world_state": world_state_json,
            "target": "is_inserted_to(shaft1, gearbase_hole1)",
            # "target": "hold(left_hand, outward_claw)",
        }
    )


def state_est_test():
    return rec_state_predictor_chain_gpt3.invoke(
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
@traceable(name="rec_seq_plan", metadata=metadata)
def make_plan(state: dict, goal: str) -> list[str]:
    print(f"----------start to make plan for the goal {goal}")
    response = rec_sequential_action_planner_chain_gpt3.invoke(
        {
            "start_world_state": state,
            "target": goal,
        }
    )
    print(f"finished making plan for the goal {goal}.")
    pprint(f'LLM thought flow: {response["explanation"]}')
    return response["task_plan"]


@traceable(name="rec_state_prediction", metadata=metadata)
def estimate_state(start_world_state: dict, action_plan: list[str]) -> dict:
    print("----------start to estimate the state after the action plan:")
    pprint(action_plan)
    response = rec_state_predictor_chain_gpt3.invoke(
        {
            "start_world_state": start_world_state,
            "action_plan": action_plan,
        }
    )
    print(f"finished estimating the state after the action plan {action_plan}.")
    return response["estimated_world_state"]


@traceable(name="rec_unit_subtree", metadata=metadata)
def generate_unit_subtree(action: str) -> dict:
    print("----------start to generate the unit subtree for the action")
    pprint(action)
    response = rec_ut_generator_chain_gpt3.invoke(
        {
            "action": action,
        }
    )
    print(f"finished generating the unit subtree for the action {action}.")
    return response


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


##################################### * speed imp
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
        render_bt(overall_tree[0], dir=result_dir)
    pprint("----------start to expand the node list:")
    pprint(node_list)
    # pause = input("paused here! check the tree.")

    assert len(node_list) > 0
    state = start_state

    for i in range(len(node_list)):
        type_name, body = match_type(node_list[i])
        # if match_type(node_list[i]) == "action":
        if type_name == "action":
            print(f"the node {node_list[i]['name']} is an action node. skip it.")
            # pause = input("paused here! check!")
        elif type_name in ["precondition", "target"]:
            # goal = node_list[i]["name"]
            goal = body
            plan = make_plan(state, goal)
            if len(plan) == 0:
                logging.warning(f"No action should be performed for the goal {goal}.")
                logging.warning(f'the node {node_list[i]["name"]} has been skipped.')
                # pause = input("paused here! check!")
            else:
                logging.info(f"Actions have been planed for the goal {goal}.")
                pprint(f"the plan for the goal {goal} is {plan}")
                # pause = input("paused here! check!")
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


def test_expand_nodes():
    start_state = the_problem["initial_world_state"]
    node_list = [
        {
            "summary": the_problem["target"],
            "name": the_problem["target"],
        }
    ]

    # start_state = world_state_json
    # node_list = [
    #     {
    #         "summary": "insert shaft1 into gearbase hole1",
    #         "name": "target: insert shaft1 into gearbase hole1",
    #     }
    # ]
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
    try:
        expand_nodes(node_list, start_state, node_list)
    except KeyboardInterrupt:
        print("The execution has been interrupted.")
        tree_result = {
            "result": "error",
            "summary": "endless loop in generation",
            "world_state": start_state,
            "final_node": None,
        }
        write_result(bt_json=node_list[0], dir=result_dir, tree_result=tree_result)
        exit(0)

    # pprint("----------check the entire tree:")
    render_bt(node_list[0], dir=result_dir)

    tree_result = behavior_tree_simulation(node_list[0], start_state)

    write_result(bt_json=node_list[0], dir=result_dir, tree_result=tree_result)
    # pprint(node_list)


if __name__ == "__main__":

    test_expand_nodes()
    # print(the_problem["target"])
    # print(the_problem["initial_world_state"])
